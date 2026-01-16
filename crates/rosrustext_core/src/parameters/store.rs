use std::collections::{BTreeMap, BTreeSet};

use crate::error::{CoreError, Domain, ErrorKind, Result};

use super::types::{DescribedParameter, Descriptor, EventRecord, ListResult, Parameter, SetResult, Type, Value};

#[derive(Debug)]
/// In-memory parameter store implementing the core parameter semantics.
///
/// # Semantics
/// `ParameterStore` models a node's parameter set keyed by name. It supports:
/// - declared-only mode (`allow_undeclared=false`)
/// - allow-undeclared mode (`allow_undeclared=true`)
///
/// It enforces:
/// - `read_only` descriptor semantics (updates rejected)
/// - `dynamic_typing` semantics (type changes allowed when enabled)
///
/// It does **not** support deletion: attempting to set a parameter to
/// [`Value::NotSet`] is rejected.
///
/// Set operations return both:
/// - a [`SetResult`] (or vector of results), and
/// - an [`EventRecord`] describing accepted changes for emission on
///   `parameter_events`.
///
/// # Threading
/// `ParameterStore` itself is not synchronized. Adapter layers are expected to
/// wrap it in a lock (e.g. `Arc<Mutex<ParameterStore>>`) when serving requests
/// concurrently.
///
/// # Example
/// ```rust
/// use rosrustext_core::parameters::{Descriptor, ParameterStore, Type, Value};
///
/// let mut store = ParameterStore::new(false);
/// store.declare("threshold", Type::Double, Value::Double(0.5), Descriptor::default()).unwrap();
///
/// let (result, record) = store.set_parameters_atomically(vec![("threshold".into(), Value::Double(1.0))]);
/// assert!(result.success);
/// assert!(!record.changed_parameters.is_empty());
/// ```
///
/// # See also
/// - [Parameters spec](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/parameters.md)
/// - [`Value`]
/// - [`Type`]
/// - [`Descriptor`]
pub struct ParameterStore {
    allow_undeclared: bool,
    parameters: BTreeMap<String, ParameterEntry>,
}

#[derive(Debug, Clone)]
struct ParameterEntry {
    ty: Type,
    value: Value,
    descriptor: Descriptor,
    declared: bool,
}

#[derive(Debug)]
struct PreparedSet {
    name: String,
    value: Value,
    ty: Type,
    is_new: bool,
    changed: bool,
}

impl ParameterStore {
    /// Create a new store.
    ///
    /// # Semantics
    /// - If `allow_undeclared=false`, setting an undeclared parameter is rejected.
    /// - If `allow_undeclared=true`, setting an undeclared parameter creates it.
    ///
    /// # Threading
    /// `ParameterStore` is not synchronized; wrap it in a lock when shared.
    ///
    /// # Example
    /// ```rust
    /// use rosrustext_core::parameters::ParameterStore;
    /// let store = ParameterStore::new(false);
    /// assert!(!store.allow_undeclared());
    /// ```
    ///
    /// # See also
    /// - [`ParameterStore::allow_undeclared`]
    pub fn new(allow_undeclared: bool) -> Self {
        Self { allow_undeclared, parameters: BTreeMap::new() }
    }

    /// Return whether undeclared parameters may be created via set operations.
    ///
    /// # Semantics
    /// See [`ParameterStore::new`].
    ///
    /// # Threading
    /// Immutable read.
    ///
    /// # Example
    /// ```rust
    /// use rosrustext_core::parameters::ParameterStore;
    /// assert!(ParameterStore::new(true).allow_undeclared());
    /// ```
    ///
    /// # See also
    /// - [`ParameterStore::new`]
    pub fn allow_undeclared(&self) -> bool {
        self.allow_undeclared
    }

    /// Declare a parameter with a fixed type, default value, and descriptor.
    ///
    /// # Semantics
    /// - Declaring the same name twice is rejected.
    /// - `ty` must match `default.ty()`, and `ty` must not be [`Type::NotSet`].
    ///
    /// # Threading
    /// Requires exclusive access (`&mut self`).
    ///
    /// # Example
    /// ```rust
    /// use rosrustext_core::parameters::{Descriptor, ParameterStore, Type, Value};
    /// let mut store = ParameterStore::new(false);
    /// store.declare("x", Type::Integer, Value::Integer(1), Descriptor::default()).unwrap();
    /// ```
    ///
    /// # See also
    /// - [`Descriptor`]
    /// - [`Type`]
    pub fn declare(&mut self, name: &str, ty: Type, default: Value, descriptor: Descriptor) -> Result<()> {
        if self.parameters.contains_key(name) {
            return Err(CoreError::warn()
                .domain(Domain::Config)
                .kind(ErrorKind::InvalidState)
                .msg("parameter already declared")
                .build());
        }
        if ty != default.ty() || ty == Type::NotSet {
            return Err(CoreError::warn()
                .domain(Domain::Config)
                .kind(ErrorKind::InvalidArgument)
                .msg("default value does not match declared type")
                .build());
        }

        self.parameters.insert(name.to_string(), ParameterEntry { ty, value: default, descriptor, declared: true });
        Ok(())
    }

    /// Get values for a list of names.
    ///
    /// # Semantics
    /// - Returns one [`Value`] per requested name.
    /// - Unknown names yield [`Value::NotSet`].
    ///
    /// # Threading
    /// Immutable read.
    ///
    /// # Example
    /// ```rust
    /// use rosrustext_core::parameters::{ParameterStore, Value};
    /// let store = ParameterStore::new(false);
    /// let vals = store.get(&vec!["missing".to_string()]);
    /// assert_eq!(vals, vec![Value::NotSet]);
    /// ```
    ///
    /// # See also
    /// - [`ParameterStore::get_types`]
    pub fn get(&self, names: &[String]) -> Vec<Value> {
        names
            .iter()
            .map(|name| self.parameters.get(name).map(|entry| entry.value.clone()).unwrap_or(Value::NotSet))
            .collect()
    }

    /// Get types for a list of names.
    ///
    /// # Semantics
    /// - Returns one [`Type`] per requested name.
    /// - Unknown names yield [`Type::NotSet`].
    ///
    /// # Threading
    /// Immutable read.
    ///
    /// # Example
    /// ```rust
    /// use rosrustext_core::parameters::{ParameterStore, Type};
    /// let store = ParameterStore::new(false);
    /// let tys = store.get_types(&vec!["missing".to_string()]);
    /// assert_eq!(tys, vec![Type::NotSet]);
    /// ```
    ///
    /// # See also
    /// - [`ParameterStore::get`]
    pub fn get_types(&self, names: &[String]) -> Vec<Type> {
        names.iter().map(|name| self.parameters.get(name).map(|entry| entry.ty).unwrap_or(Type::NotSet)).collect()
    }

    /// List parameter names matching prefixes and depth.
    ///
    /// # Semantics
    /// Implements ROS 2 list semantics:
    /// - `prefixes=[]` lists all parameters.
    /// - `depth=0` means “recursive” (no depth limit).
    ///
    /// Returns both matched names and derived prefixes.
    ///
    /// # Threading
    /// Immutable read.
    ///
    /// # Example
    /// ```rust
    /// use rosrustext_core::parameters::{Descriptor, ParameterStore, Type, Value};
    /// let mut store = ParameterStore::new(false);
    /// store.declare("a.b", Type::Integer, Value::Integer(1), Descriptor::default()).unwrap();
    /// let list = store.list(&[], 0);
    /// assert!(list.names.contains(&"a.b".to_string()));
    /// ```
    ///
    /// # See also
    /// - [`ListResult`]
    pub fn list(&self, prefixes: &[String], depth: u64) -> ListResult {
        let mut names = Vec::new();
        let depth_recursive = depth == 0;
        for name in self.parameters.keys() {
            if prefixes.is_empty() {
                if depth_recursive || depth_ok(name, depth) {
                    names.push(name.clone());
                }
                continue;
            }

            let mut matched = false;
            for prefix in prefixes {
                if name == prefix {
                    matched = true;
                    break;
                }
                if name.starts_with(prefix) && name.len() > prefix.len() {
                    if let Some(suffix) = name.strip_prefix(prefix).and_then(|suffix| suffix.strip_prefix('.')) {
                        if depth_recursive || depth_ok(suffix, depth) {
                            matched = true;
                            break;
                        }
                    }
                }
            }
            if matched {
                names.push(name.clone());
            }
        }

        let mut prefix_set = BTreeSet::new();
        for name in &names {
            if let Some(pos) = name.rfind('.') {
                prefix_set.insert(name[..pos].to_string());
            }
        }

        ListResult { names, prefixes: prefix_set.into_iter().collect() }
    }

    /// Describe parameters by name.
    ///
    /// # Semantics
    /// - For known parameters, returns the stored type and descriptor.
    /// - For unknown parameters, returns `Type::NotSet` with a default descriptor.
    ///
    /// # Threading
    /// Immutable read.
    ///
    /// # Example
    /// ```rust
    /// use rosrustext_core::parameters::{ParameterStore};
    /// let store = ParameterStore::new(false);
    /// let described = store.describe(&vec!["missing".to_string()]);
    /// assert_eq!(described[0].name, "missing");
    /// ```
    ///
    /// # See also
    /// - [`Descriptor`]
    pub fn describe(&self, names: &[String]) -> Vec<DescribedParameter> {
        names
            .iter()
            .map(|name| {
                if let Some(entry) = self.parameters.get(name) {
                    DescribedParameter { name: name.clone(), ty: entry.ty, descriptor: entry.descriptor.clone() }
                } else {
                    DescribedParameter { name: name.clone(), ty: Type::NotSet, descriptor: Descriptor::default() }
                }
            })
            .collect()
    }

    /// Set a single parameter value.
    ///
    /// # Semantics
    /// Applies the same rules as the ROS-facing services:
    /// - undeclared handling depends on `allow_undeclared`
    /// - `read_only` and type rules are enforced
    /// - deletion via `Value::NotSet` is rejected
    ///
    /// Returns a `(SetResult, EventRecord)` pair.
    ///
    /// # Threading
    /// Requires exclusive access (`&mut self`).
    ///
    /// # Example
    /// ```rust
    /// use rosrustext_core::parameters::{ParameterStore, Value};
    /// let mut store = ParameterStore::new(true);
    /// let (res, _ev) = store.set_parameter("x".into(), Value::Integer(1));
    /// assert!(res.success);
    /// ```
    ///
    /// # See also
    /// - [`ParameterStore::set_parameters`]
    pub fn set_parameter(&mut self, name: String, value: Value) -> (SetResult, EventRecord) {
        match self.prepare_set(&name, &value) {
            Ok(prepared) => {
                let record = self.apply_prepared(prepared);
                (SetResult::ok(), record)
            }
            Err(reason) => (SetResult::err(reason), EventRecord::default()),
        }
    }

    /// Set multiple parameters non-atomically.
    ///
    /// # Semantics
    /// Each parameter is attempted independently; partial success is allowed.
    /// Returns `(Vec<SetResult>, EventRecord)` where the event record contains
    /// only successful changes.
    ///
    /// # Threading
    /// Requires exclusive access (`&mut self`).
    ///
    /// # Example
    /// ```rust
    /// use rosrustext_core::parameters::{ParameterStore, Value};
    /// let mut store = ParameterStore::new(true);
    /// let (results, _ev) = store.set_parameters(vec![
    ///     ("a".into(), Value::Integer(1)),
    ///     ("b".into(), Value::String("x".into())),
    /// ]);
    /// assert_eq!(results.len(), 2);
    /// ```
    ///
    /// # See also
    /// - [`ParameterStore::set_parameters_atomically`]
    pub fn set_parameters(&mut self, params: Vec<(String, Value)>) -> (Vec<SetResult>, EventRecord) {
        let mut results = Vec::with_capacity(params.len());
        let mut record = EventRecord::default();

        for (name, value) in params {
            let (result, event) = self.set_parameter(name, value);
            results.push(result);
            record.merge(event);
        }

        (results, record)
    }

    /// Set multiple parameters atomically (all-or-nothing).
    ///
    /// # Semantics
    /// - If any update would be rejected, no updates are applied.
    /// - On success, all updates are applied and the returned [`EventRecord`]
    ///   contains all changes.
    ///
    /// # Threading
    /// Requires exclusive access (`&mut self`).
    ///
    /// # Example
    /// ```rust
    /// use rosrustext_core::parameters::{ParameterStore, Value};
    /// let mut store = ParameterStore::new(true);
    /// let (res, _ev) = store.set_parameters_atomically(vec![("x".into(), Value::Integer(1))]);
    /// assert!(res.success);
    /// ```
    ///
    /// # See also
    /// - [Parameters spec: SetParametersAtomically](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/parameters.md#setparametersatomically-atomic)
    pub fn set_parameters_atomically(&mut self, params: Vec<(String, Value)>) -> (SetResult, EventRecord) {
        let mut prepared = Vec::with_capacity(params.len());
        for (name, value) in params {
            match self.prepare_set(&name, &value) {
                Ok(set) => prepared.push(set),
                Err(reason) => return (SetResult::err(reason), EventRecord::default()),
            }
        }

        let mut record = EventRecord::default();
        for set in prepared {
            record.merge(self.apply_prepared(set));
        }

        (SetResult::ok(), record)
    }

    fn prepare_set(&self, name: &str, value: &Value) -> std::result::Result<PreparedSet, String> {
        if matches!(value, Value::NotSet) {
            return Err("parameter deletion not supported".to_string());
        }

        if let Some(entry) = self.parameters.get(name) {
            if entry.descriptor.read_only {
                return Err("read-only parameter".to_string());
            }
            if !entry.descriptor.dynamic_typing && entry.ty != value.ty() {
                return Err("parameter type mismatch".to_string());
            }
            let changed = entry.value != value.clone();
            let ty = if entry.descriptor.dynamic_typing { value.ty() } else { entry.ty };
            return Ok(PreparedSet { name: name.to_string(), value: value.clone(), ty, is_new: false, changed });
        }

        if !self.allow_undeclared {
            return Err("undeclared parameter".to_string());
        }

        let ty = value.ty();
        if ty == Type::NotSet {
            return Err("cannot declare NOT_SET parameter".to_string());
        }

        Ok(PreparedSet { name: name.to_string(), value: value.clone(), ty, is_new: true, changed: true })
    }

    fn apply_prepared(&mut self, prepared: PreparedSet) -> EventRecord {
        let descriptor = if prepared.is_new {
            Descriptor { dynamic_typing: true, ..Descriptor::default() }
        } else {
            self.parameters.get(&prepared.name).map(|entry| entry.descriptor.clone()).unwrap_or_default()
        };

        let entry = self.parameters.entry(prepared.name.clone()).or_insert(ParameterEntry {
            ty: prepared.ty,
            value: prepared.value.clone(),
            descriptor: descriptor.clone(),
            declared: false,
        });
        entry.value = prepared.value.clone();
        entry.ty = prepared.ty;
        if prepared.is_new {
            entry.declared = false;
            entry.descriptor = descriptor;
        }

        if !prepared.changed {
            return EventRecord::default();
        }

        let param = Parameter { name: prepared.name, value: prepared.value };
        if prepared.is_new {
            EventRecord { new_parameters: vec![param], changed_parameters: Vec::new(), deleted_parameters: Vec::new() }
        } else {
            EventRecord { new_parameters: Vec::new(), changed_parameters: vec![param], deleted_parameters: Vec::new() }
        }
    }
}

fn depth_ok(name: &str, depth: u64) -> bool {
    if depth == 0 {
        return true;
    }
    let count = name.matches('.').count() as u64;
    count < depth
}
