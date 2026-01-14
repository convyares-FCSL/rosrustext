use std::collections::{BTreeMap, BTreeSet};

use crate::error::{CoreError, Domain, ErrorKind, Result};

use super::types::{
    Descriptor, DescribedParameter, EventRecord, ListResult, Parameter, SetResult, Type, Value,
};

#[derive(Debug)]
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
    pub fn new(allow_undeclared: bool) -> Self {
        Self { allow_undeclared, parameters: BTreeMap::new() }
    }

    pub fn allow_undeclared(&self) -> bool {
        self.allow_undeclared
    }

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

        self.parameters.insert(
            name.to_string(),
            ParameterEntry {
                ty,
                value: default,
                descriptor,
                declared: true,
            },
        );
        Ok(())
    }

    pub fn get(&self, names: &[String]) -> Vec<Value> {
        names
            .iter()
            .map(|name| self.parameters.get(name).map(|entry| entry.value.clone()).unwrap_or(Value::NotSet))
            .collect()
    }

    pub fn get_types(&self, names: &[String]) -> Vec<Type> {
        names
            .iter()
            .map(|name| self.parameters.get(name).map(|entry| entry.ty).unwrap_or(Type::NotSet))
            .collect()
    }

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
                    let suffix = &name[prefix.len()..];
                    if suffix.starts_with('.') {
                        let suffix = &suffix[1..];
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

    pub fn describe(&self, names: &[String]) -> Vec<DescribedParameter> {
        names
            .iter()
            .map(|name| {
                if let Some(entry) = self.parameters.get(name) {
                    DescribedParameter {
                        name: name.clone(),
                        ty: entry.ty,
                        descriptor: entry.descriptor.clone(),
                    }
                } else {
                    DescribedParameter {
                        name: name.clone(),
                        ty: Type::NotSet,
                        descriptor: Descriptor::default(),
                    }
                }
            })
            .collect()
    }

    pub fn set_parameter(&mut self, name: String, value: Value) -> (SetResult, EventRecord) {
        match self.prepare_set(&name, &value) {
            Ok(prepared) => {
                let record = self.apply_prepared(prepared);
                (SetResult::ok(), record)
            }
            Err(reason) => (SetResult::err(reason), EventRecord::default()),
        }
    }

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
            return Ok(PreparedSet {
                name: name.to_string(),
                value: value.clone(),
                ty,
                is_new: false,
                changed,
            });
        }

        if !self.allow_undeclared {
            return Err("undeclared parameter".to_string());
        }

        let ty = value.ty();
        if ty == Type::NotSet {
            return Err("cannot declare NOT_SET parameter".to_string());
        }

        Ok(PreparedSet {
            name: name.to_string(),
            value: value.clone(),
            ty,
            is_new: true,
            changed: true,
        })
    }

    fn apply_prepared(&mut self, prepared: PreparedSet) -> EventRecord {
        let descriptor = if prepared.is_new {
            Descriptor {
                dynamic_typing: true,
                ..Descriptor::default()
            }
        } else {
            self.parameters
                .get(&prepared.name)
                .map(|entry| entry.descriptor.clone())
                .unwrap_or_default()
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
