/// A ROS-style parameter value.
///
/// # Semantics
/// This enum represents the value payload for a ROS 2 parameter, matching the
/// canonical ROS parameter type set.
///
/// Notes:
/// - [`Value::NotSet`] is used by ROS tooling to represent unknown parameters.
/// - In this project, attempting to *delete* a parameter by setting `NotSet` is
///   rejected by the core store (see [`crate::parameters::ParameterStore`]).
///
/// # Threading
/// This is a plain owned value. It has no interior mutability and is safe to
/// move across threads.
///
/// # Example
/// ```rust
/// use rosrustext_core::parameters::{Type, Value};
///
/// let v = Value::Integer(42);
/// assert_eq!(v.ty(), Type::Integer);
/// ```
///
/// # See also
/// - [`Type`]
/// - [`crate::parameters::ParameterStore`]
/// - [Parameters spec](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/docs/spec/parameters.md)
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    NotSet,
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    ByteArray(Vec<u8>),
    BoolArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
}

impl Value {
    /// Return the corresponding [`Type`] for this value.
    pub fn ty(&self) -> Type {
        match self {
            Value::NotSet => Type::NotSet,
            Value::Bool(_) => Type::Bool,
            Value::Integer(_) => Type::Integer,
            Value::Double(_) => Type::Double,
            Value::String(_) => Type::String,
            Value::ByteArray(_) => Type::ByteArray,
            Value::BoolArray(_) => Type::BoolArray,
            Value::IntegerArray(_) => Type::IntegerArray,
            Value::DoubleArray(_) => Type::DoubleArray,
            Value::StringArray(_) => Type::StringArray,
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
/// A ROS-style parameter type discriminator.
///
/// # Semantics
/// This is the canonical ROS 2 parameter type set (mirrors
/// `rcl_interfaces/msg/ParameterType`).
///
/// [`Type::NotSet`] is used to represent unknown parameters and “no value”.
///
/// # Threading
/// `Type` is `Copy` and has no interior mutability.
///
/// # Example
/// ```rust
/// use rosrustext_core::parameters::Type;
/// assert_eq!(Type::Double as u8, Type::Double as u8);
/// ```
///
/// # See also
/// - [`Value`]
/// - [Parameters spec](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/docs/spec/parameters.md)
pub enum Type {
    NotSet,
    Bool,
    Integer,
    Double,
    String,
    ByteArray,
    BoolArray,
    IntegerArray,
    DoubleArray,
    StringArray,
}

#[derive(Debug, Clone)]
/// Metadata describing a parameter.
///
/// # Semantics
/// This struct is a small subset of `rcl_interfaces/msg/ParameterDescriptor`:
/// - `description` and `additional_constraints` are user-facing documentation.
/// - `read_only=true` causes set requests to be rejected.
/// - `dynamic_typing=true` allows the parameter type to change on set.
///
/// Exact observable behavior is defined by the adapter layer that exposes ROS
/// services (e.g. `rosrustext_rosrs::parameters`).
///
/// # Threading
/// This is an owned value with no interior mutability.
///
/// # Example
/// ```rust
/// use rosrustext_core::parameters::Descriptor;
/// let d = Descriptor { read_only: true, ..Descriptor::default() };
/// assert!(d.read_only);
/// ```
///
/// # See also
/// - [`crate::parameters::ParameterStore`]
/// - [Parameters spec](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/docs/spec/parameters.md)
#[derive(Default)]
pub struct Descriptor {
    pub description: String,
    pub additional_constraints: String,
    pub read_only: bool,
    pub dynamic_typing: bool,
}

#[derive(Debug, Clone)]
/// A named parameter.
///
/// # Semantics
/// This pairs a parameter name with a [`Value`].
///
/// # Threading
/// Plain owned data.
///
/// # Example
/// ```rust
/// use rosrustext_core::parameters::{Parameter, Value};
/// let p = Parameter { name: "x".into(), value: Value::Integer(1) };
/// assert_eq!(p.name, "x");
/// ```
///
/// # See also
/// - [`Value`]
pub struct Parameter {
    pub name: String,
    pub value: Value,
}

#[derive(Debug, Clone)]
/// Result of a parameter set attempt.
///
/// # Semantics
/// - `success=true` indicates the update was applied.
/// - `success=false` indicates the update was rejected and no change was applied.
/// - `reason` may contain a human-readable failure reason.
///
/// This maps naturally to `rcl_interfaces/msg/SetParametersResult`.
///
/// # Threading
/// Plain owned data.
///
/// # Example
/// ```rust
/// use rosrustext_core::parameters::SetResult;
/// assert!(SetResult::ok().success);
/// assert!(!SetResult::err("no").success);
/// ```
///
/// # See also
/// - [Parameters spec: SetParameters](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/docs/spec/parameters.md#setparameters-non-atomic)
pub struct SetResult {
    pub success: bool,
    pub reason: Option<String>,
}

impl SetResult {
    pub fn ok() -> Self {
        Self { success: true, reason: None }
    }

    pub fn err(reason: impl Into<String>) -> Self {
        Self { success: false, reason: Some(reason.into()) }
    }
}

#[derive(Debug, Clone, Default)]
/// A set of parameter changes suitable for emitting as a `parameter_events` message.
///
/// # Semantics
/// - `new_parameters` are parameters that were newly created.
/// - `changed_parameters` are parameters whose values changed.
/// - `deleted_parameters` are parameters that were deleted.
///
/// # Threading
/// Plain owned data.
///
/// # Example
/// ```rust
/// use rosrustext_core::parameters::EventRecord;
/// assert!(EventRecord::default().is_empty());
/// ```
///
/// # See also
/// - [Parameters spec: Parameter events](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/docs/spec/parameters.md#parameter-events)
pub struct EventRecord {
    pub new_parameters: Vec<Parameter>,
    pub changed_parameters: Vec<Parameter>,
    pub deleted_parameters: Vec<Parameter>,
}

impl EventRecord {
    pub fn is_empty(&self) -> bool {
        self.new_parameters.is_empty() && self.changed_parameters.is_empty() && self.deleted_parameters.is_empty()
    }

    pub fn merge(&mut self, other: EventRecord) {
        self.new_parameters.extend(other.new_parameters);
        self.changed_parameters.extend(other.changed_parameters);
        self.deleted_parameters.extend(other.deleted_parameters);
    }
}

#[derive(Debug, Clone)]
/// Result of a parameter name listing operation.
///
/// # Semantics
/// - `names` are fully qualified parameter names matching the query.
/// - `prefixes` are hierarchical prefixes derived from `names` (ROS 2 list semantics).
///
/// # Threading
/// Plain owned data.
///
/// # Example
/// ```rust
/// use rosrustext_core::parameters::ListResult;
/// let r = ListResult { names: vec!["a.b".into()], prefixes: vec!["a".into()] };
/// assert_eq!(r.prefixes, vec!["a"]);
/// ```
///
/// # See also
/// - [Parameters spec: ListParameters](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/docs/spec/parameters.md#listparameters)
pub struct ListResult {
    pub names: Vec<String>,
    pub prefixes: Vec<String>,
}

#[derive(Debug, Clone)]
/// A parameter descriptor plus resolved type.
///
/// # Semantics
/// Returned by `describe`-style operations.
///
/// # Threading
/// Plain owned data.
///
/// # Example
/// ```rust
/// use rosrustext_core::parameters::{DescribedParameter, Descriptor, Type};
/// let d = DescribedParameter { name: "x".into(), ty: Type::Integer, descriptor: Descriptor::default() };
/// assert_eq!(d.name, "x");
/// ```
///
/// # See also
/// - [`Descriptor`]
pub struct DescribedParameter {
    pub name: String,
    pub ty: Type,
    pub descriptor: Descriptor,
}
