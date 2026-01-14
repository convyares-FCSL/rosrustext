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
pub struct Descriptor {
    pub description: String,
    pub additional_constraints: String,
    pub read_only: bool,
    pub dynamic_typing: bool,
}

impl Default for Descriptor {
    fn default() -> Self {
        Self {
            description: String::new(),
            additional_constraints: String::new(),
            read_only: false,
            dynamic_typing: false,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Parameter {
    pub name: String,
    pub value: Value,
}

#[derive(Debug, Clone)]
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
pub struct EventRecord {
    pub new_parameters: Vec<Parameter>,
    pub changed_parameters: Vec<Parameter>,
    pub deleted_parameters: Vec<Parameter>,
}

impl EventRecord {
    pub fn is_empty(&self) -> bool {
        self.new_parameters.is_empty()
            && self.changed_parameters.is_empty()
            && self.deleted_parameters.is_empty()
    }

    pub fn merge(&mut self, other: EventRecord) {
        self.new_parameters.extend(other.new_parameters);
        self.changed_parameters.extend(other.changed_parameters);
        self.deleted_parameters.extend(other.deleted_parameters);
    }
}

#[derive(Debug, Clone)]
pub struct ListResult {
    pub names: Vec<String>,
    pub prefixes: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct DescribedParameter {
    pub name: String,
    pub ty: Type,
    pub descriptor: Descriptor,
}
