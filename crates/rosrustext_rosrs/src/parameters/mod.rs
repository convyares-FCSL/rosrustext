//! rosrustext_rosrs::parameters
//!
//! ROS-facing parameter services + parameter_events publisher backed by
//! rosrustext_core::parameters semantics.

use std::sync::{Arc, Mutex};

use rclrs::{Executor, IntoNodeOptions, Node};
use rosrustext_core::parameters::ParameterStore;

use crate::error::Result;

mod events;
mod services;
mod watcher;

pub use rosrustext_core::parameters::{Descriptor, EventRecord, ListResult, SetResult, Type, Value};
pub use watcher::{ParameterChange, ParameterWatcher};

#[derive(Clone)]
pub struct ParameterNode {
    node: Arc<Node>,
    store: Arc<Mutex<ParameterStore>>,
    events: Arc<events::ParameterEventsPublisher>,
    internals: Arc<Mutex<Vec<Box<dyn std::any::Any + Send + Sync>>>>,
}

impl ParameterNode {
    /// Primary constructor: create a node on the executor and enable parameter services.
    pub fn create<'a>(executor: &'a Executor, options: impl IntoNodeOptions<'a>) -> Result<Self> {
        let options = options.into_node_options().start_parameter_services(false);
        let node = Arc::new(executor.create_node(options)?);
        Self::try_new(node)
    }

    /// Advanced constructor: wraps an existing node with parameter services.
    pub fn try_new(node: Arc<Node>) -> Result<Self> {
        Self::try_new_with_allow_undeclared(node, false)
    }

    /// Advanced constructor with explicit undeclared behavior.
    pub fn try_new_with_allow_undeclared(node: Arc<Node>, allow_undeclared: bool) -> Result<Self> {
        let store = Arc::new(Mutex::new(ParameterStore::new(allow_undeclared)));
        let events = Arc::new(events::ParameterEventsPublisher::new(&node)?);
        let internals = Arc::new(Mutex::new(Vec::new()));
        let pn = Self { node, store, events, internals };
        pn.enable_defaults()?;
        Ok(pn)
    }

    pub fn declare(&self, name: &str, ty: Type, default: Value, descriptor: Descriptor) -> Result<()> {
        let mut store = self.store.lock().expect("parameter store mutex poisoned");
        store.declare(name, ty, default, descriptor)?;
        Ok(())
    }

    pub fn store(&self) -> Arc<Mutex<ParameterStore>> {
        Arc::clone(&self.store)
    }

    pub fn node(&self) -> &Arc<Node> {
        &self.node
    }

    fn enable_defaults(&self) -> Result<()> {
        services::enable_services(
            &self.node,
            Arc::clone(&self.store),
            Arc::clone(&self.events),
            Arc::clone(&self.internals),
        )?;
        self.keep_internal(self.events.publisher());
        Ok(())
    }

    fn keep_internal<T>(&self, handle: T)
    where
        T: Send + Sync + 'static,
    {
        self.internals.lock().expect("ParameterNode internals poisoned").push(Box::new(handle));
    }
}

fn to_ros_type(ty: Type) -> u8 {
    match ty {
        Type::NotSet => rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET,
        Type::Bool => rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
        Type::Integer => rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
        Type::Double => rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
        Type::String => rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
        Type::ByteArray => rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY,
        Type::BoolArray => rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY,
        Type::IntegerArray => rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY,
        Type::DoubleArray => rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY,
        Type::StringArray => rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
    }
}

fn to_ros_value(value: &Value) -> rcl_interfaces::msg::ParameterValue {
    let mut msg = rcl_interfaces::msg::ParameterValue::default();
    match value {
        Value::NotSet => {
            msg.type_ = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
        }
        Value::Bool(v) => {
            msg.type_ = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            msg.bool_value = *v;
        }
        Value::Integer(v) => {
            msg.type_ = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            msg.integer_value = *v;
        }
        Value::Double(v) => {
            msg.type_ = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            msg.double_value = *v;
        }
        Value::String(v) => {
            msg.type_ = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            msg.string_value = v.clone();
        }
        Value::ByteArray(v) => {
            msg.type_ = rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY;
            msg.byte_array_value = v.clone();
        }
        Value::BoolArray(v) => {
            msg.type_ = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
            msg.bool_array_value = v.clone();
        }
        Value::IntegerArray(v) => {
            msg.type_ = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
            msg.integer_array_value = v.clone();
        }
        Value::DoubleArray(v) => {
            msg.type_ = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
            msg.double_array_value = v.clone();
        }
        Value::StringArray(v) => {
            msg.type_ = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            msg.string_array_value = v.clone();
        }
    }
    msg
}

pub(crate) fn from_ros_value(value: &rcl_interfaces::msg::ParameterValue) -> std::result::Result<Value, String> {
    match value.type_ {
        t if t == rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET => Ok(Value::NotSet),
        t if t == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL => Ok(Value::Bool(value.bool_value)),
        t if t == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER => Ok(Value::Integer(value.integer_value)),
        t if t == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE => Ok(Value::Double(value.double_value)),
        t if t == rcl_interfaces::msg::ParameterType::PARAMETER_STRING => Ok(Value::String(value.string_value.clone())),
        t if t == rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY => Ok(Value::ByteArray(value.byte_array_value.clone())),
        t if t == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY => Ok(Value::BoolArray(value.bool_array_value.clone())),
        t if t == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY => {
            Ok(Value::IntegerArray(value.integer_array_value.clone()))
        }
        t if t == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY => {
            Ok(Value::DoubleArray(value.double_array_value.clone()))
        }
        t if t == rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY => {
            Ok(Value::StringArray(value.string_array_value.clone()))
        }
        _ => Err("unknown parameter type".to_string()),
    }
}

fn to_ros_descriptor(
    name: &str,
    ty: Type,
    descriptor: &Descriptor,
) -> rcl_interfaces::msg::ParameterDescriptor {
    let mut msg = rcl_interfaces::msg::ParameterDescriptor::default();
    msg.name = name.to_string();
    msg.type_ = to_ros_type(ty);
    msg.description = descriptor.description.clone();
    msg.additional_constraints = descriptor.additional_constraints.clone();
    msg.read_only = descriptor.read_only;
    msg.dynamic_typing = descriptor.dynamic_typing;
    msg
}

fn to_ros_parameter(param: rosrustext_core::parameters::Parameter) -> rcl_interfaces::msg::Parameter {
    let rosrustext_core::parameters::Parameter { name, value } = param;
    rcl_interfaces::msg::Parameter { name, value: to_ros_value(&value) }
}

fn to_ros_set_result(result: &rosrustext_core::parameters::SetResult) -> rcl_interfaces::msg::SetParametersResult {
    rcl_interfaces::msg::SetParametersResult {
        successful: result.success,
        reason: result.reason.clone().unwrap_or_default(),
    }
}
