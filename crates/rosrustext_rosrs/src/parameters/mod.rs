//! ROS 2 parameters (adapter-owned services + event watcher for `rclrs`).
//!
//! This module provides a ROS-facing parameters surface that aims for tool
//! parity with canonical ROS 2 parameter behavior while keeping the core rules
//! transport-agnostic in `rosrustext_core`.
//!
//! The main entry point is [`ParameterNode`], which owns a
//! `rosrustext_core::parameters::ParameterStore` and:
//! - exposes the standard ROS 2 parameter services (get/list/describe/set)
//! - publishes `rcl_interfaces/msg/ParameterEvent` on `<node>/parameter_events`
//!
//! For event-driven consumption without polling, use [`ParameterWatcher`].
//!
//! # Purpose (adapter-owned parameter surface)
//! - Provide a consistent ROS-facing parameter implementation independent of
//!   upstream client-library gaps.
//! - Keep parameter state in an explicit store that is easy to test and reason about.
//!
//! # Relationship to `rclrs` built-in parameter services
//! `rclrs` can start parameter services automatically. This adapter does **not**
//! use that path:
//! - [`ParameterNode::create`] explicitly disables `rclrs` parameter services via
//!   `.start_parameter_services(false)` and installs adapter-owned services.
//! - When wrapping an existing node via [`ParameterNode::try_new`] /
//!   [`ParameterNode::try_new_with_allow_undeclared`], ensure you created the
//!   node with parameter services disabled; otherwise service name collisions may occur.
//!
//! # Parameter store semantics
//! - The store is protected by a `Mutex` and shared via [`ParameterNode::store`].
//! - Service callbacks lock the store, apply updates, then unlock **before**
//!   publishing `parameter_events` (update-before-event).
//! - Unknown names are represented as `NOT_SET` / [`Value::NotSet`] in
//!   `get_parameters` and `get_parameter_types`.
//!
//! # ParameterWatcher semantics
//! - Subscribes to `<node>/parameter_events` and dispatches per-parameter handlers.
//! - Construction seeds an internal cache from the current store state.
//! - Not lifecycle-gated: if `parameter_events` are published, handlers run
//!   regardless of lifecycle state.
//!
//! # Known limitations
//! - Parameter deletion is not supported (setting `NOT_SET` is rejected).
//! - No user-defined set-time validation callback hook is currently exposed.
//! - No convenience typed getters are provided (consume [`Value`] directly).
//!
//! # Links
//! - [Parameters spec](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/parameters.md)
//! - [rosrs parameters parity](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/adapters/ros2rust/parameters/parity.md)

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
/// An `rclrs::Node` with adapter-owned parameter services enabled.
///
/// # Semantics
/// `ParameterNode` wires up the standard ROS 2 parameter services and owns an
/// in-memory [`rosrustext_core::parameters::ParameterStore`].
///
/// It is intentionally separate from any `rclrs` built-in parameter service
/// support; this crate disables `rclrs` parameter services by default when using
/// [`ParameterNode::create`].
///
/// # Threading
/// - Internal state is protected by a `Mutex`.
/// - Parameter service callbacks run on the executor thread and take the store
///   lock briefly to apply each request.
/// - `parameter_events` are published **after** the store is updated.
///
/// # Example
/// ```rust,no_run
/// use rclrs::{Context, CreateBasicExecutor, SpinOptions};
/// use rosrustext_rosrs::parameters::{Descriptor, ParameterNode, ParameterWatcher, Type, Value};
///
/// # fn main() -> rosrustext_rosrs::Result<()> {
/// let context = Context::default();
/// let mut executor = context.create_basic_executor();
///
/// let params = ParameterNode::create(&executor, "demo")?;
/// params.declare("threshold", Type::Double, Value::Double(0.5), Descriptor::default())?;
///
/// let watcher = ParameterWatcher::new(&params)?;
/// watcher.on_change("threshold", |change| {
///     // `change.new_value` contains the latest value.
///     let _ = &change.name;
/// });
///
/// executor.spin(SpinOptions::default());
/// # Ok(()) }
/// ```
///
/// # See also
/// - [`ParameterWatcher`]
/// - [Parameters spec](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/parameters.md)
pub struct ParameterNode {
    node: Arc<Node>,
    store: Arc<Mutex<ParameterStore>>,
    events: Arc<events::ParameterEventsPublisher>,
    internals: Arc<Mutex<Vec<Box<dyn std::any::Any + Send + Sync>>>>,
}

impl ParameterNode {
    /// Primary constructor: create a node on the executor and enable parameter services.
    ///
    /// # Semantics
    /// - Creates an `rclrs::Node` on the given executor.
    /// - Disables `rclrs` built-in parameter services (`start_parameter_services(false)`).
    /// - Enables adapter-owned parameter services and `parameter_events`.
    ///
    /// # Threading
    /// See [`ParameterNode`].
    ///
    /// # Example
    /// ```rust,no_run
    /// use rclrs::{Context, CreateBasicExecutor};
    /// use rosrustext_rosrs::parameters::ParameterNode;
    ///
    /// # fn main() -> rosrustext_rosrs::Result<()> {
    /// let context = Context::default();
    /// let executor = context.create_basic_executor();
    /// let _params = ParameterNode::create(&executor, "demo")?;
    /// # Ok(()) }
    /// ```
    ///
    /// # See also
    /// - [`ParameterNode::try_new`]
    pub fn create<'a>(executor: &'a Executor, options: impl IntoNodeOptions<'a>) -> Result<Self> {
        let options = options.into_node_options().start_parameter_services(false);
        let node = Arc::new(executor.create_node(options)?);
        Self::try_new(node)
    }

    /// Advanced constructor: wraps an existing node with parameter services.
    ///
    /// # Semantics
    /// Equivalent to [`ParameterNode::try_new_with_allow_undeclared`] with
    /// `allow_undeclared=false`.
    ///
    /// # Threading
    /// See [`ParameterNode`].
    ///
    /// # Example
    /// ```rust,no_run
    /// use std::sync::Arc;
    /// use rclrs::{Context, CreateBasicExecutor};
    /// use rosrustext_rosrs::parameters::ParameterNode;
    ///
    /// # fn main() -> rosrustext_rosrs::Result<()> {
    /// let context = Context::default();
    /// let executor = context.create_basic_executor();
    /// let node = Arc::new(executor.create_node("demo")?);
    /// let _params = ParameterNode::try_new(node)?;
    /// # Ok(()) }
    /// ```
    ///
    /// # See also
    /// - [`ParameterNode::try_new_with_allow_undeclared`]
    pub fn try_new(node: Arc<Node>) -> Result<Self> {
        Self::try_new_with_allow_undeclared(node, false)
    }

    /// Advanced constructor with explicit undeclared behavior.
    ///
    /// # Semantics
    /// - If `allow_undeclared=false`, setting an undeclared parameter via the ROS
    ///   services is rejected.
    /// - If `allow_undeclared=true`, setting an undeclared parameter creates it
    ///   (type inferred from the provided value), and it appears in lists/describes.
    ///
    /// # Threading
    /// See [`ParameterNode`].
    ///
    /// # Example
    /// ```rust,no_run
    /// use std::sync::Arc;
    /// use rclrs::{Context, CreateBasicExecutor};
    /// use rosrustext_rosrs::parameters::ParameterNode;
    ///
    /// # fn main() -> rosrustext_rosrs::Result<()> {
    /// let context = Context::default();
    /// let executor = context.create_basic_executor();
    /// let node = Arc::new(executor.create_node("demo")?);
    /// let _params = ParameterNode::try_new_with_allow_undeclared(node, true)?;
    /// # Ok(()) }
    /// ```
    ///
    /// # See also
    /// - [Parameters spec: declared vs undeclared](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/parameters.md#declared-vs-undeclared)
    pub fn try_new_with_allow_undeclared(node: Arc<Node>, allow_undeclared: bool) -> Result<Self> {
        let store = Arc::new(Mutex::new(ParameterStore::new(allow_undeclared)));
        let events = Arc::new(events::ParameterEventsPublisher::new(&node)?);
        let internals = Arc::new(Mutex::new(Vec::new()));
        let pn = Self { node, store, events, internals };
        pn.enable_defaults()?;
        Ok(pn)
    }

    /// Declare a parameter in the adapter-owned store.
    ///
    /// # Semantics
    /// - Declares `name` with the given `ty`, `default`, and `descriptor`.
    /// - Declared parameters become visible via ROS services (`list_parameters`,
    ///   `describe_parameters`, `get_parameters`, …).
    /// - This method does **not** publish a `parameter_events` message; events are
    ///   emitted for successful set operations.
    ///
    /// # Threading
    /// Takes the store mutex while updating the store. Keep the critical section
    /// small (do not hold the lock across long work).
    ///
    /// # Example
    /// ```rust,ignore
    /// use rosrustext_rosrs::parameters::{Descriptor, Type, Value};
    /// params.declare("threshold", Type::Double, Value::Double(0.5), Descriptor::default())?;
    /// ```
    ///
    /// # See also
    /// - [`Descriptor`]
    /// - [`ParameterNode::store`]
    pub fn declare(&self, name: &str, ty: Type, default: Value, descriptor: Descriptor) -> Result<()> {
        let mut store = self.store.lock().expect("parameter store mutex poisoned");
        store.declare(name, ty, default, descriptor)?;
        Ok(())
    }

    /// Borrow the shared parameter store.
    ///
    /// # Semantics
    /// Returns the adapter-owned store used by the ROS parameter services. This
    /// can be used for local reads (e.g. snapshots) without going through ROS
    /// services.
    ///
    /// # Threading
    /// The store is shared behind a `Mutex`. Avoid holding the lock across long
    /// work or within callbacks that may be called by the executor.
    ///
    /// # Example
    /// ```rust,ignore
    /// let store = params.store();
    /// let guard = store.lock().unwrap();
    /// let values = guard.get(&vec!["threshold".to_string()]);
    /// ```
    ///
    /// # See also
    /// - [`ParameterWatcher::snapshot`]
    pub fn store(&self) -> Arc<Mutex<ParameterStore>> {
        Arc::clone(&self.store)
    }

    /// Access the underlying `rclrs::Node`.
    ///
    /// # Semantics
    /// Returns the `Arc<Node>` used to host the parameter services.
    ///
    /// # Threading
    /// The node handle is shareable. Creating additional entities on the node is
    /// outside the scope of this module and is not parameter-managed.
    ///
    /// # Example
    /// ```rust,ignore
    /// let raw: &std::sync::Arc<rclrs::Node> = params.node();
    /// ```
    ///
    /// # See also
    /// - [`ParameterNode::create`]
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
        Type::NotSet => rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET,
        Type::Bool => rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
        Type::Integer => rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
        Type::Double => rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
        Type::String => rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
        Type::ByteArray => rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY,
        Type::BoolArray => rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY,
        Type::IntegerArray => rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY,
        Type::DoubleArray => rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY,
        Type::StringArray => rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
    }
}

fn to_ros_value(value: &Value) -> rclrs::vendor::rcl_interfaces::msg::ParameterValue {
    let mut msg = rclrs::vendor::rcl_interfaces::msg::ParameterValue::default();
    match value {
        Value::NotSet => {
            msg.type_ = rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
        }
        Value::Bool(v) => {
            msg.type_ = rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            msg.bool_value = *v;
        }
        Value::Integer(v) => {
            msg.type_ = rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            msg.integer_value = *v;
        }
        Value::Double(v) => {
            msg.type_ = rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            msg.double_value = *v;
        }
        Value::String(v) => {
            msg.type_ = rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            msg.string_value = v.clone();
        }
        Value::ByteArray(v) => {
            msg.type_ = rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY;
            msg.byte_array_value = v.clone();
        }
        Value::BoolArray(v) => {
            msg.type_ = rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
            msg.bool_array_value = v.clone();
        }
        Value::IntegerArray(v) => {
            msg.type_ = rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
            msg.integer_array_value = v.clone();
        }
        Value::DoubleArray(v) => {
            msg.type_ = rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
            msg.double_array_value = v.clone();
        }
        Value::StringArray(v) => {
            msg.type_ = rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            msg.string_array_value = v.clone();
        }
    }
    msg
}

pub(crate) fn from_ros_value(
    value: &rclrs::vendor::rcl_interfaces::msg::ParameterValue,
) -> std::result::Result<Value, String> {
    match value.type_ {
        t if t == rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET => Ok(Value::NotSet),
        t if t == rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_BOOL => {
            Ok(Value::Bool(value.bool_value))
        }
        t if t == rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER => {
            Ok(Value::Integer(value.integer_value))
        }
        t if t == rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE => {
            Ok(Value::Double(value.double_value))
        }
        t if t == rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_STRING => {
            Ok(Value::String(value.string_value.clone()))
        }
        t if t == rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY => {
            Ok(Value::ByteArray(value.byte_array_value.clone()))
        }
        t if t == rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY => {
            Ok(Value::BoolArray(value.bool_array_value.clone()))
        }
        t if t == rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY => {
            Ok(Value::IntegerArray(value.integer_array_value.clone()))
        }
        t if t == rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY => {
            Ok(Value::DoubleArray(value.double_array_value.clone()))
        }
        t if t == rclrs::vendor::rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY => {
            Ok(Value::StringArray(value.string_array_value.clone()))
        }
        _ => Err("unknown parameter type".to_string()),
    }
}

fn to_ros_descriptor(
    name: &str, ty: Type, descriptor: &Descriptor,
) -> rclrs::vendor::rcl_interfaces::msg::ParameterDescriptor {
    rclrs::vendor::rcl_interfaces::msg::ParameterDescriptor {
        name: name.to_string(),
        type_: to_ros_type(ty),
        description: descriptor.description.clone(),
        additional_constraints: descriptor.additional_constraints.clone(),
        read_only: descriptor.read_only,
        dynamic_typing: descriptor.dynamic_typing,
        ..Default::default()
    }
}

fn to_ros_parameter(param: rosrustext_core::parameters::Parameter) -> rclrs::vendor::rcl_interfaces::msg::Parameter {
    let rosrustext_core::parameters::Parameter { name, value } = param;
    rclrs::vendor::rcl_interfaces::msg::Parameter { name, value: to_ros_value(&value) }
}

fn to_ros_set_result(
    result: &rosrustext_core::parameters::SetResult,
) -> rclrs::vendor::rcl_interfaces::msg::SetParametersResult {
    rclrs::vendor::rcl_interfaces::msg::SetParametersResult {
        successful: result.success,
        reason: result.reason.clone().unwrap_or_default(),
    }
}
