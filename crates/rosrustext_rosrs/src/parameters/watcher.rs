use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use rclrs::{IntoPrimitiveOptions, QoSProfile};

use rosrustext_core::parameters::{Type, Value};

use crate::error::Result;

use super::from_ros_value;

#[derive(Debug, Clone)]
/// A single observed parameter update.
///
/// # Semantics
/// - `name` is the fully qualified parameter name as reported in `parameter_events`.
/// - `old_value` is the previous value from the watcher's local cache (or
///   [`Value::NotSet`] if the parameter was not present in the cache).
/// - `new_value` is the value from the event message.
/// - `ty` is derived from `new_value.ty()` for convenience.
///
/// # Threading
/// This is a plain data record with no interior mutability. It is passed by
/// shared reference to change handlers.
///
/// # Example
/// ```rust,ignore
/// watcher.on_change("threshold", |change: &ParameterChange| {
///     assert_eq!(change.ty, change.new_value.ty());
/// });
/// ```
///
/// # See also
/// - [`ParameterWatcher::on_change`]
/// - [`Value`]
pub struct ParameterChange {
    pub name: String,
    pub old_value: Value,
    pub new_value: Value,
    pub ty: Type,
}

type Handler = Arc<dyn Fn(&ParameterChange) + Send + Sync + 'static>;

/// Watches `parameter_events` and dispatches per-parameter handlers.
///
/// # Semantics
/// - Subscribes to `<node>/parameter_events` using the default ROS 2 parameter
///   events QoS profile.
/// - Maintains a local cache of the latest value per parameter name.
/// - For each event, updates the cache and then invokes any handlers registered
///   for the affected parameter name.
///
/// Cache seeding:
/// - On construction, the cache is seeded from the current [`ParameterNode`]
///   store state (declared parameters and any existing values).
///
/// Note:
/// - Only `new_parameters` and `changed_parameters` are observed. Deletion is
///   not supported by this adapter and is not dispatched.
///
/// # Threading
/// - Handlers run in the `rclrs` subscription callback context (typically the
///   executor thread). Avoid blocking work in handlers.
/// - Internal state is protected by `Mutex`es (`handlers`, `cache`).
///
/// # Example
/// ```rust,no_run
/// use rosrustext_rosrs::parameters::{ParameterNode, ParameterWatcher};
///
/// # fn main() -> rosrustext_rosrs::Result<()> {
/// # use rclrs::{Context, CreateBasicExecutor};
/// let context = Context::default();
/// let executor = context.create_basic_executor();
/// let params = ParameterNode::create(&executor, "demo")?;
///
/// let watcher = ParameterWatcher::new(&params)?;
/// watcher.on_change("threshold", |change| {
///     let _ = &change.new_value;
/// });
/// # Ok(()) }
/// ```
///
/// # See also
/// - [`ParameterNode`]
/// - [Parameters spec](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/docs/spec/parameters.md)
/// - [rosrs parameters parity](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/docs/adapters/ros2rust/parameters/parity.md)
pub struct ParameterWatcher {
    _subscription: Arc<rclrs::Subscription<rclrs::vendor::rcl_interfaces::msg::ParameterEvent>>,
    handlers: Arc<Mutex<HashMap<String, Vec<Handler>>>>,
    cache: Arc<Mutex<HashMap<String, Value>>>,
}

impl ParameterWatcher {
    /// Construct a watcher for the given node.
    ///
    /// # Semantics
    /// - Seeds the internal cache from the node's current store state.
    /// - Subscribes to `<node>/parameter_events` and starts dispatching handlers.
    ///
    /// # Threading
    /// Service callbacks that mutate the store and the subscription callback that
    /// receives events may run on the executor thread. The watcher updates its
    /// cache under a `Mutex`.
    ///
    /// # Example
    /// ```rust,ignore
    /// let watcher = ParameterWatcher::new(&params)?;
    /// ```
    ///
    /// # See also
    /// - [`ParameterWatcher::on_change`]
    pub fn new(node: &super::ParameterNode) -> Result<Self> {
        let cache = Arc::new(Mutex::new(HashMap::new()));
        {
            let store = node.store();
            let guard = store.lock().expect("parameter store mutex poisoned");
            let list = guard.list(&[], 0);
            let values = guard.get(&list.names);
            let mut cache_guard = cache.lock().expect("parameter cache mutex poisoned");
            for (name, value) in list.names.into_iter().zip(values.into_iter()) {
                cache_guard.insert(name, value);
            }
        }

        let handlers = Arc::new(Mutex::new(HashMap::new()));
        let cache_for_sub = Arc::clone(&cache);
        let handlers_for_sub = Arc::clone(&handlers);

        let node_handle = node.node();
        let topic = format!("{}/parameter_events", node_handle.fully_qualified_name());
        let subscription = node_handle.create_subscription::<rclrs::vendor::rcl_interfaces::msg::ParameterEvent, _>(
            topic.qos(QoSProfile::parameter_events_default()),
            move |msg| {
                handle_event(msg, &cache_for_sub, &handlers_for_sub);
            },
        )?;

        Ok(Self { _subscription: Arc::new(subscription), handlers, cache })
    }

    /// Register a handler to run when a named parameter changes.
    ///
    /// # Semantics
    /// - `handler` is called for each `parameter_events` update that includes
    ///   `name` in `new_parameters` or `changed_parameters`.
    /// - Multiple handlers can be registered for the same name; all are called.
    ///
    /// # Threading
    /// - Registration takes a lock on the internal handler table.
    /// - Handlers are called from the subscription callback context; avoid
    ///   blocking operations.
    ///
    /// # Example
    /// ```rust,ignore
    /// watcher.on_change("threshold", |change| {
    ///     let _ = &change.old_value;
    /// });
    /// ```
    ///
    /// # See also
    /// - [`ParameterWatcher::snapshot`]
    pub fn on_change<F>(&self, name: impl Into<String>, handler: F)
    where
        F: Fn(&ParameterChange) + Send + Sync + 'static,
    {
        let mut guard = self.handlers.lock().expect("parameter handler mutex poisoned");
        guard.entry(name.into()).or_default().push(Arc::new(handler));
    }

    /// Return a snapshot of the watcher's current cached state.
    ///
    /// # Semantics
    /// Clones the internal cache (`HashMap<String, Value>`). The snapshot is
    /// eventually consistent with `parameter_events`.
    ///
    /// # Threading
    /// Takes the cache lock briefly to clone.
    ///
    /// # Example
    /// ```rust,ignore
    /// let current = watcher.snapshot();
    /// ```
    ///
    /// # See also
    /// - [`ParameterNode::store`]
    pub fn snapshot(&self) -> HashMap<String, Value> {
        self.cache.lock().expect("parameter cache mutex poisoned").clone()
    }
}

fn handle_event(
    msg: rclrs::vendor::rcl_interfaces::msg::ParameterEvent, cache: &Arc<Mutex<HashMap<String, Value>>>,
    handlers: &Arc<Mutex<HashMap<String, Vec<Handler>>>>,
) {
    let mut params = Vec::new();
    params.extend(msg.new_parameters);
    params.extend(msg.changed_parameters);

    for param in params {
        let value = match from_ros_value(&param.value) {
            Ok(value) => value,
            Err(_) => continue,
        };

        let old_value = {
            let mut cache_guard = cache.lock().expect("parameter cache mutex poisoned");
            let old_value = cache_guard.get(&param.name).cloned().unwrap_or(Value::NotSet);
            cache_guard.insert(param.name.clone(), value.clone());
            old_value
        };

        let change = ParameterChange { name: param.name, old_value, new_value: value.clone(), ty: value.ty() };

        let handlers_to_call = {
            let guard = handlers.lock().expect("parameter handler mutex poisoned");
            guard.get(&change.name).cloned()
        };

        if let Some(list) = handlers_to_call {
            for handler in list {
                handler(&change);
            }
        }
    }
}
