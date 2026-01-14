use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use rclrs::{IntoPrimitiveOptions, QoSProfile};

use rosrustext_core::parameters::{Type, Value};

use crate::error::Result;

use super::from_ros_value;

#[derive(Debug, Clone)]
pub struct ParameterChange {
    pub name: String,
    pub old_value: Value,
    pub new_value: Value,
    pub ty: Type,
}

type Handler = Arc<dyn Fn(&ParameterChange) + Send + Sync + 'static>;

/// Watches parameter_events and dispatches per-parameter handlers.
///
/// Keep this handle alive to keep the subscription active.
pub struct ParameterWatcher {
    _subscription: Arc<rclrs::Subscription<rcl_interfaces::msg::ParameterEvent>>,
    handlers: Arc<Mutex<HashMap<String, Vec<Handler>>>>,
    cache: Arc<Mutex<HashMap<String, Value>>>,
}

impl ParameterWatcher {
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
        let subscription = node_handle.create_subscription::<rcl_interfaces::msg::ParameterEvent, _>(
            topic.qos(QoSProfile::parameter_events_default()),
            move |msg| {
                handle_event(msg, &cache_for_sub, &handlers_for_sub);
            },
        )?;

        Ok(Self {
            _subscription: Arc::new(subscription),
            handlers,
            cache,
        })
    }

    pub fn on_change<F>(&self, name: impl Into<String>, handler: F)
    where
        F: Fn(&ParameterChange) + Send + Sync + 'static,
    {
        let mut guard = self.handlers.lock().expect("parameter handler mutex poisoned");
        guard.entry(name.into()).or_default().push(Arc::new(handler));
    }

    pub fn snapshot(&self) -> HashMap<String, Value> {
        self.cache.lock().expect("parameter cache mutex poisoned").clone()
    }
}

fn handle_event(
    msg: rcl_interfaces::msg::ParameterEvent,
    cache: &Arc<Mutex<HashMap<String, Value>>>,
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

        let change = ParameterChange {
            name: param.name,
            old_value,
            new_value: value.clone(),
            ty: value.ty(),
        };

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
