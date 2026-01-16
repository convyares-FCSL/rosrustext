use std::sync::{Arc, Mutex};

use rclrs::{IntoPrimitiveOptions, Node, QoSProfile};

use rosrustext_core::parameters::{EventRecord, ParameterStore, SetResult};

use crate::error::Result;

use super::events::ParameterEventsPublisher;
use super::{from_ros_value, to_ros_descriptor, to_ros_set_result, to_ros_type, to_ros_value};

pub(crate) fn enable_services(
    node: &Arc<Node>, store: Arc<Mutex<ParameterStore>>, events: Arc<ParameterEventsPublisher>,
    internals: Arc<Mutex<Vec<Box<dyn std::any::Any + Send + Sync>>>>,
) -> Result<()> {
    let fqn = node.fully_qualified_name();

    let store_for_get = Arc::clone(&store);
    let svc = node.create_service::<rclrs::vendor::rcl_interfaces::srv::GetParameters, _>(
        (fqn.clone() + "/get_parameters").qos(QoSProfile::parameter_services_default()),
        move |req: rclrs::vendor::rcl_interfaces::srv::GetParameters_Request| {
            let values = {
                let guard = store_for_get.lock().expect("parameter store mutex poisoned");
                guard.get(&req.names)
            };
            let values = values.iter().map(to_ros_value).collect();
            rclrs::vendor::rcl_interfaces::srv::GetParameters_Response { values }
        },
    )?;
    keep_internal(&internals, svc);

    let store_for_types = Arc::clone(&store);
    let svc = node.create_service::<rclrs::vendor::rcl_interfaces::srv::GetParameterTypes, _>(
        (fqn.clone() + "/get_parameter_types").qos(QoSProfile::parameter_services_default()),
        move |req: rclrs::vendor::rcl_interfaces::srv::GetParameterTypes_Request| {
            let types = {
                let guard = store_for_types.lock().expect("parameter store mutex poisoned");
                guard.get_types(&req.names)
            };
            let types = types.into_iter().map(to_ros_type).collect();
            rclrs::vendor::rcl_interfaces::srv::GetParameterTypes_Response { types }
        },
    )?;
    keep_internal(&internals, svc);

    let store_for_list = Arc::clone(&store);
    let svc = node.create_service::<rclrs::vendor::rcl_interfaces::srv::ListParameters, _>(
        (fqn.clone() + "/list_parameters").qos(QoSProfile::parameter_services_default()),
        move |req: rclrs::vendor::rcl_interfaces::srv::ListParameters_Request| {
            let list = {
                let guard = store_for_list.lock().expect("parameter store mutex poisoned");
                guard.list(&req.prefixes, req.depth)
            };
            let result =
                rclrs::vendor::rcl_interfaces::msg::ListParametersResult { names: list.names, prefixes: list.prefixes };
            rclrs::vendor::rcl_interfaces::srv::ListParameters_Response { result }
        },
    )?;
    keep_internal(&internals, svc);

    let store_for_describe = Arc::clone(&store);
    let svc = node.create_service::<rclrs::vendor::rcl_interfaces::srv::DescribeParameters, _>(
        (fqn.clone() + "/describe_parameters").qos(QoSProfile::parameter_services_default()),
        move |req: rclrs::vendor::rcl_interfaces::srv::DescribeParameters_Request| {
            let described = {
                let guard = store_for_describe.lock().expect("parameter store mutex poisoned");
                guard.describe(&req.names)
            };
            let descriptors =
                described.iter().map(|entry| to_ros_descriptor(&entry.name, entry.ty, &entry.descriptor)).collect();
            rclrs::vendor::rcl_interfaces::srv::DescribeParameters_Response { descriptors }
        },
    )?;
    keep_internal(&internals, svc);

    let store_for_set = Arc::clone(&store);
    let events_for_set = Arc::clone(&events);
    let svc = node.create_service::<rclrs::vendor::rcl_interfaces::srv::SetParameters, _>(
        (fqn.clone() + "/set_parameters").qos(QoSProfile::parameter_services_default()),
        move |req: rclrs::vendor::rcl_interfaces::srv::SetParameters_Request| {
            let mut results = Vec::with_capacity(req.parameters.len());
            let mut record = EventRecord::default();
            let mut guard = store_for_set.lock().expect("parameter store mutex poisoned");
            for param in req.parameters {
                let name = param.name.clone();
                match from_ros_value(&param.value) {
                    Ok(value) => {
                        let (result, event) = guard.set_parameter(name, value);
                        results.push(result);
                        record.merge(event);
                    }
                    Err(reason) => {
                        results.push(SetResult::err(reason));
                    }
                }
            }
            drop(guard);
            let _ = events_for_set.publish(record);
            let results = results.iter().map(to_ros_set_result).collect();
            rclrs::vendor::rcl_interfaces::srv::SetParameters_Response { results }
        },
    )?;
    keep_internal(&internals, svc);

    let store_for_atomic = Arc::clone(&store);
    let events_for_atomic = Arc::clone(&events);
    let svc = node.create_service::<rclrs::vendor::rcl_interfaces::srv::SetParametersAtomically, _>(
        (fqn + "/set_parameters_atomically").qos(QoSProfile::parameter_services_default()),
        move |req: rclrs::vendor::rcl_interfaces::srv::SetParametersAtomically_Request| {
            let mut converted = Vec::with_capacity(req.parameters.len());
            for param in req.parameters {
                match from_ros_value(&param.value) {
                    Ok(value) => converted.push((param.name.clone(), value)),
                    Err(reason) => {
                        return rclrs::vendor::rcl_interfaces::srv::SetParametersAtomically_Response {
                            result: to_ros_set_result(&SetResult::err(reason)),
                        };
                    }
                }
            }
            let (result, record) = {
                let mut guard = store_for_atomic.lock().expect("parameter store mutex poisoned");
                guard.set_parameters_atomically(converted)
            };
            if result.success {
                let _ = events_for_atomic.publish(record);
            }
            rclrs::vendor::rcl_interfaces::srv::SetParametersAtomically_Response { result: to_ros_set_result(&result) }
        },
    )?;
    keep_internal(&internals, svc);

    Ok(())
}

fn keep_internal<T>(internals: &Arc<Mutex<Vec<Box<dyn std::any::Any + Send + Sync>>>>, handle: T)
where
    T: Send + Sync + 'static,
{
    internals.lock().expect("ParameterNode internals poisoned").push(Box::new(handle));
}
