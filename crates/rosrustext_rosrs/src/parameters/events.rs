use std::sync::Arc;

use rclrs::{IntoPrimitiveOptions, Node, QoSProfile};

use rosrustext_core::parameters::EventRecord;

use crate::error::Result;

use super::to_ros_parameter;

pub(crate) struct ParameterEventsPublisher {
    node_name: String,
    publisher: Arc<rclrs::Publisher<rclrs::vendor::rcl_interfaces::msg::ParameterEvent>>,
}

impl ParameterEventsPublisher {
    pub(crate) fn new(node: &Arc<Node>) -> Result<Self> {
        let fqn = node.fully_qualified_name();
        let topic = format!("{}/parameter_events", fqn);
        let publisher = Arc::new(node.create_publisher::<rclrs::vendor::rcl_interfaces::msg::ParameterEvent>(
            topic.qos(QoSProfile::parameter_events_default()),
        )?);
        Ok(Self { node_name: fqn, publisher })
    }

    pub(crate) fn publish(&self, record: EventRecord) -> Result<()> {
        if record.is_empty() {
            return Ok(());
        }
        let msg = rclrs::vendor::rcl_interfaces::msg::ParameterEvent {
            node: self.node_name.clone(),
            new_parameters: record.new_parameters.into_iter().map(to_ros_parameter).collect(),
            changed_parameters: record.changed_parameters.into_iter().map(to_ros_parameter).collect(),
            deleted_parameters: record.deleted_parameters.into_iter().map(to_ros_parameter).collect(),
            ..Default::default()
        };
        self.publisher.publish(msg)?;
        Ok(())
    }

    pub(crate) fn publisher(&self) -> Arc<rclrs::Publisher<rclrs::vendor::rcl_interfaces::msg::ParameterEvent>> {
        Arc::clone(&self.publisher)
    }
}
