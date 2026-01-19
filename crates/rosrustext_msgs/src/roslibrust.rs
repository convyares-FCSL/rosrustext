use roslibrust::{RosMessageType, RosServiceType};

use crate::lifecycle_msgs::{msg, srv};

impl RosMessageType for msg::State {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/State";
}

impl RosMessageType for msg::Transition {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/Transition";
}

impl RosMessageType for msg::TransitionDescription {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/TransitionDescription";
}

impl RosMessageType for msg::TransitionEvent {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/TransitionEvent";
}

impl RosMessageType for srv::ChangeState_Request {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/ChangeStateRequest";
}

impl RosMessageType for srv::ChangeState_Response {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/ChangeStateResponse";
}

impl RosMessageType for srv::GetState_Request {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/GetStateRequest";
}

impl RosMessageType for srv::GetState_Response {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/GetStateResponse";
}

impl RosMessageType for srv::GetAvailableStates_Request {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/GetAvailableStatesRequest";
}

impl RosMessageType for srv::GetAvailableStates_Response {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/GetAvailableStatesResponse";
}

impl RosMessageType for srv::GetAvailableTransitions_Request {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/GetAvailableTransitionsRequest";
}

impl RosMessageType for srv::GetAvailableTransitions_Response {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/GetAvailableTransitionsResponse";
}

impl RosServiceType for srv::ChangeState {
    const ROS_SERVICE_NAME: &'static str = "lifecycle_msgs/ChangeState";
    type Request = srv::ChangeState_Request;
    type Response = srv::ChangeState_Response;
}

impl RosServiceType for srv::GetState {
    const ROS_SERVICE_NAME: &'static str = "lifecycle_msgs/GetState";
    type Request = srv::GetState_Request;
    type Response = srv::GetState_Response;
}

impl RosServiceType for srv::GetAvailableStates {
    const ROS_SERVICE_NAME: &'static str = "lifecycle_msgs/GetAvailableStates";
    type Request = srv::GetAvailableStates_Request;
    type Response = srv::GetAvailableStates_Response;
}

impl RosServiceType for srv::GetAvailableTransitions {
    const ROS_SERVICE_NAME: &'static str = "lifecycle_msgs/GetAvailableTransitions";
    type Request = srv::GetAvailableTransitions_Request;
    type Response = srv::GetAvailableTransitions_Response;
}
