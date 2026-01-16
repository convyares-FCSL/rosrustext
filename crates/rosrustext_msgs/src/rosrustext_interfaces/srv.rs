#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTransitionGraph_Request {
    pub structure_needs_at_least_one_member: u8,
}

impl Default for GetTransitionGraph_Request {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::rosrustext_interfaces::srv::rmw::GetTransitionGraph_Request::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for GetTransitionGraph_Request {
    type RmwMsg = crate::rosrustext_interfaces::srv::rmw::GetTransitionGraph_Request;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self { structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTransitionGraph_Response {
    pub states: Vec<crate::lifecycle_msgs::msg::State>,
    pub transitions: Vec<crate::lifecycle_msgs::msg::TransitionDescription>,
}

impl Default for GetTransitionGraph_Response {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::rosrustext_interfaces::srv::rmw::GetTransitionGraph_Response::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for GetTransitionGraph_Response {
    type RmwMsg = crate::rosrustext_interfaces::srv::rmw::GetTransitionGraph_Response;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                states: msg
                    .states
                    .into_iter()
                    .map(|elem| {
                        crate::lifecycle_msgs::msg::State::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned()
                    })
                    .collect(),
                transitions: msg
                    .transitions
                    .into_iter()
                    .map(|elem| {
                        crate::lifecycle_msgs::msg::TransitionDescription::into_rmw_message(std::borrow::Cow::Owned(
                            elem,
                        ))
                        .into_owned()
                    })
                    .collect(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                states: msg
                    .states
                    .iter()
                    .map(|elem| {
                        crate::lifecycle_msgs::msg::State::into_rmw_message(std::borrow::Cow::Borrowed(elem))
                            .into_owned()
                    })
                    .collect(),
                transitions: msg
                    .transitions
                    .iter()
                    .map(|elem| {
                        crate::lifecycle_msgs::msg::TransitionDescription::into_rmw_message(std::borrow::Cow::Borrowed(
                            elem,
                        ))
                        .into_owned()
                    })
                    .collect(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            states: msg.states.into_iter().map(crate::lifecycle_msgs::msg::State::from_rmw_message).collect(),
            transitions: msg
                .transitions
                .into_iter()
                .map(crate::lifecycle_msgs::msg::TransitionDescription::from_rmw_message)
                .collect(),
        }
    }
}

#[link(name = "rosrustext_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__rosrustext_interfaces__srv__GetTransitionGraph(
    ) -> *const std::ffi::c_void;
}

// Corresponds to rosrustext_interfaces__srv__GetTransitionGraph
pub struct GetTransitionGraph;

impl rosidl_runtime_rs::Service for GetTransitionGraph {
    type Request = crate::rosrustext_interfaces::srv::GetTransitionGraph_Request;
    type Response = crate::rosrustext_interfaces::srv::GetTransitionGraph_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__rosrustext_interfaces__srv__GetTransitionGraph()
        }
    }
}

pub mod rmw {
    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "rosrustext_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rosrustext_interfaces__srv__GetTransitionGraph_Request(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rosrustext_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rosrustext_interfaces__srv__GetTransitionGraph_Request__init(msg: *mut GetTransitionGraph_Request) -> bool;
        fn rosrustext_interfaces__srv__GetTransitionGraph_Request__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<GetTransitionGraph_Request>, size: usize,
        ) -> bool;
        fn rosrustext_interfaces__srv__GetTransitionGraph_Request__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<GetTransitionGraph_Request>,
        );
        fn rosrustext_interfaces__srv__GetTransitionGraph_Request__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<GetTransitionGraph_Request>,
            out_seq: *mut rosidl_runtime_rs::Sequence<GetTransitionGraph_Request>,
        ) -> bool;
    }

    // Corresponds to rosrustext_interfaces__srv__GetTransitionGraph_Request
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct GetTransitionGraph_Request {
        pub structure_needs_at_least_one_member: u8,
    }

    impl Default for GetTransitionGraph_Request {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rosrustext_interfaces__srv__GetTransitionGraph_Request__init(&mut msg as *mut _) {
                    panic!("Call to rosrustext_interfaces__srv__GetTransitionGraph_Request__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for GetTransitionGraph_Request {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rosrustext_interfaces__srv__GetTransitionGraph_Request__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rosrustext_interfaces__srv__GetTransitionGraph_Request__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rosrustext_interfaces__srv__GetTransitionGraph_Request__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for GetTransitionGraph_Request {
        type RmwMsg = Self;
        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for GetTransitionGraph_Request
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rosrustext_interfaces/srv/GetTransitionGraph_Request";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rosrustext_interfaces__srv__GetTransitionGraph_Request()
            }
        }
    }

    #[link(name = "rosrustext_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rosrustext_interfaces__srv__GetTransitionGraph_Response(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rosrustext_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rosrustext_interfaces__srv__GetTransitionGraph_Response__init(msg: *mut GetTransitionGraph_Response)
            -> bool;
        fn rosrustext_interfaces__srv__GetTransitionGraph_Response__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<GetTransitionGraph_Response>, size: usize,
        ) -> bool;
        fn rosrustext_interfaces__srv__GetTransitionGraph_Response__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<GetTransitionGraph_Response>,
        );
        fn rosrustext_interfaces__srv__GetTransitionGraph_Response__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<GetTransitionGraph_Response>,
            out_seq: *mut rosidl_runtime_rs::Sequence<GetTransitionGraph_Response>,
        ) -> bool;
    }

    // Corresponds to rosrustext_interfaces__srv__GetTransitionGraph_Response
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct GetTransitionGraph_Response {
        pub states: rosidl_runtime_rs::Sequence<crate::lifecycle_msgs::msg::rmw::State>,
        pub transitions: rosidl_runtime_rs::Sequence<crate::lifecycle_msgs::msg::rmw::TransitionDescription>,
    }

    impl Default for GetTransitionGraph_Response {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rosrustext_interfaces__srv__GetTransitionGraph_Response__init(&mut msg as *mut _) {
                    panic!("Call to rosrustext_interfaces__srv__GetTransitionGraph_Response__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for GetTransitionGraph_Response {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rosrustext_interfaces__srv__GetTransitionGraph_Response__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rosrustext_interfaces__srv__GetTransitionGraph_Response__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                rosrustext_interfaces__srv__GetTransitionGraph_Response__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for GetTransitionGraph_Response {
        type RmwMsg = Self;
        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for GetTransitionGraph_Response
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rosrustext_interfaces/srv/GetTransitionGraph_Response";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rosrustext_interfaces__srv__GetTransitionGraph_Response()
            }
        }
    }

    #[link(name = "rosrustext_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__rosrustext_interfaces__srv__GetTransitionGraph(
        ) -> *const std::ffi::c_void;
    }

    // Corresponds to rosrustext_interfaces__srv__GetTransitionGraph
    pub struct GetTransitionGraph;

    impl rosidl_runtime_rs::Service for GetTransitionGraph {
        type Request = crate::rosrustext_interfaces::srv::rmw::GetTransitionGraph_Request;
        type Response = crate::rosrustext_interfaces::srv::rmw::GetTransitionGraph_Response;

        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_service_type_support_handle__rosrustext_interfaces__srv__GetTransitionGraph()
            }
        }
    }
} // mod rmw
