pub mod rmw {
    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "bond__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__bond__msg__Constants() -> *const std::ffi::c_void;
    }

    #[link(name = "bond__rosidl_generator_c")]
    extern "C" {
        fn bond__msg__Constants__init(msg: *mut Constants) -> bool;
        fn bond__msg__Constants__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Constants>, size: usize) -> bool;
        fn bond__msg__Constants__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Constants>);
        fn bond__msg__Constants__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Constants>, out_seq: *mut rosidl_runtime_rs::Sequence<Constants>,
        ) -> bool;
    }

    // Corresponds to bond__msg__Constants
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Constants {
        pub structure_needs_at_least_one_member: u8,
    }

    impl Constants {
        pub const DEAD_PUBLISH_PERIOD: f32 = 0.05;
        pub const DEFAULT_CONNECT_TIMEOUT: f32 = 10.0;
        pub const DEFAULT_HEARTBEAT_TIMEOUT: f32 = 4.0;
        pub const DEFAULT_DISCONNECT_TIMEOUT: f32 = 2.0;
        pub const DEFAULT_HEARTBEAT_PERIOD: f32 = 1.0;
        pub const DISABLE_HEARTBEAT_TIMEOUT_PARAM: &'static str = "/bond_disable_heartbeat_timeout";
    }

    impl Default for Constants {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !bond__msg__Constants__init(&mut msg as *mut _) {
                    panic!("Call to bond__msg__Constants__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Constants {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { bond__msg__Constants__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { bond__msg__Constants__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { bond__msg__Constants__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Constants {
        type RmwMsg = Self;
        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for Constants
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "bond/msg/Constants";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe { rosidl_typesupport_c__get_message_type_support_handle__bond__msg__Constants() }
        }
    }

    #[link(name = "bond__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__bond__msg__Status() -> *const std::ffi::c_void;
    }

    #[link(name = "bond__rosidl_generator_c")]
    extern "C" {
        fn bond__msg__Status__init(msg: *mut Status) -> bool;
        fn bond__msg__Status__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Status>, size: usize) -> bool;
        fn bond__msg__Status__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Status>);
        fn bond__msg__Status__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Status>, out_seq: *mut rosidl_runtime_rs::Sequence<Status>,
        ) -> bool;
    }

    // Corresponds to bond__msg__Status
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Status {
        pub header: crate::std_msgs::msg::rmw::Header,
        pub id: rosidl_runtime_rs::String,
        pub instance_id: rosidl_runtime_rs::String,
        pub active: bool,
        pub heartbeat_timeout: f32,
        pub heartbeat_period: f32,
    }

    impl Default for Status {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !bond__msg__Status__init(&mut msg as *mut _) {
                    panic!("Call to bond__msg__Status__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Status {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { bond__msg__Status__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { bond__msg__Status__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { bond__msg__Status__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Status {
        type RmwMsg = Self;
        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for Status
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "bond/msg/Status";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe { rosidl_typesupport_c__get_message_type_support_handle__bond__msg__Status() }
        }
    }
} // mod rmw

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Constants {
    pub structure_needs_at_least_one_member: u8,
}

impl Constants {
    pub const DEAD_PUBLISH_PERIOD: f32 = 0.05;
    pub const DEFAULT_CONNECT_TIMEOUT: f32 = 10.0;
    pub const DEFAULT_HEARTBEAT_TIMEOUT: f32 = 4.0;
    pub const DEFAULT_DISCONNECT_TIMEOUT: f32 = 2.0;
    pub const DEFAULT_HEARTBEAT_PERIOD: f32 = 1.0;
    pub const DISABLE_HEARTBEAT_TIMEOUT_PARAM: &'static str = "/bond_disable_heartbeat_timeout";
}

impl Default for Constants {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::bond::msg::rmw::Constants::default())
    }
}

impl rosidl_runtime_rs::Message for Constants {
    type RmwMsg = crate::bond::msg::rmw::Constants;

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
pub struct Status {
    pub header: crate::std_msgs::msg::Header,
    pub id: std::string::String,
    pub instance_id: std::string::String,
    pub active: bool,
    pub heartbeat_timeout: f32,
    pub heartbeat_period: f32,
}

impl Default for Status {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::bond::msg::rmw::Status::default())
    }
}

impl rosidl_runtime_rs::Message for Status {
    type RmwMsg = crate::bond::msg::rmw::Status;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                header: crate::std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header))
                    .into_owned(),
                id: msg.id.as_str().into(),
                instance_id: msg.instance_id.as_str().into(),
                active: msg.active,
                heartbeat_timeout: msg.heartbeat_timeout,
                heartbeat_period: msg.heartbeat_period,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                header: crate::std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header))
                    .into_owned(),
                id: msg.id.as_str().into(),
                instance_id: msg.instance_id.as_str().into(),
                active: msg.active,
                heartbeat_timeout: msg.heartbeat_timeout,
                heartbeat_period: msg.heartbeat_period,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            header: crate::std_msgs::msg::Header::from_rmw_message(msg.header),
            id: msg.id.to_string(),
            instance_id: msg.instance_id.to_string(),
            active: msg.active,
            heartbeat_timeout: msg.heartbeat_timeout,
            heartbeat_period: msg.heartbeat_period,
        }
    }
}
