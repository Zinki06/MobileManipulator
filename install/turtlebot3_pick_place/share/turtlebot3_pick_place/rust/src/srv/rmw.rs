#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "turtlebot3_pick_place__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__turtlebot3_pick_place__srv__GetTargetPose_Request() -> *const std::ffi::c_void;
}

#[link(name = "turtlebot3_pick_place__rosidl_generator_c")]
extern "C" {
    fn turtlebot3_pick_place__srv__GetTargetPose_Request__init(msg: *mut GetTargetPose_Request) -> bool;
    fn turtlebot3_pick_place__srv__GetTargetPose_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetTargetPose_Request>, size: usize) -> bool;
    fn turtlebot3_pick_place__srv__GetTargetPose_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetTargetPose_Request>);
    fn turtlebot3_pick_place__srv__GetTargetPose_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetTargetPose_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetTargetPose_Request>) -> bool;
}

// Corresponds to turtlebot3_pick_place__srv__GetTargetPose_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTargetPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target_class: rosidl_runtime_rs::String,

}



impl Default for GetTargetPose_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !turtlebot3_pick_place__srv__GetTargetPose_Request__init(&mut msg as *mut _) {
        panic!("Call to turtlebot3_pick_place__srv__GetTargetPose_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetTargetPose_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { turtlebot3_pick_place__srv__GetTargetPose_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { turtlebot3_pick_place__srv__GetTargetPose_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { turtlebot3_pick_place__srv__GetTargetPose_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetTargetPose_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetTargetPose_Request where Self: Sized {
  const TYPE_NAME: &'static str = "turtlebot3_pick_place/srv/GetTargetPose_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__turtlebot3_pick_place__srv__GetTargetPose_Request() }
  }
}


#[link(name = "turtlebot3_pick_place__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__turtlebot3_pick_place__srv__GetTargetPose_Response() -> *const std::ffi::c_void;
}

#[link(name = "turtlebot3_pick_place__rosidl_generator_c")]
extern "C" {
    fn turtlebot3_pick_place__srv__GetTargetPose_Response__init(msg: *mut GetTargetPose_Response) -> bool;
    fn turtlebot3_pick_place__srv__GetTargetPose_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetTargetPose_Response>, size: usize) -> bool;
    fn turtlebot3_pick_place__srv__GetTargetPose_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetTargetPose_Response>);
    fn turtlebot3_pick_place__srv__GetTargetPose_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetTargetPose_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetTargetPose_Response>) -> bool;
}

// Corresponds to turtlebot3_pick_place__srv__GetTargetPose_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTargetPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_pose: geometry_msgs::msg::rmw::PoseStamped,

}



impl Default for GetTargetPose_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !turtlebot3_pick_place__srv__GetTargetPose_Response__init(&mut msg as *mut _) {
        panic!("Call to turtlebot3_pick_place__srv__GetTargetPose_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetTargetPose_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { turtlebot3_pick_place__srv__GetTargetPose_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { turtlebot3_pick_place__srv__GetTargetPose_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { turtlebot3_pick_place__srv__GetTargetPose_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetTargetPose_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetTargetPose_Response where Self: Sized {
  const TYPE_NAME: &'static str = "turtlebot3_pick_place/srv/GetTargetPose_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__turtlebot3_pick_place__srv__GetTargetPose_Response() }
  }
}






#[link(name = "turtlebot3_pick_place__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__turtlebot3_pick_place__srv__GetTargetPose() -> *const std::ffi::c_void;
}

// Corresponds to turtlebot3_pick_place__srv__GetTargetPose
#[allow(missing_docs, non_camel_case_types)]
pub struct GetTargetPose;

impl rosidl_runtime_rs::Service for GetTargetPose {
    type Request = GetTargetPose_Request;
    type Response = GetTargetPose_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__turtlebot3_pick_place__srv__GetTargetPose() }
    }
}


