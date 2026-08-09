#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to turtlebot3_pick_place__srv__GetTargetPose_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTargetPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target_class: std::string::String,

}



impl Default for GetTargetPose_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetTargetPose_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetTargetPose_Request {
  type RmwMsg = super::srv::rmw::GetTargetPose_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target_class: msg.target_class.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target_class: msg.target_class.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      target_class: msg.target_class.to_string(),
    }
  }
}


// Corresponds to turtlebot3_pick_place__srv__GetTargetPose_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTargetPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_pose: geometry_msgs::msg::PoseStamped,

}



impl Default for GetTargetPose_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetTargetPose_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetTargetPose_Response {
  type RmwMsg = super::srv::rmw::GetTargetPose_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
        target_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.target_pose)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
        target_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.target_pose)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
      target_pose: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.target_pose),
    }
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


