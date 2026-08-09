// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_pick_place:srv/GetTargetPose.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__TRAITS_HPP_
#define TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_pick_place/srv/detail/get_target_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot3_pick_place
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTargetPose_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_class
  {
    out << "target_class: ";
    rosidl_generator_traits::value_to_yaml(msg.target_class, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTargetPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_class
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_class: ";
    rosidl_generator_traits::value_to_yaml(msg.target_class, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTargetPose_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlebot3_pick_place

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_pick_place::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_pick_place::srv::GetTargetPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_pick_place::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_pick_place::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_pick_place::srv::GetTargetPose_Request & msg)
{
  return turtlebot3_pick_place::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_pick_place::srv::GetTargetPose_Request>()
{
  return "turtlebot3_pick_place::srv::GetTargetPose_Request";
}

template<>
inline const char * name<turtlebot3_pick_place::srv::GetTargetPose_Request>()
{
  return "turtlebot3_pick_place/srv/GetTargetPose_Request";
}

template<>
struct has_fixed_size<turtlebot3_pick_place::srv::GetTargetPose_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtlebot3_pick_place::srv::GetTargetPose_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtlebot3_pick_place::srv::GetTargetPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace turtlebot3_pick_place
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTargetPose_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: target_pose
  {
    out << "target_pose: ";
    to_flow_style_yaml(msg.target_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTargetPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: target_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_pose:\n";
    to_block_style_yaml(msg.target_pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTargetPose_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlebot3_pick_place

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_pick_place::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_pick_place::srv::GetTargetPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_pick_place::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_pick_place::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_pick_place::srv::GetTargetPose_Response & msg)
{
  return turtlebot3_pick_place::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_pick_place::srv::GetTargetPose_Response>()
{
  return "turtlebot3_pick_place::srv::GetTargetPose_Response";
}

template<>
inline const char * name<turtlebot3_pick_place::srv::GetTargetPose_Response>()
{
  return "turtlebot3_pick_place/srv/GetTargetPose_Response";
}

template<>
struct has_fixed_size<turtlebot3_pick_place::srv::GetTargetPose_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtlebot3_pick_place::srv::GetTargetPose_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtlebot3_pick_place::srv::GetTargetPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_pick_place::srv::GetTargetPose>()
{
  return "turtlebot3_pick_place::srv::GetTargetPose";
}

template<>
inline const char * name<turtlebot3_pick_place::srv::GetTargetPose>()
{
  return "turtlebot3_pick_place/srv/GetTargetPose";
}

template<>
struct has_fixed_size<turtlebot3_pick_place::srv::GetTargetPose>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_pick_place::srv::GetTargetPose_Request>::value &&
    has_fixed_size<turtlebot3_pick_place::srv::GetTargetPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_pick_place::srv::GetTargetPose>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_pick_place::srv::GetTargetPose_Request>::value &&
    has_bounded_size<turtlebot3_pick_place::srv::GetTargetPose_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_pick_place::srv::GetTargetPose>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_pick_place::srv::GetTargetPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_pick_place::srv::GetTargetPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__TRAITS_HPP_
