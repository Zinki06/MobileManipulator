// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_pick_place:srv/GetTargetPose.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__BUILDER_HPP_
#define TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_pick_place/srv/detail/get_target_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_pick_place
{

namespace srv
{

namespace builder
{

class Init_GetTargetPose_Request_target_class
{
public:
  Init_GetTargetPose_Request_target_class()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_pick_place::srv::GetTargetPose_Request target_class(::turtlebot3_pick_place::srv::GetTargetPose_Request::_target_class_type arg)
  {
    msg_.target_class = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_pick_place::srv::GetTargetPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_pick_place::srv::GetTargetPose_Request>()
{
  return turtlebot3_pick_place::srv::builder::Init_GetTargetPose_Request_target_class();
}

}  // namespace turtlebot3_pick_place


namespace turtlebot3_pick_place
{

namespace srv
{

namespace builder
{

class Init_GetTargetPose_Response_target_pose
{
public:
  explicit Init_GetTargetPose_Response_target_pose(::turtlebot3_pick_place::srv::GetTargetPose_Response & msg)
  : msg_(msg)
  {}
  ::turtlebot3_pick_place::srv::GetTargetPose_Response target_pose(::turtlebot3_pick_place::srv::GetTargetPose_Response::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_pick_place::srv::GetTargetPose_Response msg_;
};

class Init_GetTargetPose_Response_message
{
public:
  explicit Init_GetTargetPose_Response_message(::turtlebot3_pick_place::srv::GetTargetPose_Response & msg)
  : msg_(msg)
  {}
  Init_GetTargetPose_Response_target_pose message(::turtlebot3_pick_place::srv::GetTargetPose_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GetTargetPose_Response_target_pose(msg_);
  }

private:
  ::turtlebot3_pick_place::srv::GetTargetPose_Response msg_;
};

class Init_GetTargetPose_Response_success
{
public:
  Init_GetTargetPose_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTargetPose_Response_message success(::turtlebot3_pick_place::srv::GetTargetPose_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetTargetPose_Response_message(msg_);
  }

private:
  ::turtlebot3_pick_place::srv::GetTargetPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_pick_place::srv::GetTargetPose_Response>()
{
  return turtlebot3_pick_place::srv::builder::Init_GetTargetPose_Response_success();
}

}  // namespace turtlebot3_pick_place

#endif  // TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__BUILDER_HPP_
