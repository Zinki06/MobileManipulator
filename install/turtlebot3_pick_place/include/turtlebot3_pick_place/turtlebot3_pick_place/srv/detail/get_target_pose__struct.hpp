// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_pick_place:srv/GetTargetPose.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__STRUCT_HPP_
#define TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_pick_place__srv__GetTargetPose_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_pick_place__srv__GetTargetPose_Request __declspec(deprecated)
#endif

namespace turtlebot3_pick_place
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTargetPose_Request_
{
  using Type = GetTargetPose_Request_<ContainerAllocator>;

  explicit GetTargetPose_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_class = "";
    }
  }

  explicit GetTargetPose_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_class(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_class = "";
    }
  }

  // field types and members
  using _target_class_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_class_type target_class;

  // setters for named parameter idiom
  Type & set__target_class(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target_class = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_pick_place__srv__GetTargetPose_Request
    std::shared_ptr<turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_pick_place__srv__GetTargetPose_Request
    std::shared_ptr<turtlebot3_pick_place::srv::GetTargetPose_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTargetPose_Request_ & other) const
  {
    if (this->target_class != other.target_class) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTargetPose_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTargetPose_Request_

// alias to use template instance with default allocator
using GetTargetPose_Request =
  turtlebot3_pick_place::srv::GetTargetPose_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_pick_place


// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__turtlebot3_pick_place__srv__GetTargetPose_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_pick_place__srv__GetTargetPose_Response __declspec(deprecated)
#endif

namespace turtlebot3_pick_place
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTargetPose_Response_
{
  using Type = GetTargetPose_Response_<ContainerAllocator>;

  explicit GetTargetPose_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit GetTargetPose_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    target_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _target_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _target_pose_type target_pose;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__target_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->target_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_pick_place__srv__GetTargetPose_Response
    std::shared_ptr<turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_pick_place__srv__GetTargetPose_Response
    std::shared_ptr<turtlebot3_pick_place::srv::GetTargetPose_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTargetPose_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->target_pose != other.target_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTargetPose_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTargetPose_Response_

// alias to use template instance with default allocator
using GetTargetPose_Response =
  turtlebot3_pick_place::srv::GetTargetPose_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_pick_place

namespace turtlebot3_pick_place
{

namespace srv
{

struct GetTargetPose
{
  using Request = turtlebot3_pick_place::srv::GetTargetPose_Request;
  using Response = turtlebot3_pick_place::srv::GetTargetPose_Response;
};

}  // namespace srv

}  // namespace turtlebot3_pick_place

#endif  // TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__STRUCT_HPP_
