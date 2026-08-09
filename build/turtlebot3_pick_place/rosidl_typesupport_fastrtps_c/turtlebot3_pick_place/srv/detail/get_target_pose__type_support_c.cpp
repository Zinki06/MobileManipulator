// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from turtlebot3_pick_place:srv/GetTargetPose.idl
// generated code does not contain a copyright notice
#include "turtlebot3_pick_place/srv/detail/get_target_pose__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "turtlebot3_pick_place/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "turtlebot3_pick_place/srv/detail/get_target_pose__struct.h"
#include "turtlebot3_pick_place/srv/detail/get_target_pose__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // target_class
#include "rosidl_runtime_c/string_functions.h"  // target_class

// forward declare type support functions


using _GetTargetPose_Request__ros_msg_type = turtlebot3_pick_place__srv__GetTargetPose_Request;

static bool _GetTargetPose_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetTargetPose_Request__ros_msg_type * ros_message = static_cast<const _GetTargetPose_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: target_class
  {
    const rosidl_runtime_c__String * str = &ros_message->target_class;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _GetTargetPose_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetTargetPose_Request__ros_msg_type * ros_message = static_cast<_GetTargetPose_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: target_class
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->target_class.data) {
      rosidl_runtime_c__String__init(&ros_message->target_class);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->target_class,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'target_class'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtlebot3_pick_place
size_t get_serialized_size_turtlebot3_pick_place__srv__GetTargetPose_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetTargetPose_Request__ros_msg_type * ros_message = static_cast<const _GetTargetPose_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name target_class
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->target_class.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _GetTargetPose_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_turtlebot3_pick_place__srv__GetTargetPose_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtlebot3_pick_place
size_t max_serialized_size_turtlebot3_pick_place__srv__GetTargetPose_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: target_class
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = turtlebot3_pick_place__srv__GetTargetPose_Request;
    is_plain =
      (
      offsetof(DataType, target_class) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetTargetPose_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_turtlebot3_pick_place__srv__GetTargetPose_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetTargetPose_Request = {
  "turtlebot3_pick_place::srv",
  "GetTargetPose_Request",
  _GetTargetPose_Request__cdr_serialize,
  _GetTargetPose_Request__cdr_deserialize,
  _GetTargetPose_Request__get_serialized_size,
  _GetTargetPose_Request__max_serialized_size
};

static rosidl_message_type_support_t _GetTargetPose_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetTargetPose_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_pick_place, srv, GetTargetPose_Request)() {
  return &_GetTargetPose_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "turtlebot3_pick_place/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "turtlebot3_pick_place/srv/detail/get_target_pose__struct.h"
// already included above
// #include "turtlebot3_pick_place/srv/detail/get_target_pose__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/pose_stamped__functions.h"  // target_pose
// already included above
// #include "rosidl_runtime_c/string.h"  // message
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // message

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_turtlebot3_pick_place
size_t get_serialized_size_geometry_msgs__msg__PoseStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_turtlebot3_pick_place
size_t max_serialized_size_geometry_msgs__msg__PoseStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_turtlebot3_pick_place
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped)();


using _GetTargetPose_Response__ros_msg_type = turtlebot3_pick_place__srv__GetTargetPose_Response;

static bool _GetTargetPose_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetTargetPose_Response__ros_msg_type * ros_message = static_cast<const _GetTargetPose_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: target_pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->target_pose, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _GetTargetPose_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetTargetPose_Response__ros_msg_type * ros_message = static_cast<_GetTargetPose_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  // Field name: target_pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->target_pose))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtlebot3_pick_place
size_t get_serialized_size_turtlebot3_pick_place__srv__GetTargetPose_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetTargetPose_Response__ros_msg_type * ros_message = static_cast<const _GetTargetPose_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);
  // field.name target_pose

  current_alignment += get_serialized_size_geometry_msgs__msg__PoseStamped(
    &(ros_message->target_pose), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _GetTargetPose_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_turtlebot3_pick_place__srv__GetTargetPose_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtlebot3_pick_place
size_t max_serialized_size_turtlebot3_pick_place__srv__GetTargetPose_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: message
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: target_pose
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = turtlebot3_pick_place__srv__GetTargetPose_Response;
    is_plain =
      (
      offsetof(DataType, target_pose) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetTargetPose_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_turtlebot3_pick_place__srv__GetTargetPose_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetTargetPose_Response = {
  "turtlebot3_pick_place::srv",
  "GetTargetPose_Response",
  _GetTargetPose_Response__cdr_serialize,
  _GetTargetPose_Response__cdr_deserialize,
  _GetTargetPose_Response__get_serialized_size,
  _GetTargetPose_Response__max_serialized_size
};

static rosidl_message_type_support_t _GetTargetPose_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetTargetPose_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_pick_place, srv, GetTargetPose_Response)() {
  return &_GetTargetPose_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "turtlebot3_pick_place/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "turtlebot3_pick_place/srv/get_target_pose.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GetTargetPose__callbacks = {
  "turtlebot3_pick_place::srv",
  "GetTargetPose",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_pick_place, srv, GetTargetPose_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_pick_place, srv, GetTargetPose_Response)(),
};

static rosidl_service_type_support_t GetTargetPose__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GetTargetPose__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_pick_place, srv, GetTargetPose)() {
  return &GetTargetPose__handle;
}

#if defined(__cplusplus)
}
#endif
