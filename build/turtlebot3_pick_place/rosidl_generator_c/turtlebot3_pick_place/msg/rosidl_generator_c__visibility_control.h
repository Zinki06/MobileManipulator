// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_PICK_PLACE__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define TURTLEBOT3_PICK_PLACE__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_turtlebot3_pick_place __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_turtlebot3_pick_place __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_turtlebot3_pick_place __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_turtlebot3_pick_place __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_turtlebot3_pick_place
    #define ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_pick_place ROSIDL_GENERATOR_C_EXPORT_turtlebot3_pick_place
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_pick_place ROSIDL_GENERATOR_C_IMPORT_turtlebot3_pick_place
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_turtlebot3_pick_place __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_turtlebot3_pick_place
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_pick_place __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_pick_place
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_PICK_PLACE__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
