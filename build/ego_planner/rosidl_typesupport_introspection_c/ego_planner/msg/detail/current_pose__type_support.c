// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ego_planner:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ego_planner/msg/detail/current_pose__rosidl_typesupport_introspection_c.h"
#include "ego_planner/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ego_planner/msg/detail/current_pose__functions.h"
#include "ego_planner/msg/detail/current_pose__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ego_planner__msg__CurrentPose__init(message_memory);
}

void ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_fini_function(void * message_memory)
{
  ego_planner__msg__CurrentPose__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_message_member_array[3] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ego_planner__msg__CurrentPose, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ego_planner__msg__CurrentPose, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "theta",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ego_planner__msg__CurrentPose, theta),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_message_members = {
  "ego_planner__msg",  // message namespace
  "CurrentPose",  // message name
  3,  // number of fields
  sizeof(ego_planner__msg__CurrentPose),
  ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_message_member_array,  // message members
  ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_init_function,  // function to initialize message memory (memory has to be allocated)
  ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_message_type_support_handle = {
  0,
  &ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ego_planner
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ego_planner, msg, CurrentPose)() {
  if (!ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_message_type_support_handle.typesupport_identifier) {
    ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ego_planner__msg__CurrentPose__rosidl_typesupport_introspection_c__CurrentPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
