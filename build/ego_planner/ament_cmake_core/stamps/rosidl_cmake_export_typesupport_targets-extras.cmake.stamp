# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:ego_planner__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:ego_planner__rosidl_typesupport_fastrtps_c;__rosidl_generator_cpp:ego_planner__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:ego_planner__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_c:ego_planner__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:ego_planner__rosidl_typesupport_c;__rosidl_typesupport_introspection_cpp:ego_planner__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:ego_planner__rosidl_typesupport_cpp;__rosidl_generator_py:ego_planner__rosidl_generator_py")

# populate ego_planner_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "ego_planner::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'ego_planner' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND ego_planner_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
