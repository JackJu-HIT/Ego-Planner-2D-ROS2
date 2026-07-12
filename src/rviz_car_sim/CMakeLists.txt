cmake_minimum_required(VERSION 3.8)
project(rviz_car_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

message(status ${PROJECT_NAME})
# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CurrentPose.msg"
  DEPENDENCIES std_msgs
)

add_executable(fake_sim_node src/fake_sim_node.cpp)
ament_target_dependencies(fake_sim_node
  rclcpp
  geometry_msgs
  visualization_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fake_sim_node "${cpp_typesupport_target}")

install(TARGETS
  fake_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
