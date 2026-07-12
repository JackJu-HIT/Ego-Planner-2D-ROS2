cmake_minimum_required(VERSION 3.22)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -std=c++17 -g")
project(ego_planner)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

include_directories(${Boost_INCLUDE_DIRS})

include_directories(./planner/bspline_opt/include)
include_directories(./planner/path_searching/include)
include_directories(./planner/GridMap2D/include)
include_directories(./planner/plan_manage/include/plan_manage)
include_directories(./planner/plan_manage/include)
include_directories(./planner/include)

add_subdirectory(planner/plan_manage)
add_subdirectory(planner/bspline_opt)
add_subdirectory(planner/path_searching)
add_subdirectory(planner/GridMap2D)

find_package(rosidl_default_generators REQUIRED)

message(status ${PROJECT_NAME})

message(STATUS "包根目录：${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "检查msg文件：${CMAKE_CURRENT_SOURCE_DIR}/msg/CurrentPose.msg")

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/msg/CurrentPose.msg")
  message(FATAL_ERROR "❌ msg文件不存在！路径：${CMAKE_CURRENT_SOURCE_DIR}/msg/CurrentPose.msg")
else()
  message(STATUS "✅ msg文件存在！")
endif()
# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CurrentPose.msg"
  DEPENDENCIES std_msgs
)


# 编译节点
add_executable(motion_plan src/trajectory_publisher.cpp)
ament_target_dependencies(
  motion_plan
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  visualization_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(
  motion_plan
  bspline_opt  
  path_searching  
  local_map 
  ego_planner_lib  
  ${Boost_LIBRARIES} 
  "${cpp_typesupport_target}"
)
# 安装节点
install(TARGETS
  motion_plan
  DESTINATION lib/${PROJECT_NAME})

ament_package()