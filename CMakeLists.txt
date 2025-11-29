cmake_minimum_required(VERSION 3.28)
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

target_link_libraries(
  motion_plan
  bspline_opt  
  path_searching  
  local_map 
  ego_planner  
  ${Boost_LIBRARIES} 
)

# 安装节点
install(TARGETS
  motion_plan
  DESTINATION lib/${PROJECT_NAME})

ament_package()