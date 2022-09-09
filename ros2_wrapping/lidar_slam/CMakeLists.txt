cmake_minimum_required(VERSION 3.9)
project(lidar_slam VERSION 1.5)

# Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
#for custom interfaces
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(lidar_slam_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag_ros REQUIRED)

###########
## Build ##
###########

# Build LidarSlam lib which lies in parent directory
add_subdirectory(../.. ${CMAKE_BINARY_DIR}/slam)

# Add LiDAR SLAM ROS node
add_executable(lidar_slam_node
  src/LidarSlamNode.cxx
  src/LidarSlamNode_main.cxx
)
ament_target_dependencies(lidar_slam_node
  rclcpp
  tf2_ros
  pcl_conversions
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  #custom interfaces
  lidar_slam_interfaces
  apriltag_ros
)
target_link_libraries(lidar_slam_node
  LidarSlam
)

# Add aggregation node
add_executable(aggregation_node
  src/AggregationNode.cxx
)
ament_target_dependencies(aggregation_node
  rclcpp
  pcl_conversions
  geometry_msgs
  lidar_slam_interfaces
)
target_link_libraries(aggregation_node
  LidarSlam
)

#############
## Install ##
#############

install(TARGETS lidar_slam_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params
        DESTINATION share/${PROJECT_NAME})

ament_package()