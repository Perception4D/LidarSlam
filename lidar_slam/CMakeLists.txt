cmake_minimum_required(VERSION 3.9)
project(lidar_slam VERSION 1.5)

# Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

##################
## Dependencies ##
##################

# Find ament macros and libraries
find_package(ament_cmake REQUIRED)
# Generate custom interfaces
find_package(rosidl_default_generators REQUIRED)

# Dependencies of LidarSlam
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag_ros REQUIRED)

###########
## Build ##
###########

# Build LidarSlam lib which lies in parent directory
set(slamlib_from_subdirectory true)
add_subdirectory(../.. ${CMAKE_BINARY_DIR}/slam)

# Generate slam interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Confidence.msg"
  "msg/SlamCommand.msg"
  "srv/SavePc.srv"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Generate C++ code to use custom interfaces in the same package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add LiDAR SLAM ROS node, link dependencies and librairies
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
  apriltag_ros
)
target_link_libraries(lidar_slam_node
  LidarSlam
  "${cpp_typesupport_target}"
)

# Add aggregation node, link dependencies and librairies
add_executable(aggregation_node
  src/AggregationNode.cxx
)
ament_target_dependencies(aggregation_node
  rclcpp
  pcl_conversions
  geometry_msgs
)
target_link_libraries(aggregation_node
  LidarSlam
  "${cpp_typesupport_target}"
)

#############
## Install ##
#############

install(TARGETS lidar_slam_node aggregation_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params
        DESTINATION share/${PROJECT_NAME})

# Installs the package.xml
# Installs config (and possibly target) files for CMake so that it can be found by other packages
ament_package()