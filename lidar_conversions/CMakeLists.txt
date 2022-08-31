cmake_minimum_required(VERSION 3.9)
project(lidar_conversions VERSION 1.5)

# Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

##################
## Dependencies ##
##################

# Find ament_cmake and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)

###########
## Build ##
###########

# This package needs LidarPoint definition in LidarSlam lib.
# Sadly, adding lidar_slam dependency will create a circular dependency:
# - lidar_conversions needs a header file from LidarSlam, exported with lidar_slam: build dependency
# - lidar_slam uses lidar_conversions in its launchfiles: exec dependency
# These "soft" circular dependencies (build/exec) are not supported by catkin tools.
# To avoid this, we directly include the correct headers.
include_directories(
  "../../slam_lib/include/"
  "./external_include"
)

# Velodyne Lidar
add_executable(velodyne_conversion_node src/VelodyneToLidarNode.cxx)

#link dependencies to target
ament_target_dependencies(velodyne_conversion_node
  rclcpp
  pcl_ros
  pcl_conversions
  sensor_msgs
)


# Ouster Lidar
add_executable(ouster_conversion_node src/OusterToLidarNode.cxx)

#link dependencies to target
ament_target_dependencies(ouster_conversion_node
  rclcpp
  pcl_ros
  pcl_conversions
  sensor_msgs
)

# Robosense RSLidar
add_executable(robosense_conversion_node src/RobosenseToLidarNode.cxx)

#link dependencies to target
ament_target_dependencies(robosense_conversion_node
  rclcpp
  pcl_ros
  pcl_conversions
  sensor_msgs
)

#############
## Install ##
#############

install(TARGETS velodyne_conversion_node ouster_conversion_node robosense_conversion_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()