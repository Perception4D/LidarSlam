cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

project(LidarSlam
        LANGUAGES CXX
        VERSION 3.0)

#-------------------------
#  CMake parameters
#-------------------------

set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ standard")
message(STATUS "Building LidarSlam with C++${CMAKE_CXX_STANDARD} standard")

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "MinSizeRel" "RelWithDebInfo" "Release")
endif()

include(GNUInstallDirs) # Required for CMAKE_INSTALL_VARIABLES

# Runtime parameters
# /!\ warning : must be done before target declaration
list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")

# MSVC enforce MT
if(WIN32 AND MSVC)
	set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
endif()

#-------------------------
#  Find dependencies
#-------------------------

find_package(Boost REQUIRED)

find_package(nanoflann REQUIRED)

# Find Eigen3. If it defines the target, this is used. If not,
# fall back to the using the module form.
# See https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html for details
find_package(Eigen3 REQUIRED)
if (TARGET Eigen3::Eigen)
  message(STATUS "Lidar SLAM : using Eigen3::Eigen target (version ${Eigen3_VERSION})")
  set(Eigen3_target Eigen3::Eigen)
else()
  message(STATUS "Lidar SLAM : using Eigen3 header files (version ${EIGEN3_VERSION})")
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

find_package(Ceres REQUIRED)

# Find optional teaserpp (only used for automatic detection of loop closure)
option(ENABLE_teaserpp "Use TEASER++, necessary for automatic detection of loop closure." OFF)
if (ENABLE_teaserpp)
  find_package(teaserpp QUIET)
  if (NOT teaserpp_FOUND)
    message(STATUS "Lidar SLAM : TEASER++ requested but not found, automatic loop closure detection will be ignored.")
  endif()
  # pcl_for_teaser registers pcl components only when teaserpp is required
  set(pcl_for_teaser features kdtree)
endif()

find_package(PCL REQUIRED COMPONENTS common io octree geometry
             OPTIONAL_COMPONENTS ${pcl_for_teaser})
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Find optional g2o (only used for pose graph optimization)
option(ENABLE_g2o "Use G2O, necessary for pose graph optimization." OFF)
if (ENABLE_g2o)
  find_package(g2o QUIET)
  if (NOT g2o_FOUND)
    message(STATUS "Lidar SLAM : G2O requested but not found, pose graph optimization will be ignored.")
  endif()
endif()

# Find optional gtsam (only used for pose IMU raw data integration)
option(ENABLE_GTSAM "Use GTSAM, necessary for raw IMU data integration." OFF)
if (ENABLE_GTSAM)
  find_package(GTSAM QUIET)
  if (NOT GTSAM_FOUND)
    message(STATUS "Lidar SLAM : GTSAM requested but not found, IMU raw data will be ignored.")
  endif()
endif()

option(ENABLE_OpenCV "Use OpenCV, necessary for camera integration." ON)
if (ENABLE_OpenCV)
  find_package(OpenCV QUIET COMPONENTS core)
  if (NOT OpenCV_FOUND)
    message(STATUS "Lidar SLAM : OpenCV requested but not found, camera will be ignored.")
  endif()
endif()

# Find optional OpenMP
find_package(OpenMP)
if(TARGET OpenMP::OpenMP_CXX)
  message(STATUS "Lidar SLAM : using OpenMP ${OpenMP_CXX_VERSION} target")
  set(OpenMP_target OpenMP::OpenMP_CXX)
elseif(OpenMP_FOUND)
  message(STATUS "Lidar SLAM : using OpenMP ${OpenMP_VERSION} flags")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
  message(STATUS "Lidar SLAM : OpenMP not found")
endif()

#-------------------------
#  Build and install
#-------------------------

# Build core SLAM lib
add_subdirectory(slam_lib)

# To build/use ROS wrapping, just put this entire directory in the catkin src
# workspace and run `catkin_make` or `catkin build`.

#-------------------------
#  Install export files
#-------------------------

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_BINARY_DIR}/LidarSlamConfigVersion.cmake"
  VERSION "${CMAKE_PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in"
  "${CMAKE_BINARY_DIR}/LidarSlamConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LidarSlam"
)

install(
  FILES "${CMAKE_BINARY_DIR}/LidarSlamConfig.cmake"
        "${CMAKE_BINARY_DIR}/LidarSlamConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LidarSlam"
)
