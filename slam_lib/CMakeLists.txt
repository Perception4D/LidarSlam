cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

# MSVC enforce MT
if(WIN32 AND MSVC)
	set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
endif()

# Boost do not use STATIC libs (Superbuild like), coherent with BUILD_SHARED_LIBS ON
set(Boost_USE_STATIC_LIBS OFF)

# Force build type to release for speed
set(CMAKE_BUILD_TYPE "Release")

find_package(Boost REQUIRED COMPONENTS thread)

find_package(nanoflann REQUIRED)

# Find Eigen3. If it defines the target, this is used. If not,
# fall back to the using the module form.
# See https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html for details
#set(EIGEN_DIR ${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake)
#set(Eigen_DIR ${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake)
#set(EIGEN3_DIR ${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake)
set(Eigen3_DIR ${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake)

find_package(Eigen3 REQUIRED)
if (TARGET Eigen3::Eigen)
  message(STATUS "Lidar SLAM : using Eigen3::Eigen target (version ${Eigen3_VERSION})")
  set(Eigen3_target Eigen3::Eigen)
else()
  message(STATUS "Lidar SLAM : using Eigen3 header files (version ${EIGEN3_VERSION})")
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

find_package(Ceres REQUIRED)

find_package(PCL REQUIRED COMPONENTS common kdtree registration io)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
#We do not want VTK for now, FIX WIN32 command too long issue
#PCL is linked finds VTK in paraview/src rather than paraview/build
list(FILTER PCL_LIBRARIES EXCLUDE REGEX "VTK::.*")

# Find optional g2o (only used for pose graph optimization)
option(ENABLE_g2o "Use G2O, necessary for pose graph optimization." ON)
if (ENABLE_g2o)
  find_package(g2o QUIET)
  if (NOT g2o_FOUND)
    message("Lidar SLAM : G2O requested but not found, pose graph optimization will be ignored.")
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
  message("Lidar SLAM : OpenMP not found")
endif()

# If G2O is available, compile pose graph optimization stuff
if (g2o_FOUND)
  set(SLAM_g2o_sources src/PoseGraphOptimization.cxx)
  set(g2o_targets g2o::core g2o::types_slam3d)
  add_definitions(-DUSE_G2O=1)
endif()

# Generate export symbols on Windows to use this lib
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(headers
  include/LidarSlam/CeresCostFunctions.h
  include/LidarSlam/ConfidenceEstimators.h
  include/LidarSlam/Enums.h
  include/LidarSlam/GlobalTrajectoriesRegistration.h
  include/LidarSlam/KDTreePCLAdaptor.h
  include/LidarSlam/KeypointsMatcher.h
  include/LidarSlam/LidarPoint.h
  include/LidarSlam/LocalOptimizer.h
  include/LidarSlam/MotionModel.h
  include/LidarSlam/PointCloudStorage.h
  include/LidarSlam/PoseGraphOptimization.h
  include/LidarSlam/RollingGrid.h
  include/LidarSlam/SensorConstraints.h
  include/LidarSlam/Slam.h
  include/LidarSlam/SpinningSensorKeypointExtractor.h
  include/LidarSlam/Transform.h
  include/LidarSlam/Utilities.h
)

add_library(LidarSlam SHARED
  src/ConfidenceEstimators.cxx
  src/GlobalTrajectoriesRegistration.cxx
  src/KeypointsMatcher.cxx
  src/LocalOptimizer.cxx
  src/MotionModel.cxx
  src/RollingGrid.cxx
  src/SensorConstraints.cxx
  src/Slam.cxx
  src/SpinningSensorKeypointExtractor.cxx
  src/Transform.cxx
  src/Utilities.cxx
  ${SLAM_g2o_sources}
  ${headers}
)

#file(GLOB_RECURSE headers "${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/*.h")
message(STATUS ${headers})

set_target_properties(LidarSlam PROPERTIES
  PUBLIC_HEADER "${headers}"
)

target_link_libraries(LidarSlam
  PUBLIC
    nanoflann::nanoflann
    ceres
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${g2o_targets}
    ${Eigen3_target}
    ${OpenMP_target}
)

target_include_directories(LidarSlam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam>
  $<INSTALL_INTERFACE:include/>
  $<INSTALL_INTERFACE:include/LidarSlam>
)

install(TARGETS LidarSlam
  EXPORT LidarSlam
  RUNTIME DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
  LIBRARY DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
  ARCHIVE DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
  PUBLIC_HEADER DESTINATION ${SLAM_INSTALL_INCLUDE_DIR}/LidarSlam
  COMPONENT Runtime
)

