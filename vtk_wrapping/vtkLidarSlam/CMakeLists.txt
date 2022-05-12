
# Build dynamic vtk Module
set(BUILD_SHARED_LIBS ON)

find_package(VTK REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(classes
  vtkSlam
  vtkSlamManager
  vtkSpinningSensorKeypointExtractor
)

# Create Module
vtk_module_add_module(vtkLidarSlam
  CLASSES ${classes}
)

target_link_libraries(vtkLidarSlam
  PUBLIC
    LidarSlam
    ${Boost_LIBRARIES}
    ${VTK_LIBRARIES}
)
target_include_directories(vtkLidarSlam
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIDARSLAM_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)
