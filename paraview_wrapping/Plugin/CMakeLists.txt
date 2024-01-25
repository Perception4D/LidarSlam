# Create Plugin
set(CMAKE_PROJECT_VERSION ${LidarSlam_VERSION})
paraview_add_plugin(LidarSlamPlugin
  REQUIRED_ON_CLIENT
  REQUIRED_ON_SERVER
  VERSION ${CMAKE_PROJECT_VERSION}
  MODULES
    vtkLidarSlam
    vtkSlamUtilities
  MODULE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/vtkLidarSlam/vtk.module"
    "${CMAKE_CURRENT_SOURCE_DIR}/vtkSlamUtilities/vtk.module"
  DOCUMENTATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doc/"
)
