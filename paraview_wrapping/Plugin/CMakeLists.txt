set(interfaces)
set(sources)

set(subdir_sources
  sqButtonPausePropertyWidget.cxx
  sqButtonPausePropertyWidget.h
  sqSlamManager.cxx
  sqSlamManager.h
  sqSlamStarter.cxx
  sqSlamStarter.h
)

foreach(filename ${subdir_sources})
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/UserInterface/${filename}"
    "${CMAKE_CURRENT_BINARY_DIR}/${filename}"
    COPYONLY
  )
  list(APPEND sources "${filename}")
endforeach()

paraview_plugin_add_property_widget(
  KIND WIDGET
  TYPE "command_pause_button"
  CLASS_NAME sqButtonPausePropertyWidget
  INTERFACES pause_widget_interfaces
  SOURCES  pause_widget_sources)

paraview_plugin_add_auto_start(
  CLASS_NAME sqSlamStarter
  STARTUP onStartup
  SHUTDOWN onShutdown
  INTERFACES autostart_interface
  SOURCES autostart_sources)

list(APPEND interfaces
  ${autostart_interface}
  ${pause_widget_interfaces}
)
list(APPEND sources
  ${autostart_sources}
  ${pause_widget_sources}
)

# Create Plugin
set(CMAKE_PROJECT_VERSION ${LidarSlam_VERSION})
paraview_add_plugin(LidarSlamPlugin
  REQUIRED_ON_CLIENT
  REQUIRED_ON_SERVER
  VERSION ${CMAKE_PROJECT_VERSION}
  UI_INTERFACES ${interfaces}
  SOURCES ${sources}
  MODULES
    vtkLidarSlam
    vtkSlamUtilities
  MODULE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/vtkLidarSlam/vtk.module"
    "${CMAKE_CURRENT_SOURCE_DIR}/vtkSlamUtilities/vtk.module"
  DOCUMENTATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doc/"
)

# Libraries required by UserInterface code
set(required_modules
  ParaView::pqApplicationComponents
  ParaView::pqComponents
  ParaView::pqCore
  ParaView::RemotingServerManager
  VTK::RenderingCore
)

target_link_libraries(LidarSlamPlugin PRIVATE ${required_modules})
