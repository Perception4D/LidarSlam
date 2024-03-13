set(classes
  vtkLoopClosureActors
  vtkSlamFinder
)

vtk_module_add_module(vtkSlamUtilities
  FORCE_STATIC
  CLASSES ${classes}
)
