//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Tong Fu (Kitware SAS)
// Creation date: 2024-11-29
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#ifndef VTK_MINI_SLAM_H
#define VTK_MINI_SLAM_H

#include <deque>

#include "vtkSlam.h"
#include <vtkMultiBlockDataSet.h>
#include <vtkSetGet.h>

class vtkMiniSlam : public vtkSlam
{
public:
  static vtkMiniSlam* New();
  vtkTypeMacro(vtkMiniSlam, vtkSlam)
  void PrintSelf(ostream& os, vtkIndent indent) override;

  //! @{ @copydoc NumberOfSlamFrames
  vtkGetMacro(NumberOfSlamFrames, unsigned int)
  vtkSetMacro(NumberOfSlamFrames, unsigned int)
  //! @}

protected:
  vtkMiniSlam();

  int FillOutputPortInformation(int port, vtkInformation* info) override;
  int RequestData(vtkInformation* request,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector) override;

private:
  vtkMiniSlam(const vtkMiniSlam&) = delete;
  void operator=(const vtkMiniSlam&) = delete;

  //! Number of transformed frames to display
  unsigned int NumberOfSlamFrames = 5;

  //! Cache to save output previously produced by the filter
  std::deque<vtkSmartPointer<vtkPolyData>> SlamFrames;
};

#endif // VTK_MINI_SLAM_H
