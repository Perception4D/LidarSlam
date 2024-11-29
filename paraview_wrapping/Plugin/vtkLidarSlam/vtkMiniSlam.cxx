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

#include "vtkMiniSlam.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkStreamingDemandDrivenPipeline.h>

#include <sstream>

// vtkMiniSlam filter input ports (vtkPolyData and vtkTable)
#define LIDAR_FRAME_INPUT_PORT 0
#define INPUT_PORT_COUNT 1

#define SLAM_FRAMES_OUTPUT_PORT 0       ///< Current transformed SLAM frames
#define OUTPUT_PORT_COUNT 1

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkMiniSlam)

//----------------------------------------------------------------------------
vtkMiniSlam::vtkMiniSlam()
{
  // One input port
  this->SetNumberOfInputPorts(INPUT_PORT_COUNT);

  // The accumulation of stabilized frames
  this->SetNumberOfOutputPorts(OUTPUT_PORT_COUNT);

  this->SetProgressText("Computing slam");
}

//----------------------------------------------------------------------------
void vtkMiniSlam::PrintSelf(std::ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "MiniSlam parameters: " << std::endl;
  vtkIndent paramIndent = indent.GetNextIndent();
  #define PrintParameter(param) os << paramIndent << #param << "\t" << this->SlamAlgo->Get##param() << std::endl;

}

//----------------------------------------------------------------------------
int vtkMiniSlam::FillOutputPortInformation(int port, vtkInformation* info)
{
  if (port == 0)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkMultiBlockDataSet");
    return 1;
  }

  return 0;
}

//----------------------------------------------------------------------------
int vtkMiniSlam::RequestData(vtkInformation* request,
                             vtkInformationVector** inputVector,
                             vtkInformationVector* outputVector)
{
  // ===== Get input/output =====
  vtkPolyData* input = vtkPolyData::GetData(inputVector[LIDAR_FRAME_INPUT_PORT], 0);
  vtkMultiBlockDataSet* output = vtkMultiBlockDataSet::GetData(outputVector);
  // Check if input is a multiblock
  if (!input)
  {
    vtkMultiBlockDataSet* mb = vtkMultiBlockDataSet::GetData(inputVector[LIDAR_FRAME_INPUT_PORT], 0);
    // Extract first block if it is a vtkPolyData
    if (mb)
        input = vtkPolyData::SafeDownCast(mb->GetBlock(0));
  }
  // If the input could not be cast, return
  if (!input)
  {
    vtkErrorMacro(<< "Unable to cast input into a vtkPolyData");
    return 0;
  }
  // Check input is not empty
  vtkIdType nbPoints = input->GetNumberOfPoints();
  if (nbPoints == 0)
  {
      vtkErrorMacro(<< "Empty input data. Abort.");
      return 0;
  }
  // Check input format
  if (!vtkSlam::IdentifyInputArrays(input))
  {
    vtkErrorMacro(<< "Unable to identify LiDAR arrays to use. Please define them manually before processing the frame.");
    return 0;
  }

  return 1;
}
