//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2018-03-27
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

// LOCAL
#include "vtkSlam.h"
#include "vtkSpinningSensorKeypointExtractor.h"

// VTK
#include <vtkCellArray.h>
#include <vtkDataArray.h>
#include <vtkDelimitedTextReader.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkLine.h>
#include <vtkMath.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkTable.h>

// PCL
#include <pcl/common/transforms.h>

//Boost
// TODO : replace by std when passing to C++17 minimum
#include <boost/filesystem.hpp>

// vtkSlam filter input ports (vtkPolyData and vtkTable)
#define LIDAR_FRAME_INPUT_PORT 0       ///< Current LiDAR frame
#define INPUT_PORT_COUNT 1

// vtkSlam filter output ports (vtkPolyData)
#define SLAM_FRAME_OUTPUT_PORT 0       ///< Current transformed SLAM frame enriched with debug arrays
#define SLAM_TRAJECTORY_OUTPUT_PORT 1  ///< Trajectory (with position, orientation, covariance and time)
#define EDGE_MAP_OUTPUT_PORT 2         ///< Edge keypoints map
#define INTENSITY_EDGE_MAP_OUTPUT_PORT 3         ///< intensity edge keypoints map
#define PLANE_MAP_OUTPUT_PORT 4        ///< Plane keypoints map
#define EDGE_KEYPOINTS_OUTPUT_PORT 5   ///< Extracted edge keypoints from current frame
#define INTENSITY_EDGE_KEYPOINTS_OUTPUT_PORT 6         ///< intensity edge keypoints map
#define PLANE_KEYPOINTS_OUTPUT_PORT 7  ///< Extracted plane keypoints from current frame
#define OUTPUT_PORT_COUNT 8

#define IF_VERBOSE(minVerbosityLevel, command) if (this->SlamAlgo->GetVerbosity() >= (minVerbosityLevel)) { command; }

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlam)

namespace Utils
{
namespace
{
// Import helper functions from LidarSlam
using namespace LidarSlam::Utils;

//-----------------------------------------------------------------------------
template<typename T>
vtkSmartPointer<T> CreateArray(const std::string& Name, int NumberOfComponents = 1, int NumberOfTuples = 0)
{
  vtkSmartPointer<T> array = vtkSmartPointer<T>::New();
  array->SetNumberOfComponents(NumberOfComponents);
  array->SetNumberOfTuples(NumberOfTuples);
  array->SetName(Name.c_str());
  return array;
}

//-----------------------------------------------------------------------------
bool CheckTableFields(vtkTable* csvTable, std::vector<std::string> fields)
{
  bool allFieldsHere = true;
  for (const std::string& f : fields)
    allFieldsHere = allFieldsHere && csvTable->GetRowData()->HasArray(f.c_str());

  return allFieldsHere;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkDelimitedTextReader> CreateCSVLoader(const std::string& fileName, const std::string& delimiter)
{
  if (fileName.empty())
    return nullptr;

  vtkSmartPointer<vtkDelimitedTextReader> reader = vtkSmartPointer<vtkDelimitedTextReader>::New();
  reader->SetFileName(fileName.c_str());
  reader->DetectNumericColumnsOn();
  reader->SetHaveHeaders(true);
  reader->SetFieldDelimiterCharacters(delimiter.c_str());
  reader->Update();

  return reader;
}

//-----------------------------------------------------------------------------
bool IsPointValid(Eigen::Vector3d& point)
{
  if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z()))
    return false;

  double distance = point.norm();
  return distance > 1e-6 && distance < 1000.;
}

//-----------------------------------------------------------------------------
bool HasNanField(const LidarSlam::Slam::Point& point)
{
  return !std::isfinite(point.x)         ||
         !std::isfinite(point.y)         ||
         !std::isfinite(point.z)         ||
         !std::isfinite(point.time)      ||
         !std::isfinite(point.intensity);
}
} // end of anonymous namespace
} // end of Utils namespace

//-----------------------------------------------------------------------------
vtkSlam::vtkSlam()
: SlamAlgo(new LidarSlam::Slam)
{
  this->InitPose << 0.,0.,0.,0.,0.,0.;
  this->SetNumberOfInputPorts(INPUT_PORT_COUNT);
  this->SetNumberOfOutputPorts(OUTPUT_PORT_COUNT);
  // If auto-detect mode is disabled, user needs to specify input arrays to use
  this->SetInputArrayToProcess(0, LIDAR_FRAME_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "timestamp");
  this->SetInputArrayToProcess(1, LIDAR_FRAME_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "intensity");
  this->SetInputArrayToProcess(2, LIDAR_FRAME_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "laser_id");

  // Init slam internal state
  this->SlamAlgo->Reset(true);

  // Enable overlap computation only if required
  this->SlamAlgo->SetOverlapSamplingRatio(this->AdvancedReturnMode ||
                                          this->SlamAlgo->GetFailureDetectionEnabled() ?
                                          this->OverlapSamplingRatio :
                                          0.);
  // Enable motion metrics and averages/derivatives computation if required
  this->SlamAlgo->SetConfidenceWindow(this->AdvancedReturnMode ||
                                      this->SlamAlgo->GetFailureDetectionEnabled() ?
                                      this->ConfidenceWindow :
                                      0.);

  // As the user has supervision on the loop closure detection in PV,
  // the threshold validation value is set to a minimal low value (0.1)
  this->SlamAlgo->SetLoopEvaluationThreshold(0.1);

  // Init PV trajectory for SLAM trajectory output
  this->ResetTrajectory();
}

//-----------------------------------------------------------------------------
void vtkSlam::Reset()
{
  // Reset SLAM time to remove doubled frames
  this->FrameTime = -1;

  if (this->SlamAlgo->IsRecovery())
    vtkWarningMacro(<< "Getting out of recovery mode");

  // Reset slam internal state
  this->SlamAlgo->Reset(true);

  // Init PV trajectory for SLAM trajectory output
  // /!\ Must be done before initializing the SLAM pose/maps
  this->ResetTrajectory();

  // Init the SLAM state (map + pose)
  this->SetInitialSlam();

  // Refill sensor managers
  this->SetSensorData(this->ExtSensorFileName);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::OptimizeGraphWithIMU()
{
  const std::list<LidarSlam::LidarState>& initLidarStates = this->SlamAlgo->GetLogStates();
  if (initLidarStates.size() < 2)
    return;
  this->SlamAlgo->UpdateTrajectoryAndMapsWithIMU();
  // Update PV trajectory poses that have been optimized by the SLAM
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  // Keep old poses that have not been optimized
  this->ResetTrajectory(lidarStates.front().Time);
  for (auto const& state: lidarStates)
    this->AddPoseToTrajectory(state);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::DetectLoop()
{
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  if (lidarStates.size() < 2)
    return;

  if (!this->SlamAlgo->DetectLoopClosureIndices(this->LastLoopInfo))
  {
    vtkWarningMacro(<< "Loop closure could not be detected automatically!");
    return;
  }

  this->SetLoopDetected(true);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::AddLoopDetection()
{
  if (this->LoopDetected)
    this->SlamAlgo->AddLoopClosureIndices(this->LastLoopInfo);

  this->SetLoopDetected(false);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::ClearLoopDetections()
{
  this->SlamAlgo->ClearLoopDetections();
}

//-----------------------------------------------------------------------------
void vtkSlam::OptimizeGraph()
{
  if (!this->SlamAlgo->OptimizeGraph())
    return;
  // Add external Pose offset to slam odom
  if (this->EnableExternalPoseOffset)
    this->SlamAlgo->AddExternalPoseOffsetToOdom();
  else
    this->SlamAlgo->ResetToSlamOdom();

  // Update PV trajectory poses that have been optimized by the SLAM
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  // Keep old poses that have not been optimized
  this->ResetTrajectory(lidarStates.front().Time);
  for (auto const& state: lidarStates)
    this->AddPoseToTrajectory(state);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::EnablePGOConstraintLoopClosure(bool enabled)
{
  vtkDebugMacro(<< "Enabling loop closure constraint for pose graph optimization");
  this->SlamAlgo->EnablePGOConstraint(LidarSlam::PGOConstraint::LOOP_CLOSURE, enabled);
}

void vtkSlam::EnablePGOConstraintLandmark(bool enabled)
{
  vtkDebugMacro(<< "Enabling landmark constraint for pose graph optimization");
  this->SlamAlgo->EnablePGOConstraint(LidarSlam::PGOConstraint::LANDMARK, enabled);
}

void vtkSlam::EnablePGOConstraintGPS(bool enabled)
{
  vtkDebugMacro(<< "Enabling GPS constraint for pose graph optimization");
  this->SlamAlgo->EnablePGOConstraint(LidarSlam::PGOConstraint::GPS, enabled);
}

void vtkSlam::EnablePGOConstraintExtPose(bool enabled)
{
  vtkDebugMacro(<< "Enabling ext pose constraint for pose graph optimization");
  this->SlamAlgo->EnablePGOConstraint(LidarSlam::PGOConstraint::EXT_POSE, enabled);
}

//-----------------------------------------------------------------------------
bool vtkSlam::GetPGOConstraintLoopClosure()
{
  bool enabled = this->SlamAlgo->IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LOOP_CLOSURE);
  if (enabled)
    vtkDebugMacro(<< "Loop closure constraint for PGO is enabled");
  else
    vtkDebugMacro(<< "Loop closure constraint for PGO is disabled");
  return enabled;
}

bool vtkSlam::GetPGOConstraintLandmark()
{
  bool enabled = this->SlamAlgo->IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LANDMARK);
  if (enabled)
    vtkDebugMacro(<< "Landmark constraint for PGO is enabled");
  else
    vtkDebugMacro(<< "Landmark constraint for PGO is disabled");
  return enabled;
}

bool vtkSlam::GetPGOConstraintGPS()
{
  bool enabled = this->SlamAlgo->IsPGOConstraintEnabled(LidarSlam::PGOConstraint::GPS);
  if (enabled)
    vtkDebugMacro(<< "GPS constraint for PGO is enabled");
  else
    vtkDebugMacro(<< "GPS constraint for PGO is disabled");
  return enabled;
}

bool vtkSlam::GetPGOConstraintExtPose()
{
  bool enabled = this->SlamAlgo->IsPGOConstraintEnabled(LidarSlam::PGOConstraint::EXT_POSE);
  if (enabled)
    vtkDebugMacro(<< "Ext pose constraint for PGO is enabled");
  else
    vtkDebugMacro(<< "Ext pose constraint for PGO is disabled");
  return enabled;
}

//-----------------------------------------------------------------------------
std::vector<bool> vtkSlam::GetArePointsValid()
{
  return this->ArePointsValid;
}

//-----------------------------------------------------------------------------
void vtkSlam::ClearMapsAndLog()
{
  vtkDebugMacro(<< "Clearing the maps and the log");
  this->SlamAlgo->ClearLocalMaps();
  this->SlamAlgo->ClearLog();
  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialSlam()
{
  // Check number of log states
  const std::list<LidarSlam::LidarState>& initLidarStates = this->SlamAlgo->GetLogStates();
  double motion = (initLidarStates.front().Isometry.translation() - initLidarStates.back().Isometry.translation()).norm();
  auto initPose = LidarSlam::Utils::XYZRPYtoIsometry(this->InitPose);
  // Only set initial pose if it is not identity
  if (!initPose.matrix().isIdentity(1e-6))
  {
    // Before setting initial pose, check whether or not a slam trajectory exist
    if ((initLidarStates.size() <= 2 || motion < this->SlamAlgo->GetKfDistanceThreshold()))
    {
      // Reset slam
      this->SlamAlgo->Reset(true);
      // Init the output SLAM trajectory
      this->ResetTrajectory();
      // There is no log states, jump to initial pose and the pose is added to log states
      this->SlamAlgo->JumpPose(initPose);
      // Set TworldInit
      this->SlamAlgo->SetTworldInit(initPose);
      // Update PV trajectory
      this->AddLastPosesToTrajectory();
    }
    else
    {
      vtkWarningMacro(<< "Could not initialize the SLAM because a trajectory already exists : "
                      << "please reset manually if you wish to proceed with initialization.");
    }
  }
  // Set initial maps for slam if they are provided
  if (!this->InitMapPrefix.empty())
  {
    if (this->InitMapPrefix.substr(this->InitMapPrefix.find('.') + 1, this->InitMapPrefix.size()) == "pcd")
      vtkErrorMacro(<< "Could not load the initial map : only the prefix path must be supplied (not the complete path)");
    else
      this->SlamAlgo->LoadMapsFromPCD(this->InitMapPrefix);
  }

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialPoseTranslation(double x, double y, double z)
{
  vtkDebugMacro(<< "Setting InitialPoseTranslation to " << x << " " << y << " " << z);
  this->InitPose.x() = x;
  this->InitPose.y() = y;
  this->InitPose.z() = z;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialPoseRotation(double roll, double pitch, double yaw)
{
  vtkDebugMacro(<< "Setting InitialPoseRotation to " << roll << " " << pitch << " " << yaw);
  this->InitPose(3) = roll;
  this->InitPose(4) = pitch;
  this->InitPose(5) = yaw;
}

//-----------------------------------------------------------------------------
int vtkSlam::RequestData(vtkInformation* vtkNotUsed(request),
                         vtkInformationVector** inputVector,
                         vtkInformationVector* outputVector)
{
  IF_VERBOSE(1, Utils::Timer::Init("vtkSlam"));
  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : input conversions"));

  // Get the input
  vtkPolyData* input = vtkPolyData::GetData(inputVector[LIDAR_FRAME_INPUT_PORT], 0);
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
  if (!this->IdentifyInputArrays(input))
  {
    vtkErrorMacro(<< "Unable to identify LiDAR arrays to use. Please define them manually before processing the frame.");
    return 0;
  }

  // Get frame time
  auto arrayTime = input->GetPointData()->GetArray(this->TimeArrayName.c_str());
  this->LastFrameTime = this->FrameTime;
  // Get frame packet reception time
  double frameReceptionPOSIXTime = -1.;
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  if (inInfo->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
    frameReceptionPOSIXTime = inInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
  this->FrameTime = this->PointTimeRelativeToFrame ? frameReceptionPOSIXTime : arrayTime->GetRange()[1];

  if (this->LastFrameTime == this->FrameTime)
      vtkDebugMacro(<< "Timestamp has not changed. Skipping frame.");
  if (nbPoints < 100)
      vtkErrorMacro(<< "Input point cloud does not contain enough points. Skipping frame");

  // Conversion vtkPolyData -> PCL pointcloud
  if (this->LastFrameTime != this->FrameTime && nbPoints >= 100)
  {
      LidarSlam::Slam::PointCloud::Ptr pc(new LidarSlam::Slam::PointCloud);
      this->PolyDataToPointCloud(input, pc);

      // Get frame first point time in vendor format
      double* range = arrayTime->GetRange();
      double frameFirstPointTime = range[0] * this->TimeToSecondsFactor;
      if (this->SynchronizeOnPacket)
      {
          // Compare potential offset with current offset
          double absCurrentOffset = std::abs(this->SlamAlgo->GetSensorTimeOffset());
          double potentialOffset = frameFirstPointTime - frameReceptionPOSIXTime;
          // We exclude the first frame cause frameReceptionPOSIXTime can be badly set
          if (this->SlamAlgo->GetNbrFrameProcessed() > 0 && (absCurrentOffset < 1e-6 || std::abs(potentialOffset) < absCurrentOffset))
              this->SlamAlgo->SetSensorTimeOffset(potentialOffset);
      }

      // Run SLAM
      IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : input conversions"));
      this->SlamAlgo->AddFrame(pc);
  }
  else
    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : input conversions"));


  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : basic output conversions"));

  // ===== SLAM frame =====
  // Output : Current undistorted LiDAR frame in world coordinates
  auto* slamFrame = vtkPolyData::GetData(outputVector, SLAM_FRAME_OUTPUT_PORT);
  slamFrame->ShallowCopy(input);
  auto worldFrame = this->SlamAlgo->GetRegisteredFrame();
  // Modify only points coordinates to keep input arrays
  auto registeredPoints = vtkSmartPointer<vtkPoints>::New();
  registeredPoints->SetNumberOfPoints(nbPoints);
  slamFrame->SetPoints(registeredPoints);

  if (!worldFrame->empty())
  {
    unsigned int validFrameIndex = 0;
    for (vtkIdType i = 0; i < nbPoints; i++)
    {
      // Modify point only if valid
      double pos[3];
      input->GetPoint(i, pos);
      if (this->ArePointsValid[i])
      {
        const auto& p = worldFrame->points[validFrameIndex++];
        registeredPoints->SetPoint(i, p.data);
      }
      else
        registeredPoints->SetPoint(i, pos);
    }
  }

  IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : basic output conversions"));

  // ===== Aggregated Keypoints maps =====
  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : output keypoints maps"));

  // Get the previous outputs
  auto* edgeMap          = vtkPolyData::GetData(outputVector, EDGE_MAP_OUTPUT_PORT);
  auto* planarMap        = vtkPolyData::GetData(outputVector, PLANE_MAP_OUTPUT_PORT);
  auto* intensityEdgeMap = vtkPolyData::GetData(outputVector, INTENSITY_EDGE_MAP_OUTPUT_PORT);

  if ((this->SlamAlgo->GetNbrFrameProcessed() - 1) % this->MapsUpdateStep == 0)
  {
    // Cache maps to update them only every MapsUpdateStep frames
    for (auto k : LidarSlam::KeypointTypes)
      this->CacheMaps[k] = vtkSmartPointer<vtkPolyData>::New();

    // The expected maps can be the whole maps or the submaps
    // If the maps is fixed by the user, the whole map and the submap are equal but the submap is outputed (faster)
    switch (this->OutputKeypointsMaps)
    {
      // Output the whole maps that are available
      case OutputKeypointsMapsMode::FULL_MAPS :
        for (auto k : LidarSlam::KeypointTypes)
        {
          if (this->SlamAlgo->KeypointTypeEnabled(k))
            this->PointCloudToPolyData(this->SlamAlgo->GetMap(k), this->CacheMaps[k]);
        }
        break;

      // Output the submaps that are available
      case OutputKeypointsMapsMode::SUB_MAPS :
        for (auto k : LidarSlam::KeypointTypes)
        {
          if (this->SlamAlgo->KeypointTypeEnabled(k))
            this->PointCloudToPolyData(this->SlamAlgo->GetTargetSubMap(k), this->CacheMaps[k]);
        }
        break;

      // If no map should be outputed, let the maps empty
      case OutputKeypointsMapsMode::NONE :
        break;

      default:
        break;
    }
  }

  // Fill outputs from cache
  edgeMap->ShallowCopy(this->CacheMaps[LidarSlam::EDGE]);
  planarMap->ShallowCopy(this->CacheMaps[LidarSlam::PLANE]);
  intensityEdgeMap->ShallowCopy(this->CacheMaps[LidarSlam::INTENSITY_EDGE]);

  IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : output keypoints maps"));

  // ===== Extracted keypoints from current frame =====
  if (this->OutputCurrentKeypoints)
  {
    IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : output current keypoints"));
    for (auto k : LidarSlam::KeypointTypes)
    {
      int port = EDGE_KEYPOINTS_OUTPUT_PORT + static_cast<int>(k);
      if (port >= OUTPUT_PORT_COUNT)
        continue;
      auto* keyoints = vtkPolyData::GetData(outputVector, port);
      if (this->SlamAlgo->KeypointTypeEnabled(k))
        this->PointCloudToPolyData(this->SlamAlgo->GetKeypoints(k, this->OutputKeypointsInWorldCoordinates), keyoints);
      else
        this->PointCloudToPolyData(LidarSlam::Slam::PointCloud::Ptr(new LidarSlam::Slam::PointCloud), keyoints);
    }
    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : output current keypoints"));
  }

  // Add debug information if advanced return mode is enabled
  if (this->AdvancedReturnMode)
  {
    IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : add advanced return arrays"));

    // Keypoints extraction debug array (curvatures, depth gap, intensity gap...)
    // Arrays added to WORLD transformed frame output
    auto* slamFrame = vtkPolyData::GetData(outputVector, SLAM_FRAME_OUTPUT_PORT);
    auto keypointsExtractionDebugArray = this->SlamAlgo->GetKeyPointsExtractor()->GetDebugArray();
    for (const auto& it : keypointsExtractionDebugArray)
    {
      auto array = Utils::CreateArray<vtkFloatArray>(it.first.c_str(), 1, nbPoints);
      slamFrame->GetPointData()->AddArray(array);

      // Fill array values from debug data
      // memcpy is a better alternative than looping on all tuples
      // but can only be used if the arrays use continuous storage
      std::memcpy(array->GetVoidPointer(0), it.second.data(), sizeof(float) * it.second.size());
    }

    // ICP keypoints matching results for ego-motion registration or localization steps
    // Arrays added to keypoints extracted from current frame outputs
    if (this->OutputCurrentKeypoints)
    {
      std::unordered_map<std::string, vtkPolyData*> outputMap;
      if (this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE))
      {
        auto* edgePoints = vtkPolyData::GetData(outputVector, EDGE_KEYPOINTS_OUTPUT_PORT);
        outputMap["EgoMotion: edge matches"]     = edgePoints;
        outputMap["EgoMotion: edge weights"]     = edgePoints;
        outputMap["Localization: edge matches"]  = edgePoints;
        outputMap["Localization: edge weights"]  = edgePoints;
      }
      if (this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE))
      {
        auto* planarPoints = vtkPolyData::GetData(outputVector, PLANE_KEYPOINTS_OUTPUT_PORT);
        outputMap["EgoMotion: plane matches"]    = planarPoints;
        outputMap["EgoMotion: plane weights"]    = planarPoints;
        outputMap["Localization: plane matches"] = planarPoints;
        outputMap["Localization: plane weights"] = planarPoints;

      }

      auto debugArray = this->SlamAlgo->GetDebugArray();
      for (const auto& it : outputMap)
      {
        auto array = Utils::CreateArray<vtkDoubleArray>(it.first.c_str(), 1, debugArray[it.first].size());
        // memcpy is a better alternative than looping on all tuples
        std::memcpy(array->GetVoidPointer(0), debugArray[it.first].data(), sizeof(double) * debugArray[it.first].size());
        it.second->GetPointData()->AddArray(array);
      }
    }

    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : add advanced return arrays"));
  }

  // Update trajectory
  if (this->LastFrameTime != this->FrameTime && nbPoints >= 100)
  {
      // If SLAM had failed before
      if (this->SlamAlgo->IsRecovery())
      {
          // TMP : in the future, the user should have a look
          // at the result to validate recovery
          // Check if the SLAM can go on and pose has to be displayed
          if (this->SlamAlgo->GetOverlapEstimation() > 0.2f &&
              this->SlamAlgo->GetPositionErrorStd()  < 0.1f)
          {
              vtkWarningMacro(<< "Getting out of recovery mode");
              // Frame is relocalized, reset params
              this->SlamAlgo->EndRecovery();
              this->AddLastPosesToTrajectory();
          }
          else
              vtkWarningMacro(<< "Still waiting for recovery");
      }
      // Checking failure and add or not the poses
      else if (this->SlamAlgo->HasFailed())
      {
          vtkErrorMacro(<< "SLAM has failed : entering recovery mode :\n"
              << "\t -Maps will not be updated\n"
              << "\t -Egomotion and undistortion are disabled\n"
              << "\t -The number of ICP iterations is increased\n"
              << "\t -The maximum distance between a frame point and a map target point is increased");
          // Enable recovery mode :
          // Last frames are removed
          // Maps are not updated
          // Param are tuned to handle bigger motions
          // Warning : real time is not ensured
          this->SlamAlgo->StartRecovery(this->RecoveryTime);
          // Remove newest trajectory poses
          this->ResetTrajectory(this->SlamAlgo->GetLogStates().back().Time);
      }
      else
          this->AddLastPosesToTrajectory();
  }

  // Output : SLAM Trajectory
  auto* slamTrajectory = vtkPolyData::GetData(outputVector, SLAM_TRAJECTORY_OUTPUT_PORT);
  slamTrajectory->DeepCopy(this->Trajectory);

  IF_VERBOSE(1, Utils::Timer::StopAndDisplay("vtkSlam"));

  return 1;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetImuGravity(double x, double y, double z)
{
  vtkDebugMacro(<< "Setting ImuGravity to " << x << " " << y << " " << z);
  this->SlamAlgo->SetImuGravity(Eigen::Vector3d({x, y, z}));
  // refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
bool vtkSlam::GetCalibrationMatrix(const std::string& fileName, Eigen::Isometry3d& calibration) const
{
  // Look for file
  std::ifstream fin (fileName);
  calibration = Eigen::Isometry3d::Identity();
  if (fin.is_open())
  {
    // Parse elements
    int idxElement = 0;
    while (fin.good() && idxElement < 16)
    {
      std::string elementString;
      fin >> elementString;
      try
      {
        calibration.matrix()(idxElement) = std::stof(elementString);
      }
      catch (...)
      {
        vtkWarningMacro(<< "Calibration file not well formed"
                        << " -> calibration is set to identity");
        calibration = Eigen::Isometry3d::Identity();
        return false;
      }
      ++idxElement;
    }
    if (idxElement < 16)
    {
        vtkWarningMacro(<< "Calibration file not well formed"
                        << " -> calibration is set to identity");
        calibration = Eigen::Isometry3d::Identity();
        return false;
    }
    calibration.matrix().transposeInPlace();
  }
  else
  {
    vtkWarningMacro(<< "No calibration file named "
                    << fileName << " was found");
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetSensorData(const std::string& fileName)
{
  vtkDebugMacro(<< "Setting sensor data from " << fileName);
  this->ExtSensorFileName = fileName;
  // Empty current measurements and reset local sensor params
  this->SlamAlgo->ResetSensors(true);

  std::string delimiter = " ;,";
  vtkSmartPointer<vtkDelimitedTextReader> reader = Utils::CreateCSVLoader(fileName, delimiter);
  if (!reader)
     return;
  vtkTable* csvTable = reader->GetOutput();

  // Check if time exists and extract it
  if (!Utils::CheckTableFields(csvTable, {"time"}))
  {
    vtkErrorMacro(<< "No time found in external sensor file, loading aborted");
    return;
  }
  auto arrayTime = csvTable->GetRowData()->GetArray("time");
  if (arrayTime->GetNumberOfTuples() == 0)
  {
    vtkErrorMacro(<< "No measure found in external sensor file");
    return;
  }
  // Set the maximum number of measurements stored in the SLAM filter
  this->SlamAlgo->SetSensorMaxMeasures(arrayTime->GetNumberOfTuples());

  // Look for a calibration file next to first file
  boost::filesystem::path path(fileName);
  std::string calibFileName = (path.parent_path() / "calibration_external_sensor.mat").string();
  // Set calibration
  Eigen::Isometry3d base2Sensor;
  if (this->GetCalibrationMatrix(calibFileName, base2Sensor))
    vtkWarningMacro(<< this->GetClassName() << " (" << this
                    << "): Calibration loaded : \n"
                    << base2Sensor.matrix());
  else
    vtkWarningMacro(<< this->GetClassName() << " (" << this
                    << "): Calibration has not been loaded. Make sure to calibrate your sensor before using it.");

  bool extSensorFit = false;

  // Process wheel odometer data
  if (Utils::CheckTableFields(csvTable, {"odom"}))
  {
    this->SlamAlgo->SetWheelOdomCalibration(base2Sensor);
    auto arrayOdom = csvTable->GetRowData()->GetArray("odom");
    for (vtkIdType i = 0; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      LidarSlam::ExternalSensors::WheelOdomMeasurement odomMeasurement;
      odomMeasurement.Time = arrayTime->GetTuple1(i);
      odomMeasurement.Distance = arrayOdom->GetTuple1(i);
      this->SlamAlgo->AddWheelOdomMeasurement(odomMeasurement);
    }
    PRINT_INFO("Odometry data successfully loaded")
    extSensorFit = true;
  }

  // Process IMU data
  #ifdef USE_GTSAM
  if (Utils::CheckTableFields(csvTable, {"acc_x", "acc_y", "acc_z", "w_x", "w_y", "w_z"}))
  {
    // Deduce frequency using time array
    // Build a frequency histogram and extract max bin
    // (in case of cuts in the data or missing measures)
    std::map<int,int> freqHistogram;
    for (vtkIdType i = 1; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      int freq = std::round(1. / (arrayTime->GetTuple1(i) - arrayTime->GetTuple1(i - 1)));
      ++freqHistogram[freq];
    }
    int frequency = std::max_element(freqHistogram.begin(), freqHistogram.end(),
                                     [](const auto &x, const auto &y)
                                     {return x.second < y.second;})->first;

    vtkDebugMacro(<< "IMU frequency detected is " << frequency << " Hz");
    this->SlamAlgo->SetImuFrequency(frequency);

    this->SlamAlgo->SetImuCalibration(base2Sensor);
    auto arrayAccX = csvTable->GetRowData()->GetArray("acc_x");
    auto arrayAccY = csvTable->GetRowData()->GetArray("acc_y");
    auto arrayAccZ = csvTable->GetRowData()->GetArray("acc_z");
    auto arrayVelR = csvTable->GetRowData()->GetArray("w_x");
    auto arrayVelP = csvTable->GetRowData()->GetArray("w_y");
    auto arrayVelY = csvTable->GetRowData()->GetArray("w_z");
    for (vtkIdType i = 0; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      LidarSlam::ExternalSensors::ImuMeasurement imuMeasurement;
      imuMeasurement.Time = arrayTime->GetTuple1(i);
      imuMeasurement.Acceleration.x()  = arrayAccX->GetTuple1(i);
      imuMeasurement.Acceleration.y()  = arrayAccY->GetTuple1(i);
      imuMeasurement.Acceleration.z()  = arrayAccZ->GetTuple1(i);
      imuMeasurement.AngleVelocity.x() = arrayVelR->GetTuple1(i);
      imuMeasurement.AngleVelocity.y() = arrayVelP->GetTuple1(i);
      imuMeasurement.AngleVelocity.z() = arrayVelY->GetTuple1(i);
      this->SlamAlgo->AddImuMeasurement(imuMeasurement);
    }
    PRINT_INFO("IMU data successfully loaded");
    extSensorFit = true;
  }
  else if (Utils::CheckTableFields(csvTable, {"acc_x", "acc_y", "acc_z"}))
  {
  #else
  if (Utils::CheckTableFields(csvTable, {"acc_x", "acc_y", "acc_z"}))
  {
    this->SlamAlgo->SetGravityCalibration(base2Sensor);
  #endif
    auto arrayAccX = csvTable->GetRowData()->GetArray("acc_x");
    auto arrayAccY = csvTable->GetRowData()->GetArray("acc_y");
    auto arrayAccZ = csvTable->GetRowData()->GetArray("acc_z");
    for (vtkIdType i = 0; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      LidarSlam::ExternalSensors::GravityMeasurement gravityMeasurement;
      gravityMeasurement.Time = arrayTime->GetTuple1(i);
      gravityMeasurement.Acceleration.x() = arrayAccX->GetTuple1(i);
      gravityMeasurement.Acceleration.y() = arrayAccY->GetTuple1(i);
      gravityMeasurement.Acceleration.z() = arrayAccZ->GetTuple1(i);
      this->SlamAlgo->AddGravityMeasurement(gravityMeasurement);
    }
    PRINT_INFO("IMU data successfully loaded for gravity integration");
    extSensorFit = true;
  }

  // Process external pose data
  // 1_ Format XYZRPY
  if (Utils::CheckTableFields(csvTable, {"x", "y", "z", "roll", "pitch", "yaw"}))
  {
    this->SlamAlgo->SetPoseCalibration(base2Sensor);
    auto arrayX     = csvTable->GetRowData()->GetArray("x"    );
    auto arrayY     = csvTable->GetRowData()->GetArray("y"    );
    auto arrayZ     = csvTable->GetRowData()->GetArray("z"    );
    auto arrayRoll  = csvTable->GetRowData()->GetArray("roll" );
    auto arrayPitch = csvTable->GetRowData()->GetArray("pitch");
    auto arrayYaw   = csvTable->GetRowData()->GetArray("yaw"  );

    for (vtkIdType i = 0; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      LidarSlam::ExternalSensors::PoseMeasurement meas;
      meas.Time = arrayTime->GetTuple1(i);
      // Derive Isometry
      meas.Pose.linear() = Eigen::Matrix3d(
                           Eigen::AngleAxisd(arrayYaw->GetTuple1(i),   Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(arrayPitch->GetTuple1(i), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(arrayRoll->GetTuple1(i),  Eigen::Vector3d::UnitX())
                           );
      meas.Pose.translation() = Eigen::Vector3d(arrayX->GetTuple1(i), arrayY->GetTuple1(i), arrayZ->GetTuple1(i));
      meas.Pose.makeAffine();
      meas.Covariance = LidarSlam::Utils::CreateDefaultCovariance();
      this->SlamAlgo->AddPoseMeasurement(meas);
    }
    PRINT_INFO("Pose data successfully loaded")
    extSensorFit = true;
  }

  // 2_ Matrix format
  else if (Utils::CheckTableFields(csvTable, {"x", "y", "z",
                                              "x0", "x1", "x2",
                                              "y0", "y1", "y2",
                                              "z0", "z1", "z2"}))
  {
    this->SlamAlgo->SetPoseCalibration(base2Sensor);
    auto arrayX   = csvTable->GetRowData()->GetArray("x");
    auto arrayY   = csvTable->GetRowData()->GetArray("y");
    auto arrayZ   = csvTable->GetRowData()->GetArray("z");
    auto arrayX0  = csvTable->GetRowData()->GetArray("x0" );
    auto arrayY0  = csvTable->GetRowData()->GetArray("y0" );
    auto arrayZ0  = csvTable->GetRowData()->GetArray("z0" );
    auto arrayX1  = csvTable->GetRowData()->GetArray("x1" );
    auto arrayY1  = csvTable->GetRowData()->GetArray("y1" );
    auto arrayZ1  = csvTable->GetRowData()->GetArray("z1" );
    auto arrayX2  = csvTable->GetRowData()->GetArray("x2" );
    auto arrayY2  = csvTable->GetRowData()->GetArray("y2" );
    auto arrayZ2  = csvTable->GetRowData()->GetArray("z2" );

    for (vtkIdType i = 0; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      LidarSlam::ExternalSensors::PoseMeasurement meas;
      meas.Time = arrayTime->GetTuple1(i);
      // Derive Isometry
      meas.Pose.matrix() << arrayX0->GetTuple1(i), arrayX1->GetTuple1(i), arrayX2->GetTuple1(i), arrayX->GetTuple1(i),
                            arrayY0->GetTuple1(i), arrayY1->GetTuple1(i), arrayY2->GetTuple1(i), arrayY->GetTuple1(i),
                            arrayZ0->GetTuple1(i), arrayZ1->GetTuple1(i), arrayZ2->GetTuple1(i), arrayZ->GetTuple1(i),
                            0, 0, 0, 1;
      meas.Covariance = LidarSlam::Utils::CreateDefaultCovariance();
      this->SlamAlgo->AddPoseMeasurement(meas);
    }
    PRINT_INFO("Pose data successfully loaded")
    extSensorFit = true;
  }

  // Process GPS data
  else if (Utils::CheckTableFields(csvTable, {"x", "y", "z"}))
  {
    this->SlamAlgo->SetGpsCalibration(base2Sensor);
    auto arrayX = csvTable->GetRowData()->GetArray("x");
    auto arrayY = csvTable->GetRowData()->GetArray("y");
    auto arrayZ = csvTable->GetRowData()->GetArray("z");

    for (vtkIdType i = 0; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      LidarSlam::ExternalSensors::GpsMeasurement meas;
      meas.Time = arrayTime->GetTuple1(i);
      // Position
      meas.Position = Eigen::Vector3d(arrayX->GetTuple1(i), arrayY->GetTuple1(i), arrayZ->GetTuple1(i));
      // Default covariance
      meas.Covariance = 1e-4 * Eigen::Matrix3d::Identity();
      this->SlamAlgo->AddGpsMeasurement(meas);
    }
    PRINT_INFO("GPS data successfully loaded")
    extSensorFit = true;
  }

  if (!extSensorFit)
    vtkWarningMacro(<< this->GetClassName() << " (" << this << "): No usable data found in the external sensor file");

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::Calibrate()
{
  if (!this->SlamAlgo->PoseHasData() &&
      !this->SlamAlgo->GpsHasData())
    vtkWarningMacro(<< this->GetClassName() << " (" << this << "): No external poses, no GPS -> cannot estimate any calibration");

  if (this->SlamAlgo->PoseHasData())
    this->SlamAlgo->CalibrateWithExtPoses(this->CalibrationWindow, this->LeverArm, true, this->PlanarTrajectory); // true := reset calibration
  else
    this->SlamAlgo->CalibrateWithGps(this->CalibrationWindow, this->LeverArm, true, this->PlanarTrajectory); // true := reset calibration

}

//-----------------------------------------------------------------------------
void vtkSlam::SetTrajectory(const std::string& fileName)
{
  std::string delimiter = " ;,";
  vtkSmartPointer<vtkDelimitedTextReader> reader = Utils::CreateCSVLoader(fileName, delimiter);
  if (!reader)
     return;
  vtkTable* csvTable = reader->GetOutput();

  // Check if time exists and extract it
  if (!Utils::CheckTableFields(csvTable, {"Time"}))
  {
    vtkWarningMacro(<<"No time information in the trajectory file. Load trajectory failed.");
    return;
  }
  auto arrayTime = csvTable->GetRowData()->GetArray("Time");
  vtkIdType numPose = arrayTime->GetNumberOfTuples();
  if (numPose == 0)
  {
    vtkWarningMacro(<<"No valid data in the trajectory file. Load trajectory failed.");
    return;
  }

  // Initialize a pose manager to store the external trajectory
  // Enable Verbose is useful to know whether the new trajectory is loaded correctly.
  // Set DistanceThreshold and AngleThreshold by the same values used in slam for checking keyframes.
  LidarSlam::ExternalSensors::PoseManager trajectoryManager("new trajectory");
  trajectoryManager.SetVerbose(true);
  trajectoryManager.SetDistanceThreshold(std::max(2., 2*this->GetKfDistanceThreshold()));

  // Set default covariance
  float defaultPositionError = 1e-2; // 1cm
  float defaultAngleError = Utils::Deg2Rad(1.f); // 1°
  Eigen::Matrix6d defaultCovariance = Utils::CreateDefaultCovariance(defaultPositionError, defaultAngleError); //1cm, 1°
  // If 2D mode enabled, supply constant covariance for unevaluated variables
  if (this->GetTwoDMode())
  {
    defaultCovariance(2, 2) = std::pow(defaultPositionError, 2);
    defaultCovariance(3, 3) = std::pow(defaultAngleError,    2);
    defaultCovariance(4, 4) = std::pow(defaultAngleError,    2);
  }
  // Process Covariance data
  std::vector<Eigen::Matrix6d> newCovariances(numPose, defaultCovariance);
  bool hasCovariance = true;
  for (int nCov = 0; nCov < 36; ++nCov)
  {
    hasCovariance = hasCovariance && Utils::CheckTableFields(csvTable, {"Covariance:" + std::to_string(nCov)});
    if (!hasCovariance)
      break;
  }
  if (hasCovariance)
  {
    // If covariance exists, set CovarianceRotation true
    trajectoryManager.SetCovarianceRotation(true);
    // Load covariance matrix
    for (int nCov = 0; nCov < 36; ++nCov)
    {
      auto arrayCovariance =  csvTable->GetRowData()->GetArray(("Covariance:"+std::to_string(nCov)).c_str());
      for (vtkIdType poseIdx = 0; poseIdx < numPose; ++poseIdx)
        newCovariances[poseIdx](nCov) =  arrayCovariance->GetTuple1(poseIdx);
    }
  }

  // Process Pose data
  if (Utils::CheckTableFields(csvTable, {"x", "y", "z", "roll", "pitch", "yaw"}))
  {
    auto arrayX     = csvTable->GetRowData()->GetArray("x"    );
    auto arrayY     = csvTable->GetRowData()->GetArray("y"    );
    auto arrayZ     = csvTable->GetRowData()->GetArray("z"    );
    auto arrayRoll  = csvTable->GetRowData()->GetArray("roll" );
    auto arrayPitch = csvTable->GetRowData()->GetArray("pitch");
    auto arrayYaw   = csvTable->GetRowData()->GetArray("yaw"  );

    for (vtkIdType i = 0; i < numPose; ++i)
    {
      LidarSlam::ExternalSensors::PoseMeasurement meas;
      meas.Pose = Utils::XYZRPYtoIsometry(arrayX->GetTuple1(i), arrayY->GetTuple1(i), arrayZ->GetTuple1(i),
                                          arrayRoll->GetTuple1(i), arrayPitch->GetTuple1(i), arrayYaw->GetTuple1(i));
      meas.Time = arrayTime->GetTuple1(i);
      // If covariance data is available, check the validity and add to measurement.
      if(hasCovariance)
      {
        if (!Utils::isCovarianceValid(newCovariances[i]))
          newCovariances[i] = defaultCovariance;
        meas.Covariance = newCovariances[i];
      }
      trajectoryManager.AddMeasurement(meas);
    }
  }
  else if (Utils::CheckTableFields(csvTable, {"Orientation(AxisAngle):0", "Orientation(AxisAngle):1",
                                              "Orientation(AxisAngle):2", "Orientation(AxisAngle):3",
                                              "Points:0", "Points:1", "Points:2"}))
  {
    auto arrayAxisX = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):0");
    auto arrayAxisY = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):1");
    auto arrayAxisZ = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):2");
    auto arrayAngle = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):3");
    auto arrayX     = csvTable->GetRowData()->GetArray("Points:0"                );
    auto arrayY     = csvTable->GetRowData()->GetArray("Points:1"                );
    auto arrayZ     = csvTable->GetRowData()->GetArray("Points:2"                );
    for (vtkIdType i = 0; i < numPose; ++i)
    {
      LidarSlam::ExternalSensors::PoseMeasurement meas;
      meas.Pose = Utils::XYZAngleAxistoIsometry(arrayX->GetTuple1(i), arrayY->GetTuple1(i), arrayZ->GetTuple1(i),
                                                arrayAngle->GetTuple1(i),
                                                arrayAxisX->GetTuple1(i), arrayAxisY->GetTuple1(i), arrayAxisZ->GetTuple1(i));
      meas.Time = arrayTime->GetTuple1(i);
      // If covariance data is available, check the validity and add to measurement.
      if(hasCovariance)
      {
        if (!Utils::isCovarianceValid(newCovariances[i]))
          newCovariances[i] = defaultCovariance;
        meas.Covariance = newCovariances[i];
      }
      trajectoryManager.AddMeasurement(meas);
    }
  }

  // Reload LogStates in Slam with new trajectory
  this->SlamAlgo->ResetStatePoses(trajectoryManager);
  PRINT_INFO("Trajectory successfully loaded.");

  // Update PV trajectory poses that have been modified by the SLAM
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  // Keep old poses that have not been modified
  this->ResetTrajectory(lidarStates.front().Time);
  for (auto const& state: lidarStates)
    this->AddPoseToTrajectory(state);

  // Clear loop detections
  this->ClearLoopDetections();
  PRINT_INFO("Loop indices are cleared!");

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetSensorTimeSynchronization(int mode)
{
  if (mode > 1)
  {
    vtkErrorMacro(<< "Invalid time synchronization mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting SensorTimeSynchronization to " << mode);
  this->SynchronizeOnPacket = (mode == 0);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Slam parameters: " << std::endl;
  vtkIndent paramIndent = indent.GetNextIndent();
  #define PrintParameter(param) os << paramIndent << #param << "\t" << this->SlamAlgo->Get##param() << std::endl;

  PrintParameter(Undistortion)
  PrintParameter(NbThreads)
  PrintParameter(Verbosity)

  for (auto& k : LidarSlam::KeypointTypes)
  {
    if (this->SlamAlgo->KeypointTypeEnabled(k))
      os << LidarSlam::KeypointTypeNames.at(k) << " enabled" << std::endl;
  }

  PrintParameter(EgoMotionICPMaxIter)
  PrintParameter(EgoMotionLMMaxIter)
  PrintParameter(EgoMotionMaxNeighborsDistance)
  PrintParameter(EgoMotionEdgeNbNeighbors)
  PrintParameter(EgoMotionEdgeMinNbNeighbors)
  PrintParameter(EgoMotionEdgeMaxModelError)
  PrintParameter(EgoMotionPlaneNbNeighbors)
  PrintParameter(EgoMotionPlaneMaxModelError)
  PrintParameter(EgoMotionPlanarityThreshold)
  PrintParameter(EgoMotionInitSaturationDistance)
  PrintParameter(EgoMotionFinalSaturationDistance)

  PrintParameter(LocalizationICPMaxIter)
  PrintParameter(LocalizationLMMaxIter)
  PrintParameter(LocalizationMaxNeighborsDistance)
  PrintParameter(LocalizationEdgeNbNeighbors)
  PrintParameter(LocalizationEdgeMinNbNeighbors)
  PrintParameter(LocalizationEdgeMaxModelError)
  PrintParameter(LocalizationPlaneNbNeighbors)
  PrintParameter(LocalizationPlanarityThreshold)
  PrintParameter(LocalizationPlaneMaxModelError)
  PrintParameter(LocalizationBlobNbNeighbors)
  PrintParameter(LocalizationInitSaturationDistance)
  PrintParameter(LocalizationFinalSaturationDistance)

  this->GetKeyPointsExtractor()->PrintSelf(os, indent);
}

//-----------------------------------------------------------------------------
int vtkSlam::FillInputPortInformation(int port, vtkInformation* info)
{
  // Pointcloud data
  if (port == LIDAR_FRAME_INPUT_PORT)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData");
    return 1;
  }
  return 0;
}

//-----------------------------------------------------------------------------
vtkMTimeType vtkSlam::GetMTime()
{
  return std::max(this->Superclass::GetMTime(), this->ParametersModificationTime.GetMTime());
}

// =============================================================================
//   Useful helpers
// =============================================================================

//-----------------------------------------------------------------------------
bool vtkSlam::IdentifyInputArrays(vtkPolyData* poly)
{
  // Check if requested lidar scan arrays exist and set them if they are valid
  if (!this->AutoDetectInputArrays)
  {
    for (int i = 0; i < 3; i++)
    {
      if (!this->GetInputArrayToProcess(i, poly))
      {
        vtkWarningMacro(<< "Failed to get input array to process.");
        return false;
      }
    }
    this->TimeArrayName = this->GetInputArrayToProcess(0, poly)->GetName();
    this->IntensityArrayName = this->GetInputArrayToProcess(1, poly)->GetName();
    this->LaserIdArrayName = this->GetInputArrayToProcess(2, poly)->GetName();
  }
  else
  {
    auto checkAndSetScanArray = [&](const char* name, std::string& member)
    {
      if (poly->GetPointData()->HasArray(name))
        member = name;
    };

    checkAndSetScanArray("time", this->TimeArrayName);
    checkAndSetScanArray("times", this->TimeArrayName);
    checkAndSetScanArray("timestamp", this->TimeArrayName);
    checkAndSetScanArray("adjustedTime", this->TimeArrayName);
    if (this->TimeArrayName == "")
      return false;

    checkAndSetScanArray("reflectivity", this->IntensityArrayName);
    checkAndSetScanArray("Signal Photons", this->IntensityArrayName);
    checkAndSetScanArray("intensity", this->IntensityArrayName);
    if (this->IntensityArrayName == "")
      return false;

    checkAndSetScanArray("laser_id", this->LaserIdArrayName);
    checkAndSetScanArray("ring", this->LaserIdArrayName);
    if (this->LaserIdArrayName == "")
      return false;
  }

  // Estimate the factor to convert times to seconds
  auto arrayTime = poly->GetPointData()->GetArray(this->TimeArrayName.c_str());
  double* range = arrayTime->GetRange();
  double duration = std::abs(range[1] - range[0]);

  // We suppose the duration time is contained between 20ms and 0.9s.
  while (this->TimeToSecondsFactor * duration > 0.9) // Min = ~1 Hz
  {
    this->TimeToSecondsFactor *= 1e-3;
    PRINT_INFO("Time factor estimated to " << this->TimeToSecondsFactor)
  }

  return true;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkSlam::CreateInitTrajectory()
{
  vtkSmartPointer<vtkPolyData> traj = vtkSmartPointer<vtkPolyData>::New();
  auto pts = vtkSmartPointer<vtkPoints>::New();
  traj->SetPoints(pts);
  auto cellArray = vtkSmartPointer<vtkCellArray>::New();
  traj->SetLines(cellArray);
  traj->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Time", 1));
  traj->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Orientation(Quaternion)", 4));
  traj->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Orientation(AxisAngle)", 4));
  traj->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Covariance", 36));
  // Add debug arrays if required
  auto debugInfo = this->SlamAlgo->GetDebugInformation();
  if (this->AdvancedReturnMode)
  {
    for (const auto& it : debugInfo)
      traj->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>(it.first, 1));
  }
  return traj;
}

//-----------------------------------------------------------------------------
void vtkSlam::ResetTrajectory(double endTime)
{
  // By default reset the output SLAM trajectory
  if (endTime < 0)
  {
    this->Trajectory = this->CreateInitTrajectory();
    return;
  }
  // Create a temporary trajectory to save the trajectory before endTime
  vtkSmartPointer<vtkPolyData> trajectoryTmp = this->CreateInitTrajectory();

  auto pointData = this->Trajectory->GetPointData();
  auto arrayTime = pointData->GetArray("Time");
  for (vtkIdType idx = 0; idx < arrayTime->GetNumberOfTuples(); ++idx)
  {
    if (*arrayTime->GetTuple(idx) < endTime)
    {
      double *translation = this->Trajectory->GetPoint(idx);
      trajectoryTmp->GetPoints()->InsertNextPoint(translation);

      for (vtkIdType idxArray = 0; idxArray < pointData->GetNumberOfArrays(); ++idxArray)
      {
        char *fieldName = pointData->GetArray(idxArray)->GetName();
        auto arrayTmp = trajectoryTmp->GetPointData()->GetArray(fieldName);
        if (arrayTmp)
        {
          double *value = pointData->GetArray(idxArray)->GetTuple(idx);
          arrayTmp->InsertNextTuple(value);
        }
      }

      // Add line linking 2 successive points
      vtkIdType nPoints = trajectoryTmp->GetNumberOfPoints();
      if (nPoints >= 2)
      {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, nPoints - 2);
        line->GetPointIds()->SetId(1, nPoints - 1);
        trajectoryTmp->GetLines()->InsertNextCell(line);
      }
    }
    else
      break;
  }

  // Copy temporary trajectory to Trajectory
  this->Trajectory->ShallowCopy(trajectoryTmp);

}

//-----------------------------------------------------------------------------
void vtkSlam::AddPoseToTrajectory(const LidarSlam::LidarState& state)
{
  // Add position
  Eigen::Vector3d translation = state.Isometry.translation();
  this->Trajectory->GetPoints()->InsertNextPoint(translation.x(), translation.y(), translation.z());

  // Add orientation as quaternion
  Eigen::Quaterniond quaternion(state.Isometry.linear());
  double wxyz[] = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  this->Trajectory->GetPointData()->GetArray("Orientation(Quaternion)")->InsertNextTuple(wxyz);

  // Add orientation as axis angle
  Eigen::AngleAxisd angleAxis(state.Isometry.linear());
  Eigen::Vector3d axis = angleAxis.axis();
  double xyza[] = {axis.x(), axis.y(), axis.z(), angleAxis.angle()};
  this->Trajectory->GetPointData()->GetArray("Orientation(AxisAngle)")->InsertNextTuple(xyza);

  // Add pose time and covariance
  this->Trajectory->GetPointData()->GetArray("Time")->InsertNextTuple(&state.Time);
  this->Trajectory->GetPointData()->GetArray("Covariance")->InsertNextTuple(state.Covariance.data());

  // Add line linking 2 successive points
  vtkIdType nPoints = this->Trajectory->GetNumberOfPoints();
  if (nPoints >= 2)
  {
    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
    line->GetPointIds()->SetId(0, nPoints - 2);
    line->GetPointIds()->SetId(1, nPoints - 1);
    this->Trajectory->GetLines()->InsertNextCell(line);
  }

  if (this->AdvancedReturnMode)
  {
    // General SLAM info (number of keypoints used in ICP and optimization, max variance, ...)
    // Arrays added to trajectory output
    auto debugInfo = this->SlamAlgo->GetDebugInformation();
    for (const auto& it : debugInfo)
    {
      auto point = this->Trajectory->GetPointData();
      if (!point)
        continue;
      auto array = point->GetArray(it.first.c_str());
      if (!array)
        continue;
      array->InsertNextTuple1(it.second);
    }
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::AddLastPosesToTrajectory()
{
  // Get current SLAM pose in WORLD coordinates
  std::vector<LidarSlam::LidarState> lastStates = this->SlamAlgo->GetLastStates(this->TrajFrequency);

  for (const auto& state : lastStates)
    this->AddPoseToTrajectory(state);
}

//-----------------------------------------------------------------------------
void vtkSlam::PolyDataToPointCloud(vtkPolyData* poly,
                                   LidarSlam::Slam::PointCloud::Ptr pc)
{
  const vtkIdType nbPoints = poly->GetNumberOfPoints();

  // Get pointers to arrays
  auto arrayTime = poly->GetPointData()->GetArray(this->TimeArrayName.c_str());
  auto arrayLaserId = poly->GetPointData()->GetArray(this->LaserIdArrayName.c_str());
  auto arrayIntensity = poly->GetPointData()->GetArray(this->IntensityArrayName.c_str());

  // Loop over points data
  pc->reserve(nbPoints);
  pc->header.stamp = this->PointTimeRelativeToFrame ? this->FrameTime * 1e6
                     : this->FrameTime * (this->TimeToSecondsFactor * 1e6); // max time in microseconds
  pc->header.frame_id = "mainLidar";
  this->ArePointsValid.resize(nbPoints);
  for (vtkIdType i = 0; i < nbPoints; i++)
  {
    // Get point coordinates
    Eigen::Vector3d pos;
    poly->GetPoint(i, pos.data());
    this->ArePointsValid[i] = false;
    // Check that points coordinates are not null before adding point
    if (!Utils::IsPointValid(pos))
      continue;
    LidarSlam::Slam::Point p;
    p.x = pos[0];
    p.y = pos[1];
    p.z = pos[2];
    p.time = this->PointTimeRelativeToFrame ? arrayTime->GetTuple1(i) * this->TimeToSecondsFactor
              : (arrayTime->GetTuple1(i) - this->FrameTime) * this->TimeToSecondsFactor; // time in seconds
    p.laser_id = arrayLaserId->GetTuple1(i);
    p.intensity = arrayIntensity->GetTuple1(i);
    if (Utils::HasNanField(p))
      continue;
    pc->push_back(p);
    this->ArePointsValid[i] = true;
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::PointCloudToPolyData(LidarSlam::Slam::PointCloud::Ptr pc, vtkPolyData* poly) const
{
  const vtkIdType nbPoints = pc->size();

  // Init and register points
  vtkNew<vtkPoints> pts;
  pts->SetNumberOfPoints(nbPoints);
  poly->SetPoints(pts);
  auto intensityArray = Utils::CreateArray<vtkDoubleArray>(this->IntensityArrayName.c_str(), 1, nbPoints);
  poly->GetPointData()->AddArray(intensityArray);

  // Init and register cells
  vtkNew<vtkIdTypeArray> connectivity;
  connectivity->SetNumberOfValues(nbPoints);
  vtkNew<vtkCellArray> cellArray;
  cellArray->SetData(1 , connectivity);
  poly->SetVerts(cellArray);

  // Fill points and cells values
  for (vtkIdType i = 0; i < nbPoints; ++i)
  {
    // Set point
    const auto& p = pc->points[i];
    pts->SetPoint(i, p.x, p.y, p.z);
    intensityArray->SetTuple1(i, p.intensity);
    // TODO : add other fields (time, laserId)?

    connectivity->SetValue(i, i); //TODO can we iota this thing
  }
}

// =============================================================================
//   Getters / setters
// =============================================================================

bool vtkSlam::areEdgesEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE);
  if (enabled)
    vtkDebugMacro(<< "Edges are enabled");
  else
    vtkDebugMacro(<< "Edges are disabled");
  return enabled;
}

bool vtkSlam::areIntensityEdgesEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::INTENSITY_EDGE);
  if (enabled)
    vtkDebugMacro(<< "Intensity edges are enabled");
  else
    vtkDebugMacro(<< "Intensity edges are disabled");
  return enabled;
}

bool vtkSlam::arePlanesEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE);
  if (enabled)
    vtkDebugMacro(<< "Planes are enabled");
  else
    vtkDebugMacro(<< "Planes are disabled");
  return enabled;
}

bool vtkSlam::areBlobsEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB);
  if (enabled)
    vtkDebugMacro(<< "Blobs are enabled");
  else
    vtkDebugMacro(<< "Blobs are disabled");
  return enabled;
}

//-----------------------------------------------------------------------------
void vtkSlam::EnableEdges(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling edges");
  else
  {
    vtkDebugMacro(<< "Disabling edges");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::EDGE, enabled);
  // Reset trajectory to add/remove confidence estimators related to edge keypoints
  if (this->AdvancedReturnMode)
    this->ResetTrajectory(this->FrameTime);
}

void vtkSlam::EnableIntensityEdges(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling intensity edges");
  else
  {
    vtkDebugMacro(<< "Disabling intensity edges");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::INTENSITY_EDGE, enabled);
  // Reset trajectory to add/remove confidence estimators related to intensity edge keypoints
  if (this->AdvancedReturnMode)
    this->ResetTrajectory(this->FrameTime);
}

void vtkSlam::EnablePlanes(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling planes");
  else
  {
    vtkDebugMacro(<< "Disabling planes");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::PLANE, enabled);
  // Reset trajectory to add/remove confidence estimators related to plane keypoints
  if (this->AdvancedReturnMode)
    this->ResetTrajectory(this->FrameTime);
}

void vtkSlam::EnableBlobs(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling blobs");
  else
  {
    vtkDebugMacro(<< "Disabling blobs");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::BLOB, enabled);
  // Reset trajectory to add/remove confidence estimators related to blob keypoints
  if (this->AdvancedReturnMode)
    this->ResetTrajectory(this->FrameTime);
}

//-----------------------------------------------------------------------------
void vtkSlam::SetFailureDetectionEnabled(bool faildetect)
{
  // If failure detection is being activated
  if (faildetect)
  {
    // Enable overlap computation
    this->SlamAlgo->SetOverlapSamplingRatio(this->OverlapSamplingRatio);
    // Enable motion metrics and averages/derivatives computation
    this->SlamAlgo->SetConfidenceWindow(this->ConfidenceWindow);
  }
  // If failure detection is being disabled
  else
  {
    // End recovery mode
    if (this->SlamAlgo->IsRecovery())
    {
      this->SlamAlgo->EndRecovery();
      vtkWarningMacro(<< "Getting out of recovery mode");
    }

    if (!this->AdvancedReturnMode)
    {
      // Disable overlap computation
      this->SlamAlgo->SetOverlapSamplingRatio(0.);
      // Disable motion metrics and averages/derivatives computation
      this->SlamAlgo->SetConfidenceWindow(0);
    }
  }
  this->SlamAlgo->SetFailureDetectionEnabled(faildetect);
}

//-----------------------------------------------------------------------------
void vtkSlam::SetAdvancedReturnMode(bool _arg)
{
  vtkDebugMacro(<< "Setting AdvancedReturnMode to " << _arg);
  if (this->AdvancedReturnMode != _arg)
  {
    auto debugInfo = this->SlamAlgo->GetDebugInformation();

    // If AdvancedReturnMode is being activated
    if (_arg)
    {
      // Add new optional arrays to trajectory, and init past values to 0.
      for (const auto& it : debugInfo)
      {
        auto array = Utils::CreateArray<vtkDoubleArray>(it.first, 1, this->Trajectory->GetNumberOfPoints());
        for (vtkIdType i = 0; i < this->Trajectory->GetNumberOfPoints(); i++)
          array->SetTuple1(i, 0.);
        this->Trajectory->GetPointData()->AddArray(array);
      }
      // Enable overlap computation
      this->SlamAlgo->SetOverlapSamplingRatio(this->OverlapSamplingRatio);
      // Enable motion metrics and averages/derivatives computation
      this->SlamAlgo->SetConfidenceWindow(this->ConfidenceWindow);
    }

    // If AdvancedReturnMode is being disabled
    else
    {
      // Delete optional arrays
      for (const auto& it : debugInfo)
        this->Trajectory->GetPointData()->RemoveArray(it.first.c_str());
      if (!this->SlamAlgo->GetFailureDetectionEnabled())
      {
        // Disable overlap computation
        this->SlamAlgo->SetOverlapSamplingRatio(0.);
        // Disable motion metrics and averages/derivatives computation
        this->SlamAlgo->SetConfidenceWindow(0);
      }
    }

    this->AdvancedReturnMode = _arg;
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkSlam::GetOutputKeypointsMaps()
{
  int outputMaps = static_cast<int>(this->OutputKeypointsMaps);
  vtkDebugMacro(<< "Returning output keypoints maps mode : " << outputMaps);
  return outputMaps;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetOutputKeypointsMaps(int mode)
{
  OutputKeypointsMapsMode outputMaps = static_cast<OutputKeypointsMapsMode>(mode);
  if (outputMaps != OutputKeypointsMapsMode::NONE      &&
      outputMaps != OutputKeypointsMapsMode::FULL_MAPS &&
      outputMaps != OutputKeypointsMapsMode::SUB_MAPS)
  {
    vtkErrorMacro(<< "Invalid output keypoints maps mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting output keypoints maps mode to " << mode);
  if (this->OutputKeypointsMaps != outputMaps)
  {
    this->OutputKeypointsMaps = outputMaps;
    this->ParametersModificationTime.Modified();
  }
  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
int vtkSlam::GetEgoMotion()
{
  int egoMotion = static_cast<int>(this->SlamAlgo->GetEgoMotion());
  vtkDebugMacro(<< "Returning Ego-Motion of " << egoMotion);
  return egoMotion;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetEgoMotion(int mode)
{
  if (this->SlamAlgo->IsRecovery())
  {
    vtkErrorMacro(<< "Cannot change ego motion in recovery mode! This param might be falsely set afterwards");
    return;
  }
  LidarSlam::EgoMotionMode egoMotion = static_cast<LidarSlam::EgoMotionMode>(mode);
  if (egoMotion != LidarSlam::EgoMotionMode::NONE                 &&
      egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION &&
      egoMotion != LidarSlam::EgoMotionMode::REGISTRATION         &&
      egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION_AND_REGISTRATION &&
      egoMotion != LidarSlam::EgoMotionMode::EXTERNAL &&
      egoMotion != LidarSlam::EgoMotionMode::EXTERNAL_OR_MOTION_EXTRAPOLATION)
  {
    vtkErrorMacro(<< "Invalid ego-motion mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting Ego-Motion to " << mode);
  if (this->SlamAlgo->GetEgoMotion() != egoMotion)
  {
    this->SlamAlgo->SetEgoMotion(egoMotion);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkSlam::GetUndistortion()
{
  int undistortion = static_cast<int>(this->SlamAlgo->GetUndistortion());
  vtkDebugMacro(<< "Returning Undistortion of " << undistortion);
  return undistortion;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetUndistortion(int mode)
{
  if (this->SlamAlgo->IsRecovery())
  {
    vtkErrorMacro(<< "Cannot change undistortion in recovery mode! This param might be falsely set afterwards");
    return;
  }
  LidarSlam::UndistortionMode undistortion = static_cast<LidarSlam::UndistortionMode>(mode);
  if (undistortion != LidarSlam::UndistortionMode::NONE &&
      undistortion != LidarSlam::UndistortionMode::ONCE &&
      undistortion != LidarSlam::UndistortionMode::REFINED &&
      undistortion != LidarSlam::UndistortionMode::EXTERNAL)
  {
    vtkErrorMacro(<< "Invalid undistortion mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting Undistortion to " << mode);
  if (this->SlamAlgo->GetUndistortion() != undistortion)
  {
    this->SlamAlgo->SetUndistortion(undistortion);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkSlam::GetInterpolation()
{
  int interpoModel = static_cast<int>(this->SlamAlgo->GetInterpolation());
  vtkDebugMacro(<< this->GetClassName() << "(" << this << "): returning Interpolation Model of " << interpoModel);
  return interpoModel;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInterpolation(int model)
{
  LidarSlam::Interpolation::Model interpoModel = static_cast<LidarSlam::Interpolation::Model>(model);
  if (interpoModel != LidarSlam::Interpolation::Model::LINEAR &&
      interpoModel != LidarSlam::Interpolation::Model::QUADRATIC &&
      interpoModel != LidarSlam::Interpolation::Model::CUBIC)
  {
    vtkErrorMacro("Invalid Interpolation Model (" << model << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< this->GetClassName() << "(" << this << "): setting Interpolation Model to " << model);
  if (this->SlamAlgo->GetInterpolation() != interpoModel)
  {
    this->SlamAlgo->SetInterpolation(interpoModel);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialPose(std::string filename)
{
  if (filename.empty())
    return;

  Eigen::Isometry3d newPose;
  this->GetCalibrationMatrix(filename, newPose);
  vtkDebugMacro(<< "Setting Initial pose to \n" << newPose.matrix() << "\n");
  // Move odom so the initial pose corresponds to the newPose
  this->SlamAlgo->SetInitialPose(newPose);

  // Update PV trajectory poses that have been optimized by the SLAM
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  if (!lidarStates.empty())
  {
    // Only logged trajectory can be displayed after changing odom
    this->ResetTrajectory();
    for (auto const& state: lidarStates)
      this->AddPoseToTrajectory(state);
  }

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetCurrentPose(std::string filename)
{
  if (filename.empty())
    return;

  Eigen::Isometry3d newPose;
  this->GetCalibrationMatrix(filename, newPose);
  vtkDebugMacro(<< "Setting current pose to \n" << newPose.matrix() << "\n");
  // Move odom so the current pose corresponds to the newPose
  this->SlamAlgo->SetCurrentPose(newPose);

  // Update PV trajectory poses that have been optimized by the SLAM
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  if (!lidarStates.empty())
  {
    // Only logged trajectory can be displayed after changing odom
    this->ResetTrajectory();
    for (auto const& state: lidarStates)
      this->AddPoseToTrajectory(state);
  }

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetBaseToLidarTransform(std::string filename)
{
  Eigen::Isometry3d baseToLidar;
  if (filename.empty())
    baseToLidar = Eigen::Isometry3d::Identity();
  else
    this->GetCalibrationMatrix(filename, baseToLidar);
  vtkDebugMacro(<< "Setting BaseToLidarTransform to \n" << baseToLidar.matrix() << "\n");
  this->SlamAlgo->SetBaseToLidarOffset(baseToLidar);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetKeyPointsExtractor(vtkSpinningSensorKeypointExtractor* _arg)
{
  vtkSetObjectBodyMacro(KeyPointsExtractor, vtkSpinningSensorKeypointExtractor, _arg);
  this->SlamAlgo->SetKeyPointsExtractor(this->KeyPointsExtractor->GetExtractor());
  this->SlamAlgo->GetKeyPointsExtractor()->SetNbThreads(this->SlamAlgo->GetNbThreads());
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
unsigned int vtkSlam::GetMapUpdate()
{
  unsigned int mapUpdate = static_cast<unsigned int>(this->SlamAlgo->GetMapUpdate());
  vtkDebugMacro(<< "Returning mapping mode of " << mapUpdate);
  return mapUpdate;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetMapUpdate(unsigned int mode)
{
  if (this->SlamAlgo->IsRecovery())
  {
    vtkErrorMacro(<< "Cannot change map update in recovery mode! This param might be falsely displayed afterwards");
    return;
  }
  LidarSlam::MappingMode mapUpdate = static_cast<LidarSlam::MappingMode>(mode);
  if (mapUpdate != LidarSlam::MappingMode::NONE         &&
      mapUpdate != LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP &&
      mapUpdate != LidarSlam::MappingMode::UPDATE)
  {
    vtkErrorMacro(<< "Invalid mapping mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting mapping mode to " << mode);
  if (this->SlamAlgo->GetMapUpdate() != mapUpdate)
  {
    this->SlamAlgo->SetMapUpdate(mapUpdate);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
unsigned int vtkSlam::GetSubmapMode()
{
  unsigned int submapMode = static_cast<unsigned int>(this->SlamAlgo->GetSubmapMode());
  vtkDebugMacro(<< "Returning mapping mode of " << submapMode);
  return submapMode;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetSubmapMode(unsigned int mode)
{
  if (this->SlamAlgo->IsRecovery())
  {
    vtkErrorMacro(<< "Cannot change submap mode in recovery mode! This param might be falsely displayed afterwards");
    return;
  }
  LidarSlam::PreSearchMode submapMode = static_cast<LidarSlam::PreSearchMode>(mode);
  if (submapMode != LidarSlam::PreSearchMode::BOUNDING_BOX &&
      submapMode != LidarSlam::PreSearchMode::PROFILE)
  {
    vtkErrorMacro(<< "Invalid submap mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting submap mode to " << mode);
  if (this->SlamAlgo->GetSubmapMode() != submapMode)
  {
    this->SlamAlgo->SetSubmapMode(submapMode);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridLeafSize(LidarSlam::Keypoint k, double s)
{
  // The setting of this parameter is only possible if the relative map exists
  // the enabling step will create the map on which the sampling mode parameter can be set
  // The user can call this setter function only if the keypoint type has been enabled (see xml)
  // So, this function must not be called before clicking on enabled
  // However, clicking on Apply call all the setters with default values.
  // Therefore, the on/off state is checked but no warning can be raised
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
    return;

  vtkDebugMacro(<< "Setting VoxelGridLeafSize to " << s);
  this->SlamAlgo->SetVoxelGridLeafSize(k, s);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
double vtkSlam::GetVoxelGridLeafSize(LidarSlam::Keypoint k) const
{
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
  {
    vtkErrorMacro(<< "Cannot get leaf size, " << LidarSlam::KeypointTypeNames.at(k) << " keypoints are not enabled.");
    return -1.;
  }
  double leafSize = this->SlamAlgo->GetVoxelGridLeafSize(k);
  vtkDebugMacro(<< "Returning sampling mode : " << leafSize);
  return leafSize;
}

//-----------------------------------------------------------------------------
int vtkSlam::GetVoxelGridSamplingMode(LidarSlam::Keypoint k) const
{
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
  {
    vtkErrorMacro(<< "Cannot get sampling mode, " << LidarSlam::KeypointTypeNames.at(k) << " keypoints are not enabled.");
    return -1;
  }
  LidarSlam::SamplingMode sampling = this->SlamAlgo->GetVoxelGridSamplingMode(k);
  int sm = static_cast<int>(sampling);
  vtkDebugMacro(<< "Returning sampling mode : " << sm);
  return sm;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridSamplingMode(LidarSlam::Keypoint k, int mode)
{
  // The setting of this parameter is only possible if the relative map exists
  // the enabling step will create the map on which the sampling mode parameter can be set
  // The user can call this setter function only if the keypoint type has been enabled (see xml)
  // So, this function must not be called before clicking on enabled
  // However, clicking on Apply call all the setters with default values.
  // Therefore, the on/off state is checked but no warning can be raised
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
    return;

  LidarSlam::SamplingMode sampling = static_cast<LidarSlam::SamplingMode>(mode);
  if (sampling != LidarSlam::SamplingMode::FIRST         &&
      sampling != LidarSlam::SamplingMode::LAST          &&
      sampling != LidarSlam::SamplingMode::MAX_INTENSITY &&
      sampling != LidarSlam::SamplingMode::CENTER_POINT  &&
      sampling != LidarSlam::SamplingMode::CENTROID)
  {
    vtkErrorMacro(<< "Invalid sampling mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting sampling mode to " << mode);
  if (this->SlamAlgo->GetVoxelGridSamplingMode(k) != sampling)
  {
    this->SlamAlgo->SetVoxelGridSamplingMode(k, sampling);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetOverlapSamplingRatio(double ratio)
{
  // Change parameter value if it is modified
  vtkDebugMacro(<< "Setting OverlapSamplingRatio to " << ratio);
  if (ratio < 0 || ratio > 1)
  {
    vtkWarningMacro(<< "Overlap sampling ratio should be contained between 0 and 1"
                    << "Input value is : " << ratio
                    << "It is set to default value : 0.25");
    ratio = 0.25;
  }
  if (this->OverlapSamplingRatio != ratio)
  {
    this->OverlapSamplingRatio = ratio;
    // Forward this parameter change to SLAM if it is to be used in the interface
    if (this->AdvancedReturnMode || this->SlamAlgo->GetFailureDetectionEnabled())
      this->SlamAlgo->SetOverlapSamplingRatio(this->OverlapSamplingRatio);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetConfidenceWindow(unsigned int window)
{
  // Change parameter value if it is modified
  vtkDebugMacro(<< "Setting ConfidenceWindow to " << window);
  if (window == 1)
  {
    vtkWarningMacro(<< "Some confidence metrics will not be computed, "
                    << "please increase Confidence window value if you want to use it");
    window = 0;
  }
  if (this->ConfidenceWindow != window)
  {
    this->ConfidenceWindow = window;
    // Forward this parameter change to SLAM if it is to be used in the interface
    if (this->AdvancedReturnMode || this->SlamAlgo->GetFailureDetectionEnabled())
      this->SlamAlgo->SetConfidenceWindow(this->ConfidenceWindow);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetAccelerationLimits(float linearAcc, float angularAcc)
{
  vtkDebugMacro(<< "Setting AccelerationLimits to " << linearAcc << " " << angularAcc);
  Eigen::Array2f accLim = {linearAcc, angularAcc};
  this->SlamAlgo->SetAccelerationLimits(accLim);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVelocityLimits(float linearVel, float angularVel)
{
  vtkDebugMacro(<< "Setting VelocityLimits to " << linearVel << " " << angularVel);
  Eigen::Array2f velLim = {linearVel, angularVel};
  this->SlamAlgo->SetVelocityLimits(velLim);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetPoseLimits(float position, float orientation)
{
  vtkDebugMacro(<< "Setting PoseLimits to " << position << " " << orientation);
  Eigen::Array2f posLim = {position, orientation};
  this->SlamAlgo->SetPoseLimits(posLim);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetLoggingTimeout(double loggingTimeout)
{
  // Change parameter value if it is modified
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting LoggingTimeout to " << loggingTimeout);
  if (this->SlamAlgo->GetLoggingTimeout() != loggingTimeout)
  {
    // Forward this parameter change to SLAM
    this->SlamAlgo->SetLoggingTimeout(loggingTimeout);
    this->ParametersModificationTime.Modified();
  }

  // If UsePoseGraph is enabled, check LoggingTimeout and return a warning if LoggingTimeout is 0
  if (this->UsePoseGraph && loggingTimeout <= 1e-6)
    vtkWarningMacro(<< "Pose graph is required but the logging timeout is null : "
                       "no pose can be used to build the graph, please increase the logging timeout.");
}

//-----------------------------------------------------------------------------
void vtkSlam::SetUsePoseGraph(bool usePoseGraph)
{
  if (this->UsePoseGraph != usePoseGraph)
  {
    this->UsePoseGraph = usePoseGraph;
    this->ParametersModificationTime.Modified();

    // If UsePoseGraph is enabled, check LoggingTimeout and return a warning if LoggingTimeout is 0
    if (this->UsePoseGraph && this->GetLoggingTimeout() <= 1e-6)
      vtkWarningMacro(<< "Pose graph is required but the logging timeout is null : "
                         "no pose can be used to build the graph, please increase the logging timeout.");
  }
}

//-----------------------------------------------------------------------------
double* vtkSlam::GetLoopClosurePosition()
{
  Eigen::Vector3d revistedPosition = this->SlamAlgo->GetStatePosition(this->LastLoopInfo.RevisitedIdx);
  this->LastLoopClosurePosition[0] = revistedPosition.x();
  this->LastLoopClosurePosition[1] = revistedPosition.y();
  this->LastLoopClosurePosition[2] = revistedPosition.z();
  return this->LastLoopClosurePosition;
}

//-----------------------------------------------------------------------------
int vtkSlam::GetLoopDetector()
{
  int loopClosureDetector = static_cast<int>(this->SlamAlgo->GetLoopDetector());
  vtkDebugMacro(<< "Returning loop closure detection of " << loopClosureDetector);
  return loopClosureDetector;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetLoopDetector(int detector)
{
  LidarSlam::LoopClosureDetector loopClosureDetector = static_cast<LidarSlam::LoopClosureDetector>(detector);
  if (loopClosureDetector != LidarSlam::LoopClosureDetector::EXTERNAL   &&
      loopClosureDetector != LidarSlam::LoopClosureDetector::TEASERPP)
  {
    vtkErrorMacro(<< "Invalid loop closure detector (" << detector << "), ignoring setting.");
    return;
  }
  #ifndef USE_TEASERPP
  if (loopClosureDetector == LidarSlam::LoopClosureDetector::TEASERPP)
  {
    vtkErrorMacro(<< "Automatic loop closure detection requires TEASER++, but it was not found.");
    return;
  }
  #endif

  vtkDebugMacro(<< "Setting loop closure detector to " << static_cast<int>(loopClosureDetector));
  if (this->SlamAlgo->GetLoopDetector() != loopClosureDetector)
  {
    this->SlamAlgo->SetLoopDetector(loopClosureDetector);
    this->ParametersModificationTime.Modified();
  }

  // If teaser detector is enabled, the detection is performed on current frame.
  // As there is no mid submap AFTER current frame, we force the submaps to be built upon previous frames.
  if (loopClosureDetector == LidarSlam::LoopClosureDetector::TEASERPP)
  {
    this->SlamAlgo->SetLoopQueryMapEndRange(0);
    this->SlamAlgo->SetLoopRevisitedMapEndRange(0);
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::LoadLoopDetectionIndices(const std::string& fileName)
{
  if (static_cast<LidarSlam::LoopClosureDetector>(this->GetLoopDetector()) != LidarSlam::LoopClosureDetector::EXTERNAL)
  {
    vtkWarningMacro(<< "Loading loop indices from external source is disabled!");
    return;
  }

  // Reset loop indices before loading new file
  this->ClearLoopDetections();

  std::string delimiter = " ;,";
  vtkSmartPointer<vtkDelimitedTextReader> reader = Utils::CreateCSVLoader(fileName, delimiter);
  if (!reader)
     return;
  vtkTable* csvTable = reader->GetOutput();

  // Check if loop closure information exists
  if (!Utils::CheckTableFields(csvTable, {"queryIdx", "revisitedIdx"}))
  {
    vtkWarningMacro(<<"No loop closure information in the file. Load loop closure indices failed.");
    return;
  }

  auto arrayQueryIdx     = csvTable->GetRowData()->GetArray("queryIdx"    );
  auto arrayRevisitedIdx = csvTable->GetRowData()->GetArray("revisitedIdx");
  vtkIdType numLoops     = arrayQueryIdx->GetNumberOfTuples();
  if (numLoops == 0)
  {
    vtkWarningMacro(<<"No valid data in the loop closure indices file. Load loop closure indices failed.");
    return;
  }

  // Process query frame indices and revisited frame indices
  for (vtkIdType i = 0; i < numLoops; ++i)
  {
    LidarSlam::LoopClosure::LoopInfo loop(arrayQueryIdx->GetTuple1(i), arrayRevisitedIdx->GetTuple1(i), -1);
    this->SlamAlgo->AddLoopClosureIndices(loop);
  }

  PRINT_INFO("Loop closure indices are loaded successfully from external source!");

  // Refresh view
  this->ParametersModificationTime.Modified();
}
