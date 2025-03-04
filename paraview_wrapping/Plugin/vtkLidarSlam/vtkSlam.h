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

#ifndef VTK_SLAM_H
#define VTK_SLAM_H

// VTK
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

// LOCAL
#include <LidarSlam/Slam.h>

//! Which keypoints' maps to output
enum OutputKeypointsMapsMode
{
  //! No maps output
  NONE = 0,
  //! Output the whole keypoints' maps
  FULL_MAPS = 1,
  //! Output the target sub maps used for the current frame registration
  SUB_MAPS = 2
};

// This custom macro is needed to make the SlamManager time agnostic.
// The SlamManager needs to know when RequestData is called, if it's due
// to a new timestep being requested or due to SLAM parameters being changed.
// By keeping track of the last time the parameters have been modified there is
// no ambiguity anymore. This mecanimsm is similar to the one used by the
// paraview filter PlotDataOverTime.
#define vtkCustomSetMacro(name, type)                                                            \
virtual void Set##name(type _arg)                                                                \
{                                                                                                \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting " #name " to " << _arg);  \
  if (this->SlamAlgo->Get##name() != _arg)                                                       \
  {                                                                                              \
    this->SlamAlgo->Set##name(_arg);                                                             \
    this->ParametersModificationTime.Modified();                                                 \
  }                                                                                              \
}
#define vtkCustomSetMacroNoCheck(name, type)                                                     \
virtual void Set##name(type _arg)                                                                \
{                                                                                                \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting " #name " to " << _arg);  \
  this->SlamAlgo->Set##name(_arg);                                                               \
  this->ParametersModificationTime.Modified();                                                   \
}

#define vtkCustomGetMacro(name, type)                                                            \
virtual type Get##name()                                                                         \
{                                                                                                \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): returning " << #name " of "       \
                << this->SlamAlgo->Get##name());                                                 \
  return this->SlamAlgo->Get##name();                                                            \
}


#define vtkCustomSetMacroExternalSensor(sensorName, name, type)                                  \
virtual void Set##name(type _arg)                                                                \
{                                                                                                \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting " #name " to " << _arg);  \
  if (this->Get##name() != _arg)                                                                 \
  {                                                                                              \
    if (this->SlamAlgo->sensorName##HasData())                                                   \
    {                                                                                            \
      this->SlamAlgo->Set##name(_arg);                                                           \
      this->ParametersModificationTime.Modified();                                               \
    }                                                                                            \
  }                                                                                              \
}

#define vtkCustomGetMacroExternalSensor(sensorName, name, type)                                  \
virtual type Get##name()                                                                         \
{                                                                                                \
  if (this->SlamAlgo->sensorName##HasData())                                                     \
  {                                                                                              \
    vtkDebugMacro(<< this->GetClassName() << " (" << this << "): returning " << #name " of "     \
                  << this->SlamAlgo->Get##name());                                               \
    return this->SlamAlgo->Get##name();                                                          \
  }                                                                                              \
  type def;                                                                                      \
  return def;                                                                                    \
}


class vtkSpinningSensorKeypointExtractor;
class vtkTable;

class vtkSlam : public vtkPolyDataAlgorithm
{
public:
  static vtkSlam* New();
  vtkTypeMacro(vtkSlam, vtkPolyDataAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent) override;

  virtual vtkMTimeType GetMTime() override;

  // ---------------------------------------------------------------------------
  //   General stuff and flags
  // ---------------------------------------------------------------------------

  void Reset();
  // Clear the maps and the logged trajectory
  // but keep all other values, notably ego-motion and external sensor info
  void ClearMapsAndLog();

  // Initialization
  void SetInitialSlam();
  vtkSetMacro(InitMapPrefix, std::string)
  void SetInitialPoseTranslation(double x, double y, double z);
  void SetInitialPoseRotation(double roll, double pitch, double yaw);

  // Getters / Setters
  vtkGetMacro(PointTimeRelativeToFrame, bool)
  vtkSetMacro(PointTimeRelativeToFrame, bool)

  vtkGetMacro(AutoDetectInputArrays, bool)
  vtkSetMacro(AutoDetectInputArrays, bool)

  vtkGetMacro(AdvancedReturnMode, bool)
  virtual void SetAdvancedReturnMode(bool _arg);

  vtkGetMacro(TimeArrayName, std::string)
  vtkSetMacro(TimeArrayName, std::string)

  vtkGetMacro(TimeToSecondsFactor, double)
  vtkSetMacro(TimeToSecondsFactor, double)

  vtkGetMacro(LastFrameTime, double)
  vtkSetMacro(LastFrameTime, double)

  vtkGetMacro(FrameTime, double)
  vtkSetMacro(FrameTime, double)

  vtkGetMacro(SynchronizeOnPacket, bool)
  vtkSetMacro(SynchronizeOnPacket, bool)

  vtkGetMacro(OutputCurrentKeypoints, bool)
  vtkSetMacro(OutputCurrentKeypoints, bool)

  virtual int GetOutputKeypointsMaps();
  virtual void SetOutputKeypointsMaps(int mode);

  vtkGetMacro(MapsUpdateStep, unsigned int)
  vtkSetMacro(MapsUpdateStep, unsigned int)

  vtkGetMacro(OutputKeypointsInWorldCoordinates, bool)
  vtkSetMacro(OutputKeypointsInWorldCoordinates, bool)

  void EnableEdges(bool enable);
  void EnableIntensityEdges(bool enable);
  void EnablePlanes(bool enable);
  void EnableBlobs(bool enable);

  bool areEdgesEnabled();
  bool areIntensityEdgesEnabled();
  bool arePlanesEnabled();
  bool areBlobsEnabled();

  vtkCustomGetMacro(Verbosity, int)
  vtkCustomSetMacro(Verbosity, int)

  vtkCustomGetMacro(NbThreads, int)
  vtkCustomSetMacro(NbThreads, int)

  virtual int GetEgoMotion();
  virtual void SetEgoMotion(int mode);

  virtual int GetUndistortion();
  virtual void SetUndistortion(int mode);

  // Load trajectory from a file and recompute maps
  void SetTrajectory(const std::string& fileName);

  virtual int GetInterpolation();
  virtual void SetInterpolation(int model);

  // Set measurements to Slam algo
  virtual void SetSensorData(const std::string& fileName);
  virtual void Calibrate();

  vtkGetMacro(TrajFrequency, double)
  vtkSetMacro(TrajFrequency, double)

  vtkGetMacro(PlanarTrajectory, bool)
  vtkSetMacro(PlanarTrajectory, bool)

  vtkGetMacro(LeverArm, double)
  vtkSetMacro(LeverArm, double)

  // ---------------------------------------------------------------------------
  //   Graph parameters
  // ---------------------------------------------------------------------------

  // Check LoggingTimeout is set correctly to ensure the use of pose graph optimization
  void SetUsePoseGraph(bool usePoseGraph);

  // Use the optimized poses from the IMU/SLAM graph to
  // update the trajectory and rebuild the maps (needs GTSAM not G2O)
  void OptimizeGraphWithIMU();

  // Optimize the graph with available information (needs G2O not GTSAM)
  void OptimizeGraph();

  // Check if there is a loop closure implying current frame
  void DetectLoop();

  // Add the detected loop indices into slam
  void AddLoopDetection();

  // Reset the LoopDetections vector
  void ClearLoopDetections();

  vtkCustomGetMacro(G2oFileName, std::string)
  vtkCustomSetMacro(G2oFileName, std::string)

  vtkCustomGetMacro(CovarianceScale, float)
  vtkCustomSetMacro(CovarianceScale, float)

  vtkCustomGetMacro(NbGraphIterations, int)
  vtkCustomSetMacro(NbGraphIterations, int)

  void EnablePGOConstraintLoopClosure(bool enable);
  void EnablePGOConstraintLandmark(bool enable);
  void EnablePGOConstraintGPS(bool enable);
  void EnablePGOConstraintExtPose(bool enable);

  bool GetPGOConstraintLoopClosure();
  bool GetPGOConstraintLandmark();
  bool GetPGOConstraintGPS();
  bool GetPGOConstraintExtPose();

  // ---------------------------------------------------------------------------
  //   Loop closure parameters
  // ---------------------------------------------------------------------------

  vtkGetMacro(LoopDetected, bool)
  vtkSetMacro(LoopDetected, bool)

  // Return revisited frame position of detected loop closure
  double* GetLoopClosurePosition() VTK_SIZEHINT(3);

  virtual int  GetLoopDetector();
  virtual void SetLoopDetector(int detector);

  void LoadLoopDetectionIndices(const std::string& fileName);

  vtkCustomGetMacro(LoopQueryMapStartRange, double)
  vtkCustomSetMacro(LoopQueryMapStartRange, double)

  vtkCustomGetMacro(LoopQueryMapEndRange, double)
  vtkCustomSetMacro(LoopQueryMapEndRange, double)

  vtkCustomGetMacro(LoopRevisitedMapStartRange, double)
  vtkCustomSetMacro(LoopRevisitedMapStartRange, double)

  vtkCustomGetMacro(LoopRevisitedMapEndRange, double)
  vtkCustomSetMacro(LoopRevisitedMapEndRange, double)

  // Get/Set automatic detection parameters
  vtkCustomGetMacro(LoopGapLength, double)
  vtkCustomSetMacro(LoopGapLength, double)

  vtkCustomGetMacro(LoopSampleStep, double)
  vtkCustomSetMacro(LoopSampleStep, double)

  // Get/Set Loop closure registration parameters
  vtkCustomGetMacro(LoopEnableOffset, bool)
  vtkCustomSetMacro(LoopEnableOffset, bool)

  vtkCustomGetMacro(LoopICPWithSubmap, bool)
  vtkCustomSetMacro(LoopICPWithSubmap, bool)

  vtkCustomGetMacro(LoopLMMaxIter, unsigned int)
  vtkCustomSetMacro(LoopLMMaxIter, unsigned int)

  vtkCustomGetMacro(LoopICPMaxIter, unsigned int)
  vtkCustomSetMacro(LoopICPMaxIter, unsigned int)

  vtkCustomGetMacro(LoopMaxNeighborsDistance, double)
  vtkCustomSetMacro(LoopMaxNeighborsDistance, double)

  vtkCustomGetMacro(LoopEdgeNbNeighbors, unsigned int)
  vtkCustomSetMacro(LoopEdgeNbNeighbors, unsigned int)

  vtkCustomGetMacro(LoopEdgeMinNbNeighbors, unsigned int)
  vtkCustomSetMacro(LoopEdgeMinNbNeighbors, unsigned int)

  vtkCustomGetMacro(LoopEdgeMaxModelError, double)
  vtkCustomSetMacro(LoopEdgeMaxModelError, double)

  vtkCustomGetMacro(LoopPlaneNbNeighbors, unsigned int)
  vtkCustomSetMacro(LoopPlaneNbNeighbors, unsigned int)

  vtkCustomGetMacro(LoopPlanarityThreshold, double)
  vtkCustomSetMacro(LoopPlanarityThreshold, double)

  vtkCustomGetMacro(LoopPlaneMaxModelError, double)
  vtkCustomSetMacro(LoopPlaneMaxModelError, double)

  vtkCustomGetMacro(LoopBlobNbNeighbors, unsigned int)
  vtkCustomSetMacro(LoopBlobNbNeighbors, unsigned int)

  vtkCustomGetMacro(LoopInitSaturationDistance, double)
  vtkCustomSetMacro(LoopInitSaturationDistance, double)

  vtkCustomGetMacro(LoopFinalSaturationDistance, double)
  vtkCustomSetMacro(LoopFinalSaturationDistance, double)

  // ---------------------------------------------------------------------------
  //   Transform tree
  // ---------------------------------------------------------------------------

  // Set calibration base to lidar
  virtual void SetBaseToLidarTransform(std::string filename);

  // Move odom so that the initial pose corresponds to the input pose
  virtual void SetInitialPose(std::string filename);

  // Move odom so that the current pose corresponds to the input pose
  virtual void SetCurrentPose(std::string filename);

  // ---------------------------------------------------------------------------
  //   Optimization parameters
  // ---------------------------------------------------------------------------

  vtkCustomGetMacro(TwoDMode, bool)
  vtkCustomSetMacro(TwoDMode, bool)

  // Get/Set EgoMotion
  vtkCustomGetMacro(EgoMotionLMMaxIter, unsigned int)
  vtkCustomSetMacro(EgoMotionLMMaxIter, unsigned int)

  vtkCustomGetMacro(EgoMotionICPMaxIter, unsigned int)
  vtkCustomSetMacro(EgoMotionICPMaxIter, unsigned int)

  vtkCustomGetMacro(EgoMotionMaxNeighborsDistance, double)
  vtkCustomSetMacro(EgoMotionMaxNeighborsDistance, double)

  vtkCustomGetMacro(EgoMotionEdgeNbNeighbors, unsigned int)
  vtkCustomSetMacro(EgoMotionEdgeNbNeighbors, unsigned int)

  vtkCustomGetMacro(EgoMotionEdgeMinNbNeighbors, unsigned int)
  vtkCustomSetMacro(EgoMotionEdgeMinNbNeighbors, unsigned int)

  vtkCustomGetMacro(EgoMotionEdgeMaxModelError, double)
  vtkCustomSetMacro(EgoMotionEdgeMaxModelError, double)

  vtkCustomGetMacro(EgoMotionPlaneNbNeighbors, unsigned int)
  vtkCustomSetMacro(EgoMotionPlaneNbNeighbors, unsigned int)

  vtkCustomGetMacro(EgoMotionPlanarityThreshold, double)
  vtkCustomSetMacro(EgoMotionPlanarityThreshold, double)

  vtkCustomGetMacro(EgoMotionPlaneMaxModelError, double)
  vtkCustomSetMacro(EgoMotionPlaneMaxModelError, double)

  vtkCustomGetMacro(EgoMotionInitSaturationDistance, double)
  vtkCustomSetMacro(EgoMotionInitSaturationDistance, double)

  vtkCustomGetMacro(EgoMotionFinalSaturationDistance, double)
  vtkCustomSetMacro(EgoMotionFinalSaturationDistance, double)

  // Get/Set Localization
  vtkCustomGetMacro(LocalizationLMMaxIter, unsigned int)
  vtkCustomSetMacro(LocalizationLMMaxIter, unsigned int)

  vtkCustomGetMacro(LocalizationICPMaxIter, unsigned int)
  vtkCustomSetMacro(LocalizationICPMaxIter, unsigned int)

  vtkCustomGetMacro(LocalizationMaxNeighborsDistance, double)
  vtkCustomSetMacro(LocalizationMaxNeighborsDistance, double)

  vtkCustomGetMacro(LocalizationEdgeNbNeighbors, unsigned int)
  vtkCustomSetMacro(LocalizationEdgeNbNeighbors, unsigned int)

  vtkCustomGetMacro(LocalizationEdgeMinNbNeighbors, unsigned int)
  vtkCustomSetMacro(LocalizationEdgeMinNbNeighbors, unsigned int)

  vtkCustomGetMacro(LocalizationEdgeMaxModelError, double)
  vtkCustomSetMacro(LocalizationEdgeMaxModelError, double)

  vtkCustomGetMacro(LocalizationPlaneNbNeighbors, unsigned int)
  vtkCustomSetMacro(LocalizationPlaneNbNeighbors, unsigned int)

  vtkCustomGetMacro(LocalizationPlanarityThreshold, double)
  vtkCustomSetMacro(LocalizationPlanarityThreshold, double)

  vtkCustomGetMacro(LocalizationPlaneMaxModelError, double)
  vtkCustomSetMacro(LocalizationPlaneMaxModelError, double)

  vtkCustomGetMacro(LocalizationBlobNbNeighbors, unsigned int)
  vtkCustomSetMacro(LocalizationBlobNbNeighbors, unsigned int)

  vtkCustomGetMacro(LocalizationInitSaturationDistance, double)
  vtkCustomSetMacro(LocalizationInitSaturationDistance, double)

  vtkCustomGetMacro(LocalizationFinalSaturationDistance, double)
  vtkCustomSetMacro(LocalizationFinalSaturationDistance, double)

  vtkCustomGetMacroExternalSensor(WheelOdom, WheelOdomWeight, double)
  vtkCustomSetMacroExternalSensor(WheelOdom, WheelOdomWeight, double)

  vtkCustomGetMacroExternalSensor(WheelOdom, WheelOdomRelative, bool)
  vtkCustomSetMacroExternalSensor(WheelOdom, WheelOdomRelative, bool)

  vtkCustomGetMacroExternalSensor(Gravity, GravityWeight, double)
  vtkCustomSetMacroExternalSensor(Gravity, GravityWeight, double)

  vtkCustomGetMacroExternalSensor(Imu, ImuWeight, double)
  vtkCustomSetMacroExternalSensor(Imu, ImuWeight, double)

  void SetImuGravity(double x, double y, double z);

  vtkCustomGetMacro(ImuResetThreshold, unsigned int)
  vtkCustomSetMacro(ImuResetThreshold, unsigned int)

  vtkCustomGetMacro(ImuUpdate, bool)
  vtkCustomSetMacro(ImuUpdate, bool)

  vtkCustomGetMacroExternalSensor(Pose, PoseWeight, double)
  vtkCustomSetMacroExternalSensor(Pose, PoseWeight, double)

  vtkCustomGetMacro(SensorTimeOffset, double)
  vtkCustomSetMacro(SensorTimeOffset, double)

  vtkCustomGetMacro(SensorTimeThreshold, double)
  vtkCustomSetMacro(SensorTimeThreshold, double)

  vtkCustomGetMacro(SensorMaxMeasures, unsigned int)
  vtkCustomSetMacro(SensorMaxMeasures, unsigned int)

  void SetSensorTimeSynchronization(int mode);

  // ---------------------------------------------------------------------------
  //   Keypoints extractor, Key frames and Maps parameters
  // ---------------------------------------------------------------------------

  // Keypoints extractor
  vtkGetObjectMacro(KeyPointsExtractor, vtkSpinningSensorKeypointExtractor)
  virtual void SetKeyPointsExtractor(vtkSpinningSensorKeypointExtractor*);

  // Key frames
  vtkCustomGetMacro(KfDistanceThreshold, double)
  vtkCustomSetMacro(KfDistanceThreshold, double)

  vtkCustomGetMacro(KfAngleThreshold, double)
  vtkCustomSetMacro(KfAngleThreshold, double)

  // Set RollingGrid Parameters

  virtual unsigned int GetMapUpdate();
  virtual void SetMapUpdate(unsigned int mode);

  virtual unsigned int GetSubmapMode();
  virtual void SetSubmapMode(unsigned int mode);

  vtkCustomGetMacro(VoxelGridDecayingThreshold, double)
  vtkCustomSetMacro(VoxelGridDecayingThreshold, double)

  virtual int GetVoxelGridSamplingMode (LidarSlam::Keypoint k) const;
  virtual void SetVoxelGridSamplingMode(LidarSlam::Keypoint k, int sm);
  virtual double GetVoxelGridLeafSize(LidarSlam::Keypoint k) const;
  virtual void SetVoxelGridLeafSize(LidarSlam::Keypoint k, double s);

  // For edges
  virtual int GetVoxelGridSamplingModeEdges() const {return this->GetVoxelGridSamplingMode(LidarSlam::EDGE);}
  virtual void SetVoxelGridSamplingModeEdges(int sm)  {this->SetVoxelGridSamplingMode(LidarSlam::EDGE, sm);}
  virtual double GetVoxelGridLeafSizeEdges() const {return this->GetVoxelGridLeafSize(LidarSlam::EDGE);}
  virtual void SetVoxelGridLeafSizeEdges(double s) {this->SetVoxelGridLeafSize(LidarSlam::EDGE, s);}

  // For intensity edges
  virtual int GetVoxelGridSamplingModeIntensityEdges() const {return this->GetVoxelGridSamplingMode(LidarSlam::INTENSITY_EDGE);}
  virtual void SetVoxelGridSamplingModeIntensityEdges(int sm)  {this->SetVoxelGridSamplingMode(LidarSlam::INTENSITY_EDGE, sm);}
  virtual double GetVoxelGridLeafSizeIntensityEdges() const {return this->GetVoxelGridLeafSize(LidarSlam::INTENSITY_EDGE);}
  virtual void SetVoxelGridLeafSizeIntensityEdges(double s) {this->SetVoxelGridLeafSize(LidarSlam::INTENSITY_EDGE, s);}

  // For planes
  virtual int GetVoxelGridSamplingModePlanes() const {return this->GetVoxelGridSamplingMode(LidarSlam::Keypoint::PLANE);}
  virtual void SetVoxelGridSamplingModePlanes(int sm) { this->SetVoxelGridSamplingMode(LidarSlam::Keypoint::PLANE, sm);}
  virtual double GetVoxelGridLeafSizePlanes() const {return this->GetVoxelGridLeafSize(LidarSlam::Keypoint::PLANE);}
  virtual void SetVoxelGridLeafSizePlanes(double s) {this->SetVoxelGridLeafSize(LidarSlam::Keypoint::PLANE, s);}

  // For blobs
  virtual int GetVoxelGridSamplingModeBlobs() const {return this->GetVoxelGridSamplingMode(LidarSlam::Keypoint::BLOB);}
  virtual void SetVoxelGridSamplingModeBlobs(int sm) {this->SetVoxelGridSamplingMode(LidarSlam::Keypoint::BLOB, sm);}
  virtual double GetVoxelGridLeafSizeBlobs() const {return this->GetVoxelGridLeafSize(LidarSlam::Keypoint::BLOB);}
  virtual void SetVoxelGridLeafSizeBlobs(double s) {this->SetVoxelGridLeafSize(LidarSlam::Keypoint::BLOB, s);}

  vtkCustomSetMacroNoCheck(VoxelGridSize, int)
  vtkCustomSetMacroNoCheck(VoxelGridResolution, double)

  vtkCustomSetMacroNoCheck(VoxelGridMinFramesPerVoxel, unsigned int)

  // ---------------------------------------------------------------------------
  //   Confidence estimator parameters
  // ---------------------------------------------------------------------------

  vtkCustomGetMacro(OverlapSamplingRatio, double)
  virtual void SetOverlapSamplingRatio (double ratio);

  // Motion constraints
  virtual void SetAccelerationLimits(float linearAcc, float angularAcc);
  virtual void SetVelocityLimits(float linearVel, float angularVel);
  virtual void SetPoseLimits(float position, float orientation);

  virtual void SetLoggingTimeout(double loggingTimeout);
  vtkCustomGetMacro(LoggingTimeout, double)

  vtkGetMacro(ConfidenceWindow, unsigned int)
  virtual void SetConfidenceWindow (unsigned int window);

  vtkCustomGetMacro(OverlapDerivativeThreshold, double)
  vtkCustomSetMacro(OverlapDerivativeThreshold, double)

  vtkCustomGetMacro(PositionErrorThreshold, double)
  vtkCustomSetMacro(PositionErrorThreshold, double)

  vtkGetMacro(RecoveryTime, float)
  vtkSetMacro(RecoveryTime, float)

  vtkCustomGetMacro(FailureDetectionEnabled, bool)
  virtual void SetFailureDetectionEnabled(bool failDetect);

protected:
  vtkSlam();

  int FillInputPortInformation(int port, vtkInformation* info) override;
  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  // Identify input arrays to use
  bool IdentifyInputArrays(vtkPolyData* poly);

  // Convert VTK PolyData to PCL pointcloud
  // Returns true if all input points are valid (null coordinates), false otherwise
  bool PolyDataToPointCloud(vtkPolyData* poly,
                            LidarSlam::Slam::PointCloud::Ptr pc) const;

  // Convert PCL pointcloud to VTK PolyData
  void PointCloudToPolyData(LidarSlam::Slam::PointCloud::Ptr pc,
                            vtkPolyData* poly) const;

private:
  vtkSlam(const vtkSlam&) = delete;
  void operator=(const vtkSlam&) = delete;

  // ---------------------------------------------------------------------------
  //   Useful helpers
  // ---------------------------------------------------------------------------

  // Create polydata with trajectory arrays and points
  vtkSmartPointer<vtkPolyData> CreateInitTrajectory();

  // Init/reset the output SLAM trajectory
  // If endTime is set, remove the newest poses after endTime
  void ResetTrajectory(double endTime = -1.);

  // Add a SLAM pose and covariance in WORLD coordinates to Trajectory.
  void AddPoseToTrajectory(const LidarSlam::LidarState& state);

  // Add current SLAM pose and covariance in WORLD coordinates to Trajectory.
  void AddLastPosesToTrajectory();

  // Fill calibration arg with the calibration supplied in fileName file
  // Filename must be a .mat file with matrix saved as numbers spaced by empty char
  // Example :
  // 0.707107 0.640856 0.298836 -0.043362
  // -0.707107 0.640856 0.298836 0.032025
  // 0.000000 -0.422618 0.906308 -0.072949
  // 0.000000 0.000000 0.000000 1.000000
  bool GetCalibrationMatrix(const std::string& fileName, Eigen::Isometry3d& calibration) const;

  // ---------------------------------------------------------------------------
  //   Member attributes
  // ---------------------------------------------------------------------------

protected:

  // Keeps track of the time the parameters have been modified
  // This will enable the SlamManager to be time-agnostic
  // MTime is a much more general mecanism so we can't rely on it
  vtkTimeStamp ParametersModificationTime;

  std::unique_ptr<LidarSlam::Slam> SlamAlgo;
  vtkSpinningSensorKeypointExtractor* KeyPointsExtractor = nullptr;

private:

  // If true, the input point time data is relative to the frame time, such as with MCAP data. 
  bool PointTimeRelativeToFrame = false;

  // Member to store the current time
  double LastFrameTime = -1.;
  double FrameTime = -1.;

  // Cache to save maps
  std::map<LidarSlam::Keypoint, vtkSmartPointer<vtkPolyData>> CacheMaps;

  // Polydata which represents the computed trajectory
  vtkSmartPointer<vtkPolyData> Trajectory;

  // If enabled, advanced return mode will add arrays to outputs showing some
  // additional results or info of the SLAM algorithm such as :
  //  - Trajectory : matching summary, localization error summary
  //  - Output transformed frame : neighborhoods angle, intensity gap, depth gap, space gap
  //  - Extracted keypoints : ICP matching results
  bool AdvancedReturnMode = false;

  // Allows to choose which map keypoints to output
  OutputKeypointsMapsMode OutputKeypointsMaps = FULL_MAPS;

  // Update keypoints maps only each MapsUpdateStep frame
  // (ex: every frame, every 2 frames, 3 frames, ...)
  unsigned int MapsUpdateStep = 1;

  // If enabled, SLAM filter will output keypoints extracted from current
  // frame. Otherwise, these filter outputs are left empty to save time.
  bool OutputCurrentKeypoints = true;

  // If disabled, return raw keypoints extracted from current frame in BASE
  // coordinates, without undistortion. If enabled, return keypoints in WORLD
  // coordinates, optionally undistorted if undistortion is activated.
  // Only used if OutputCurrentKeypoints = true.
  bool OutputKeypointsInWorldCoordinates = true;

  // Arrays to use (depending on LiDAR model) to fill points data
  bool AutoDetectInputArrays = true;   ///< If true, try to auto-detect arrays to use. Otherwise, user needs to specify them.
  std::string TimeArrayName;           ///< Point measurement timestamp
  std::string IntensityArrayName;      ///< Point intensity/reflectivity values
  std::string LaserIdArrayName;        ///< Laser ring id
  double TimeToSecondsFactor = 1.;     ///< Coef to apply to TimeArray values to express time in seconds

  // SLAM initialization
  std::string InitMapPrefix; ///< Path prefix of initial maps
  Eigen::Vector6d InitPose;  ///< Initial pose of the SLAM

  // Internal variables to store confidence parameters when advanced
  // return mode and failure detection are disabled.
  float OverlapSamplingRatio = 0.25f;
  unsigned int ConfidenceWindow = 10;

  // In case of failure, duration to come back in time to previous state
  float RecoveryTime = 1.f;

  // Boolean to decide whether or not to use the pose graph
  bool UsePoseGraph = false;

  // Boolean to register whether or not a loop is detected
  bool LoopDetected = false;
  double LastLoopClosurePosition[3];

  // Storage value for loop detection information
  // Used in loop pop up actions
  LidarSlam::LoopClosure::LoopInfo LastLoopInfo;

  // Choose whether to synchronize on network packet
  // reception time or on Lidar frame header time
  bool SynchronizeOnPacket = false;

  // Sensor file name stored to reload the external sensor data after reset
  std::string ExtSensorFileName;

  // Output trajectory require frequency (Hz)
  double TrajFrequency = -1;

  // Boolean used in calibration process with external poses to precise if
  // the trajectory was planar (one degree of liberty is missing)
  // so we consider there is no lever arm in world z direction
  // This might be the case for vehicle acquisitions
  // If the boolean is set to false and the trajectory is planar
  // A big translation uncertainty might be added, leading to numerical unstability
  bool PlanarTrajectory = false;

  // When determining the calibration, a hint about the lever arm can be supplied
  // This allows to restrict the search to 5 variable elements
  // instead of 6 and allows to use less complex trajectories
  // The lever arm is the distance between the SLAM tracked
  // frame position and the external sensor frame position
  double LeverArm = -1.;

  // Number of poses for which to compute
  // the relative poses on which the calibration is sought
  // This window allows to reduce the drift effect on both trajectories
  // A too tight window can lead to numerical side effects
  // /!\ No interface has been done for simplicity
  int CalibrationWindow = 5;
};

#endif // VTK_SLAM_H
