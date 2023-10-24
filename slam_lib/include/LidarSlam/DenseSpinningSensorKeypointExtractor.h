//==============================================================================
// Copyright 2022 Kitware, Inc., Kitware SAS
// Author: Jeanne Faure (Kitware SAS)
// Creation date: 2023-10-12
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

#include "Utilities.h"
#include "LidarPoint.h"
#include "Enums.h"
#include "KeypointExtractor.h"
#include <unordered_map>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{

struct PtFeat
{
  int Index;
  float Depth;
  float SpaceGap;
  float DepthGap;
  float IntensityGap;
  float Angle;
  std::bitset<Keypoint::nKeypointTypes> KptTypes;

  PtFeat() : Index(0), Depth(0.0f), SpaceGap(-1.0f), DepthGap(-1.0f), IntensityGap(-1.0f), Angle(-1.0f), KptTypes({}) {}
};

struct IdxVM
{
  int Row;
  int Col;
};

class DenseSpinningSensorKeypointExtractor : public KeypointExtractor
{
public:
  // Extract keypoints from the pointcloud. The key points
  // will be separated in two classes : Edges keypoints which
  // correspond to area with high curvature scan lines and
  // planar keypoints which have small curvature.
  // NOTE: This expects that the lowest/bottom laser_id is 0, and is increasing upward.
  void ComputeKeyPoints(const PointCloud::Ptr& pc) override;

  PointCloud::Ptr GetKeypoints(Keypoint k) override;

  // Function to enable to have some inside on why a given point was detected as a keypoint
  std::unordered_map<std::string, std::vector<float>> GetDebugArray() const override;

private:

  // Find the pointer to PtFeat of a point in the scan (designated by its index in the scan)
  // (PtFeat containing the features of the point : space gap, depth...)
  std::shared_ptr<PtFeat> GetPtFeat(int idxInScan) const;

  // Count the number of non null ptr in a scanline
  int GetScanLineSize(const std::vector<std::shared_ptr<PtFeat>>& scanLine);

  // Initialize LaserIdMap, NbLaserRings, AzimuthalResolution and Pc2VmIndices
  void InitInternalParameters();

  // Create vertex map from input pointcloud using indices stored in Pc2VmIndices
  void CreateVertexMap();

  // Compute the curvature features within each scan line : depth
  // space gap, intensity gap and line angle
  void ComputeCurvature() override;

  // Labelize points (unvalid, edge, plane)
  // and extract them in correspondant pointcloud
  // note : blobs are extracted identically in SSKE and DSSKE so ComputeBlobs() is defined in parent abstract class
  void ComputePlanes() override;
  void ComputeEdges() override;
  void ComputeIntensityEdges() override;

  // Add point to keypoint structure
  void AddKeypoint(const Keypoint& k, const LidarPoint &pt) override;

  // Add all keypoints of the type k that comply with the threshold criteria for these values
  // The threshold can be a minimum or maximum value (threshIsMax)
  // The weight basis allow to weight the keypoint depending on its certainty
  void AddKptsUsingCriterion(Keypoint k);

  // ---------------------------------------------------------------------------
  //   Parameters
  // ---------------------------------------------------------------------------

  //TODO

  // ---------------------------------------------------------------------------
  //   Internal variables
  // ---------------------------------------------------------------------------

  // Dimensions of the Vertex Map
  int HeightVM;
  int WidthVM;

  // Map of laser_id to fit random laser_ids into {0, ..., NbLaserRings-1}
  std::unordered_map<int, int> LaserIdMap;

  // Vector linking the index of a point in the pointcloud to its index in the Vertex Map
  std::vector<IdxVM> Pc2VmIndices;

  // Vertex Map of points' indices in the pointcloud
  std::vector<std::vector<std::shared_ptr<PtFeat>>> VertexMap;
};

} // namespace LidarSlam