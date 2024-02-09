//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
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

#include "sqSubsetPropertiesDialog.h"
#include "vtkSlamFinder.h"

#include <pqActiveObjects.h>
#include <pqCoreUtilities.h>
#include <pqPipelineFilter.h>
#include <pqPipelineSource.h>
#include <pqProxyWidgetDialog.h>
#include <vtkSMSourceProxy.h>

#include <QStringList>

namespace
{
constexpr const char* ADD_EXTERNAL_SENSOR_TITLE = "Add External Sensor Dialog";
const QStringList ADD_EXTERNAL_SENSOR_PROPERTIES = { "External sensors data file", "Calibrate",
  "Ego-Motion mode", "Undistortion mode", "Wheel odom weight", "Wheel odometry relative mode",
  "IMU gravity weight", "IMU weight", "Gravity", "Pose weight", "Imu reset threshold", "Imu update",
  "Time synchronization" };

constexpr const char* OPTIMIZE_GRAPH_TITLE = "Optimize Graph Dialog";
const QStringList OPTIMIZE_GRAPH_PROPERTIES = { "Optimize Graph", "Logging timeout",
  "Use loop closure constraint", "Use landmark constraint", "Use GPS constraint",
  "Use external pose constraint", "Loop closure detector", "Load loop indices", "Detect Loop" };
}

//-----------------------------------------------------------------------------
sqSubsetPropertiesDialog::sqSubsetPropertiesDialog(QAction* parentObject, PropertiesSubset type)
  : Superclass(parentObject)
  , Type(type)
{
}

//-----------------------------------------------------------------------------
sqSubsetPropertiesDialog::~sqSubsetPropertiesDialog() = default;

//-----------------------------------------------------------------------------
void sqSubsetPropertiesDialog::showSubsetPropertiesDialog(
  const QString& title, const QStringList& properties)
{
  pqPipelineFilter* filter =
    qobject_cast<pqPipelineFilter*>(pqActiveObjects::instance().activeSource());

  if (filter && vtkSlamFinder::isSlamFilter(filter->getSourceProxy()))
  {
    if (!this->Dialogs[this->Type])
    {
      this->Dialogs[this->Type] = std::make_unique<pqProxyWidgetDialog>(
        filter->getProxy(), properties, pqCoreUtilities::mainWidget());
      this->Dialogs[this->Type]->setWindowTitle(title);
    }
    this->Dialogs[this->Type]->show();
    this->Dialogs[this->Type]->raise();
    this->Dialogs[this->Type]->activateWindow();
  }
}

//-----------------------------------------------------------------------------
void sqSubsetPropertiesDialog::onTriggered()
{
  switch (this->Type)
  {
    case PropertiesSubset::ADD_EXTERNAL_SENSOR:
      this->showSubsetPropertiesDialog(
        ::ADD_EXTERNAL_SENSOR_TITLE, ::ADD_EXTERNAL_SENSOR_PROPERTIES);
      break;

    case PropertiesSubset::OPTIMIZE_GRAPH:
      this->showSubsetPropertiesDialog(
        ::OPTIMIZE_GRAPH_TITLE, ::OPTIMIZE_GRAPH_PROPERTIES);
      break;

    default:
      qWarning("Invalid properties subset type.");
      break;
  }
}
