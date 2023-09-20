//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Arthur Bourbousson (Kitware SAS)
// Creation date: 2023-06-16
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

#include "SlamControlPanel.h"


namespace slam_visualization
{

//----------------------------------------------------------------------------
SlamControlPanel::SlamControlPanel(QWidget* parent)
{
  this->CreateLayout();

  this->visualization_node = std::make_shared<rclcpp::Node>("slam_control_pannel_node");
  this->CommandPublisher = this->visualization_node->create_publisher<lidar_slam::msg::SlamCommand>("slam_command", 1);
}

//----------------------------------------------------------------------------
void  SlamControlPanel::onInitialize()
{
  // Access to rviz node using the context of the plugin
  auto rviz_ros_node = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Create parallel callback group to separate our interfaces from other rviz2 interfaces
  this->SlamCallbackGroup = rviz_ros_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions ops;
  ops.callback_group = this->SlamCallbackGroup;

  this->ConfidenceSubscriber = rviz_ros_node->create_subscription<lidar_slam::msg::Confidence>("/slam_confidence", 1,
                               std::bind(&SlamControlPanel::SlamConfidenceCallback, this, std::placeholders::_1), ops);
}

//----------------------------------------------------------------------------
void SlamControlPanel::CreateLayout()
{
  auto resetStateButton = new QPushButton;
  resetStateButton->setText("Reset state");
  connect(resetStateButton, &QPushButton::clicked, this, &SlamControlPanel::ResetSlamState);

  auto disableMapUpdateButton = new QPushButton;
  disableMapUpdateButton->setText("Disable map update");
  connect(disableMapUpdateButton, &QPushButton::clicked, this, &SlamControlPanel::DisableMapUpdate);

  auto enableMapExpansionButton = new QPushButton;
  enableMapExpansionButton->setText("Enable map expansion");
  connect(
    enableMapExpansionButton, &QPushButton::clicked, this, &SlamControlPanel::EnableMapExpansion);

  auto enableMapUpdateButton = new QPushButton;
  enableMapUpdateButton->setText("Enable map update");
  connect(enableMapUpdateButton, &QPushButton::clicked, this, &SlamControlPanel::EnableMapUpdate);

  auto commandLayout = new QVBoxLayout;
  commandLayout->addWidget(resetStateButton);
  commandLayout->addWidget(disableMapUpdateButton);
  commandLayout->addWidget(enableMapExpansionButton);
  commandLayout->addWidget(enableMapUpdateButton);

  auto commandBox = new QGroupBox;
  commandBox->setLayout(commandLayout);
  commandBox->setTitle("Commands");

  // Confidence
  auto overlapLabel = new QLabel{ "Overlap:" };
  this->OverlapValueLabel = new QLabel;
  this->OverlapValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  overlapLabel->setBuddy(overlapLabel);

  auto complyMotionLimitsLabel = new QLabel{ "Comply motion limits: " };
  this->ComplyMotionLimitsValueLabel = new QLabel;
  this->ComplyMotionLimitsValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  complyMotionLimitsLabel->setBuddy(this->ComplyMotionLimitsValueLabel);

  auto computationTimeLabel = new QLabel{ "Frame computation time: " };
  this->ComputationTimeValueLabel = new QLabel;
  this->ComputationTimeValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  computationTimeLabel->setBuddy(this->ComputationTimeValueLabel);

  auto confidenceLayout = new QGridLayout;
  confidenceLayout->addWidget(overlapLabel, 0, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->OverlapValueLabel, 0, 1, Qt::AlignRight);
  confidenceLayout->addWidget(complyMotionLimitsLabel, 1, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->ComplyMotionLimitsValueLabel, 1, 1, Qt::AlignRight);
  confidenceLayout->addWidget(computationTimeLabel, 2, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->ComputationTimeValueLabel, 2, 1, Qt::AlignRight);

  auto confidenceBox = new QGroupBox;
  confidenceBox->setTitle("Confidence estimator");
  confidenceBox->setLayout(confidenceLayout);

  auto mainLayout = new QVBoxLayout;
  mainLayout->addWidget(commandBox);
  mainLayout->addWidget(confidenceBox);

  setLayout(mainLayout);
}

//----------------------------------------------------------------------------
void SlamControlPanel::ResetSlamState()
{
  this->SendCommand(lidar_slam::msg::SlamCommand::RESET_SLAM);
}

//----------------------------------------------------------------------------
void SlamControlPanel::DisableMapUpdate()
{
  this->SendCommand(lidar_slam::msg::SlamCommand::DISABLE_SLAM_MAP_UPDATE);
}

//----------------------------------------------------------------------------
void SlamControlPanel::EnableMapExpansion()
{
  this->SendCommand(lidar_slam::msg::SlamCommand::ENABLE_SLAM_MAP_EXPANSION);
}

//----------------------------------------------------------------------------
void SlamControlPanel::EnableMapUpdate()
{
  this->SendCommand(lidar_slam::msg::SlamCommand::ENABLE_SLAM_MAP_UPDATE);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SendCommand(
  lidar_slam::msg::SlamCommand::_command_type command, lidar_slam::msg::SlamCommand::_string_arg_type arg)
{
  lidar_slam::msg::SlamCommand msg;
  msg.command = command;
  msg.string_arg = std::move(arg);
  this->CommandPublisher->publish(msg);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SlamConfidenceCallback(const lidar_slam::msg::Confidence &confidence)
{
  this->OverlapValueLabel->setText(
    QString::number(static_cast<int>(confidence.overlap * 100)) + '%');

  QPalette palette = this->ComplyMotionLimitsValueLabel->palette();
  palette.setColor(this->ComplyMotionLimitsValueLabel->foregroundRole(),
    confidence.comply_motion_limits ? Qt::black : Qt::red);

  this->ComplyMotionLimitsValueLabel->setPalette(palette);
  this->ComplyMotionLimitsValueLabel->setText(confidence.comply_motion_limits ? "Yes" : "No");

  this->ComputationTimeValueLabel->setText(
    QString::number(confidence.computation_time * 1000.0) + " ms");
}

} // namespace slam_visualization.

PLUGINLIB_EXPORT_CLASS(slam_visualization::SlamControlPanel, rviz_common::Panel)
