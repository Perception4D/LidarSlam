from email.policy import default
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription 
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')

  ###############
  ## ARGUMENTS ##
  ###############
  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("replay", default_value="true", description="Whether to process live or replayed data"),
    DeclareLaunchArgument("outdoor", default_value="true", description="Decide which set of parameters to use"),
    DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
    # Possibility to use VLP32 with appropriate launch files
    # Check github repo : https://github.com/ros-drivers/velodyne/tree/ros2
    DeclareLaunchArgument("os_driver", default_value="false", description="If true, activate os_node."),
    DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
    DeclareLaunchArgument("camera_topic", default_value="camera", description="topic from which to get the rgb camera data"),
    DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="topic from which to get the rgb camera info"),
    #  Ouster driver parameters
    DeclareLaunchArgument("sensor_hostname", default_value="10.5.5.96", description="Hostname or IP in dotted decimal form of the sensor"),
    DeclareLaunchArgument("udp_dest", default_value="", description="Hostname or IP where the sensor will send data packets"),
    DeclareLaunchArgument("lidar_port", default_value="7502", description="Port to which the sensor should send lidar data"),
    DeclareLaunchArgument("imu_port", default_value="7503", description="Port to which the sensor should send imu data"),
    DeclareLaunchArgument("lidar_mode", default_value="1024x10", description="Resolution modes for the LiDAR"),
    DeclareLaunchArgument("metadata_in", default_value=os.path.join(lidar_slam_share_path, 'params', 'metadata_OS1_64_1024x10.json'), description="Configuration file for Ouster data to replay"),
    DeclareLaunchArgument("eth_device", default_value="lo", description="Ethernet interfaces used for replaying data"),
    # /!\ rpm and timestamp_first_packet are also used to generate approximate point-wise timestamps as 'time' field is not usable. -->
    DeclareLaunchArgument("rpm", default_value="600.", description="Ouster sensor spinning speed."),
    DeclareLaunchArgument("timestamp_first_packet", default_value="false", description="If Ouster timestamping is based on the first or last packet of each scan."),
    DeclareLaunchArgument("aggregate", default_value="false", description="Run aggregation node"),
  ])

  ##########
  ## Rviz ##
  ##########
  rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2",
    arguments=["-d", os.path.join(lidar_slam_share_path, 'params', 'slam.rviz')],
    parameters=[{'use_sim_time': LaunchConfiguration('replay')},],
    condition = IfCondition(LaunchConfiguration("rviz")),
  )

  #####################
  ### Ouster driver ###
  #####################
  ouster_driver_path = get_package_share_directory("ouster_ros")

  #for replay
  group_ouster = GroupAction(
    actions=[
      # TODO Replay mode after live mode works
      #! TEST IT
      # Replay
      # IncludeLaunchDescription(
      #   PythonLaunchDescriptionSource([os.path.join(ouster_driver_path, "launch"), "/replay.launch.xml"]),
      #   launch_arguments={
      #     'viz' : False,
      #     # 'metadata': LaunchConfiguration("metadata_in"),
      #   }.items(),
      #   condition=IfCondition(LaunchConfiguration("replay")),
      # ),

      IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(ouster_driver_path, "launch", "sensor.launch.xml")]),
        launch_arguments={
          "sensor_hostname" : LaunchConfiguration("sensor_hostname"),
          "udp_dest"        : LaunchConfiguration("udp_dest"),
          "lidar_port"      : LaunchConfiguration("lidar_port"),
          "imu_port"        : LaunchConfiguration("imu_port"),
          "lidar_mode"      : LaunchConfiguration("lidar_mode"),
          "timestamp_mode"  : "TIME_FROM_INTERNAL_OSC",
          "metadata"        : LaunchConfiguration("metadata_in"),
          "sensor_frame"    : "laser_sensor_frame",
          "laser_frame"     : "laser_data_frame",
          "imu_frame"       : "imu_data_frame",
          "viz"             : "False",
      }.items(),
        condition=UnlessCondition(LaunchConfiguration("replay")),
      ),
    ],
    condition=IfCondition(LaunchConfiguration("os_driver"))
  )

  ##########
  ## Slam ##
  ##########

  # Ouster points conversion
  ouster_conversion_node = Node(
    name="ouster_conversion", package="lidar_conversions", executable="ouster_conversion_node", output="screen",
    parameters=[{
      "rpm": LaunchConfiguration("rpm"),
      "timestamp_first_packet": LaunchConfiguration("timestamp_first_packet"),
      "use_sim_time": LaunchConfiguration("replay"),
    }],
  )

  # LiDAR SLAM : compute TF slam_init -> velodyne

  # Outdoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_outdoor.yaml"), 'r') as f:
    params_slam_out = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  # Manualy override lidar_config_outdoor_node parameters from parameter file
  params_slam_out['use_sim_time'] = LaunchConfiguration("replay")

  slam_outdoor_node = Node(name="lidar_slam", package="lidar_slam", executable="lidar_slam_node", output="screen",
    parameters=[params_slam_out],
    remappings=[("tag_detections", LaunchConfiguration("tags_topic")),
                ("camera", LaunchConfiguration("camera_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),],
    condition=IfCondition(LaunchConfiguration("outdoor")),
  )

  # Indoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_indoor.yaml"), 'r') as f:
    params_slam_in = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  # Manualy override lidar_config_indoor_node parameters from parameter file
  params_slam_in['use_sim_time'] = LaunchConfiguration("replay")

  slam_indoor_node = Node(name="lidar_slam", package="lidar_slam", executable="lidar_slam_node", output="screen",
    parameters=[params_slam_in],
    remappings=[("tag_detections", LaunchConfiguration("tags_topic")),
                ("camera", LaunchConfiguration("camera_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),],
    condition= UnlessCondition(LaunchConfiguration("outdoor")),
  )

  # Aggregate points
  slam_aggregation_config_path = os.path.join(lidar_slam_share_path, "params", "aggregation_config.yaml")
  aggregation_node = Node(name="aggregation", package="lidar_slam", executable="aggregation_node", output="screen",
    parameters=[slam_aggregation_config_path], 
    condition=IfCondition(LaunchConfiguration("aggregate")),
  )

  # Moving base coordinates systems description                                         tf_FROM_to_TO
  tf_base_to_os_node = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_base_to_lidar",
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0", 
               "--frame-id", "base_link", "--child-frame-id", "laser_sensor_frame"],
  )

  # Moving base coordinates systems description                                     tf_FROM_to_TO
  gps_tf_node = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_base_to_gps",
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0", 
               "--frame-id", "base_link", "--child-frame-id", "gps"],
  )

  # Default transformation for Odom frame
  odom_tf_node = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_odom_to_base",
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0", 
               "--frame-id", "odom", "--child-frame-id", "base_link"],
    )

  ld.add_action(rviz_node)
  ld.add_action(ouster_conversion_node)
  ld.add_action(group_ouster)
  ld.add_action(slam_outdoor_node)
  ld.add_action(slam_indoor_node)
  ld.add_action(aggregation_node)
  ld.add_action(tf_base_to_os_node)
  ld.add_action(gps_tf_node)
  ld.add_action(odom_tf_node)
  return (ld)
