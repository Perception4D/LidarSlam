
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')
  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("replay", default_value="true", description="Whether to process live or replayed data"),
    DeclareLaunchArgument("pointcloud2", default_value="false" , description="True if pointcloud message is in pointcloud2 format"),
    DeclareLaunchArgument("outdoor", default_value="true", description="Decide which set of parameters to use."),
    DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
    DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
    DeclareLaunchArgument("camera_topic", default_value="camera", description="topic from which to get the rgb camera data"),
    DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="topic from which to get the rgb camera info"),
    DeclareLaunchArgument("aggregate", default_value="false", description="run aggregation node"),
  ])

  ##########
  ## Rviz ##
  ##########
  rviz_node = Node(package="rviz2", executable="rviz2",
    arguments=["-d", os.path.join(lidar_slam_share_path, 'params', 'slam.rviz')],
    parameters=[{'use_sim_time': LaunchConfiguration('replay')},],
    condition = IfCondition(LaunchConfiguration("rviz")),
  )

  livox_conversion_node = Node(
      name="livox_conversion_node", package="lidar_conversions", executable="livox_conversion_node", output="screen",
      parameters=[{
          "pointcloud2" :LaunchConfiguration("pointcloud2"),
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
               "--frame-id", "base_link", "--child-frame-id", "livox_frame"],
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
  ld.add_action(livox_conversion_node)
  ld.add_action(slam_outdoor_node)
  ld.add_action(slam_indoor_node)
  ld.add_action(aggregation_node)
  ld.add_action(tf_base_to_os_node)
  ld.add_action(gps_tf_node)
  ld.add_action(odom_tf_node)

  return (ld)