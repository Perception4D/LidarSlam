from email.policy import default
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import TextSubstitution, LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')

  ###############
  ## ARGUMENTS ##
  ###############
  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time when replaying rosbags with '--clock' option."),
    DeclareLaunchArgument("outdoor", default_value="true", description="Decide which set of parameters to use"),
    DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
    # Possibility to use VLP32 with appropriate launch files
    # Check github repo : https://github.com/ros-drivers/velodyne/tree/ros2
    DeclareLaunchArgument("vlp16_driver", default_value="false", description="If true, start Velodyne driver for vlp16."),
    DeclareLaunchArgument("gps", default_value="false", description="If true, use GPS data to calibrate SLAM output. Otherwise, provide calibration."),
    DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
    # Velodyne driver parameters
    DeclareLaunchArgument("device_ip", default_value=""),
    DeclareLaunchArgument("port", default_value=TextSubstitution(text="2368")),
    DeclareLaunchArgument("pcap", default_value=""),
    #! /!\ rpm and timestamp_first_packet are also used to generate approximate point-wise timestamps as 'time' field is not usable.
    DeclareLaunchArgument("rpm", default_value=TextSubstitution(text="600."), description="Velodyne sensor spinning speed."),
    DeclareLaunchArgument("timestamp_first_packet", default_value="false", description="If Velodyne timestamping is based on the first or last packet of each scan."),
    DeclareLaunchArgument("aggregate", default_value="false", description="Run aggregation node"),
  ])

  ##########
  ## Rviz ##
  ##########
  rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2",
    arguments=["-d", os.path.join(lidar_slam_share_path, 'params', 'slam.rviz')],
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},],
    condition = IfCondition(LaunchConfiguration("rviz")),
  )

  ##############
  ## Velodyne ##
  ##############

  # Manualy override velodyne_driver_node parameters
  params_velod_driver_path = os.path.join(get_package_share_directory('velodyne_driver'), 'config', 'VLP16-velodyne_driver_node-params.yaml')
  with open(params_velod_driver_path, 'r') as f:
    params_velod_driv = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

  params_velod_driv['device_ip']    =  LaunchConfiguration('device_ip')
  params_velod_driv["gps_time"]     = False
  params_velod_driv["read_once"]    = True
  params_velod_driv["read_fast"]    = False
  params_velod_driv["repeat_delay"] = 0.0
  params_velod_driv["frame_id"]     = "velodyne"
  params_velod_driv["model"]        = "VLP16"
  params_velod_driv["rpm"]          = LaunchConfiguration('rpm')
  params_velod_driv["pcap"]         = LaunchConfiguration('pcap')
  params_velod_driv["port"]         = LaunchConfiguration('port')

  # Manualy override velodyne_convert_node parameters 
  velodyne_pointcloud_share_path = get_package_share_directory('velodyne_pointcloud')
  params_velod_pcl_path = os.path.join(velodyne_pointcloud_share_path, 'config', 'VLP16-velodyne_convert_node-params.yaml')
  with open(params_velod_pcl_path, 'r') as f:
      params_velod_pcl = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
  
  params_velod_pcl['calibration']    = os.path.join(velodyne_pointcloud_share_path, 'params', 'VLP16db.yaml')
  params_velod_pcl["min_range"]      = 0.4
  params_velod_pcl["max_range"]      = 130.0
  params_velod_pcl["organize_cloud"] = False
  params_velod_pcl["target_frame"]   = ""
  params_velod_pcl["fixed_frame"]    = ""

  velodyne_group = GroupAction(
    actions=[
      # Start driver node
      Node(package='velodyne_driver', executable='velodyne_driver_node', name='velodyne_driver_node', output='both',
        parameters=[params_velod_driver_path]),
      # Start convertion node
      Node(package='velodyne_pointcloud', executable='velodyne_convert_node', output='both', name='velodyne_convert_node',
        parameters=[params_velod_pcl],)
    ],
    condition = IfCondition(LaunchConfiguration("vlp16_driver"))
  )

  ##########
  ## Slam ##
  ##########

  # Velodyne points conversion
  velodyne_conversion_node = Node(
    package="lidar_conversions", executable="velodyne_conversion_node", name="velodyne_conversion", output="screen",
    parameters=[{
      "use_sim_time": LaunchConfiguration("use_sim_time"),
      "rpm": LaunchConfiguration("rpm"),
      "timestamp_first_packet": LaunchConfiguration("timestamp_first_packet"),
    }],
  )

  # LiDAR SLAM : compute TF slam_init -> velodyne
  # Outdoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_outdoor.yaml"), 'r') as f:
    params_slam_out = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  params_slam_out['use_sim_time'] = LaunchConfiguration("use_sim_time")
  params_slam_out['gps.use_gps'] = LaunchConfiguration("gps")

  slam_outdoor_node = Node(name="lidar_slam", package="lidar_slam", executable="lidar_slam_node", output="screen",
    parameters=[params_slam_out],
    remappings=[("tag_detections", LaunchConfiguration("tags_topic")),],
    condition=IfCondition(LaunchConfiguration("outdoor")),
  )
  # Indoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_indoor.yaml"), 'r') as f:
    params_slam_in = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  params_slam_in['use_sim_time'] = LaunchConfiguration("use_sim_time")
  params_slam_in['gps.use_gps'] = LaunchConfiguration("gps")

  slam_indoor_node = Node(name="lidar_slam", package="lidar_slam", executable="lidar_slam_node", output="screen",
    parameters=[params_slam_in],
    remappings=[("tag_detections", LaunchConfiguration("tags_topic")),],
    condition= UnlessCondition(LaunchConfiguration("outdoor")),
  )

  # Aggregate points
  slam_aggregation_config_path = os.path.join(lidar_slam_share_path, "params", "aggregation_config.yaml")
  aggregation_node = Node(name="aggregation", package="lidar_slam", executable="aggregation_node", output="screen",
    parameters=[slam_aggregation_config_path], 
    condition=IfCondition(LaunchConfiguration("aggregate")),
  )

  # Moving base coordinates systems description            tf_FROM_to_TO           X  Y  Z  rZ rY rX  FROM     TO
  tf_base_to_velo_node = Node( package="tf2_ros",executable="static_transform_publisher",name="tf_base_to_lidar",
    arguments=['--frame-id', 'base_link', '--child-frame-id', 'velodyne'],
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},],
  )

  # Launch GPS/UTM conversions nodes
  gps_conversions_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(lidar_slam_share_path, "launch", "gps_conversions.launch")]),
    condition=IfCondition(LaunchConfiguration("gps"))
  )

  # Moving base coordinates systems description                                     tf_FROM_to_TO
  gps_tf_node = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_base_to_gps",
    #           X    Y    Z    rZ   rY   rX      FROM      TO
    arguments=["0", "0", "0", "0", "0", "0", "base_link", "gps"]
  )

  ld.add_action(rviz_node)
  ld.add_action(velodyne_group)
  ld.add_action(velodyne_conversion_node)
  ld.add_action(slam_outdoor_node)
  ld.add_action(slam_indoor_node)
  ld.add_action(aggregation_node)
  ld.add_action(tf_base_to_velo_node)
  ld.add_action(gps_conversions_include)
  ld.add_action(gps_tf_node)
  return (ld)
