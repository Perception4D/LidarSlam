from email.policy import default
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import TextSubstitution, LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')
  lidar_conversion_share_path = get_package_share_directory('lidar_conversions')

  ###############
  ## ARGUMENTS ##
  ###############
  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time when replaying rosbags with '--clock' option."),
    DeclareLaunchArgument("outdoor", default_value="true", description="Decide which set of parameters to use"),
    DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
    DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
    DeclareLaunchArgument("camera_topic", default_value="camera", description="topic from which to get the rgb camera data"),
    DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="topic from which to get the rgb camera info"),
    # Velodyne driver parameters
    DeclareLaunchArgument("velodyne_driver", default_value="false", description="If true, start Velodyne driver."),
    DeclareLaunchArgument("calibration_file_path", default_value=os.path.join(get_package_share_directory('velodyne_pointcloud'), 'params', 'VLP16db.yaml'), description="calibration file path"),
    DeclareLaunchArgument("model", default_value="VLP16", description="Model of Velodyne Lidar, choices are : VLP16 / 32C / VLS128"),
    DeclareLaunchArgument("device_ip", default_value=""),
    DeclareLaunchArgument("rpm", default_value="600.0"),
    DeclareLaunchArgument("port", default_value=TextSubstitution(text="2368")),
    DeclareLaunchArgument("pcap", default_value=""),
    DeclareLaunchArgument("aggregate", default_value="false", description="Run aggregation node"),
    DeclareLaunchArgument("domain_id", default_value="0", description="Set to different value to avoid interference when several computers running ROS2 on the same network."),
    SetEnvironmentVariable(name='ROS_DOMAIN_ID',value=LaunchConfiguration('domain_id')),
  ])

  ##########
  ## Rviz ##
  ##########
  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", os.path.join(lidar_slam_share_path, 'params', 'slam.rviz')],
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},],
    condition = IfCondition(LaunchConfiguration("rviz"))
  )

  ##############
  ## Velodyne ##
  ##############

  #! For now, velodyne packages are not ported on Windows 10
  if os.name != 'nt':
    # Manually override velodyne_driver_node parameters
    params_driver_path = os.path.join(get_package_share_directory('velodyne_driver'), 'config', 'VLP16-velodyne_driver_node-params.yaml')
    with open(params_driver_path, 'r') as f:
      params_velod_driv = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

    params_velod_driv['device_ip']    =  LaunchConfiguration('device_ip')
    params_velod_driv["gps_time"]     = False
    params_velod_driv["read_once"]    = True
    params_velod_driv["read_fast"]    = False
    params_velod_driv["repeat_delay"] = 0.0
    params_velod_driv["frame_id"]     = "velodyne"
    params_velod_driv["model"]        = LaunchConfiguration('model')
    params_velod_driv["rpm"]          = LaunchConfiguration('rpm')
    params_velod_driv["pcap"]         = LaunchConfiguration('pcap')
    params_velod_driv["port"]         = LaunchConfiguration('port')
    params_velod_driv["use_sim_time"] = False

    # Manually override velodyne_convert_node parameters
    velodyne_pointcloud_share_path = get_package_share_directory('velodyne_pointcloud')
    params_velod_pcl_path = os.path.join(velodyne_pointcloud_share_path, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(params_velod_pcl_path, 'r') as f:
        params_velod_pcl = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']

    params_velod_pcl['calibration']    = LaunchConfiguration('calibration_file_path')
    params_velod_pcl["model"]          = LaunchConfiguration('model')
    params_velod_pcl["min_range"]      = 0.4
    params_velod_pcl["max_range"]      = 130.0
    params_velod_pcl["organize_cloud"] = False
    params_velod_pcl["target_frame"]   = ""
    params_velod_pcl["fixed_frame"]    = ""
    params_velod_pcl["use_sim_time"]   = LaunchConfiguration('use_sim_time')

    velodyne_group = GroupAction(
      actions=[
        # Part 1
        Node(
          package='velodyne_driver',
          executable='velodyne_driver_node',
          name='velodyne_driver_node',
          output='both',
          parameters=[params_velod_driv]),
        # Part 2
        Node(
          package='velodyne_pointcloud',
          executable='velodyne_transform_node',
          output='both',
          name='velodyne_transform_node',
          parameters=[params_velod_pcl])
      ],
      condition = IfCondition(LaunchConfiguration("velodyne_driver"))
    )

  ##########
  ## Slam ##
  ##########

  # Velodyne points conversion
  with open(os.path.join(lidar_conversion_share_path, 'params', "conversion_config.yaml"), 'r') as f:
    params_conversion = yaml.safe_load(f)['/lidar_conversions']['ros__parameters']
  params_conversion['use_sim_time'] = LaunchConfiguration("use_sim_time")

  velodyne_conversion_node = Node(
    package="lidar_conversions",
    executable="velodyne_conversion_node",
    name="velodyne_conversion",
    output="screen",
    parameters=[params_conversion]
  )

  # Outdoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_outdoor.yaml"), 'r') as f:
    params_slam_out = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  params_slam_out['use_sim_time'] = LaunchConfiguration("use_sim_time")

  slam_outdoor_node = Node(
    name="lidar_slam",
    package="lidar_slam",
    executable="lidar_slam_node",
    output="screen",
    parameters=[params_slam_out],
    remappings=[("tag_detections", LaunchConfiguration("tags_topic")),
                ("camera", LaunchConfiguration("camera_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),],
    condition=IfCondition(LaunchConfiguration("outdoor"))
  )

  # Indoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_indoor.yaml"), 'r') as f:
    params_slam_in = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  params_slam_in['use_sim_time'] = LaunchConfiguration("use_sim_time")

  slam_indoor_node = Node(
    name="lidar_slam",
    package="lidar_slam",
    executable="lidar_slam_node",
    output="screen",
    parameters=[params_slam_in],
    remappings=[("tag_detections", LaunchConfiguration("tags_topic")),
                ("camera", LaunchConfiguration("camera_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),],
    condition= UnlessCondition(LaunchConfiguration("outdoor"))
  )

  # Aggregate points
  print( os.path.join(lidar_slam_share_path, 'params', "aggregation_config.yaml"))
  with open(os.path.join(lidar_slam_share_path, 'params', "aggregation_config.yaml"), 'r') as f:
    params_aggregation = yaml.safe_load(f)['/aggregation']['ros__parameters']
  params_aggregation['use_sim_time'] = LaunchConfiguration("use_sim_time")

  aggregation_node = Node(
    name="aggregation",
    package="lidar_slam",
    executable="aggregation_node",
    output="screen",
    parameters=[params_aggregation],
    condition=IfCondition(LaunchConfiguration("aggregate"))
  )

  # Static TF base to velodyne LiDAR
  tf_base_to_velodyne = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_lidar",
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},],
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "base_link", "--child-frame-id", "velodyne"]
  )

  # Static TF base to wheel
  tf_base_to_wheel = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_wheel",
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "base_link", "--child-frame-id", "wheel"]
  )

  # Static TF base to ext sensor
  tf_base_to_ext_sensor = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_ext_sensor",
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "base_link", "--child-frame-id", "ext_sensor"]
  )

  ld.add_action(rviz_node)
  if os.name != "nt" :
    ld.add_action(velodyne_group)
  ld.add_action(velodyne_conversion_node)
  ld.add_action(slam_outdoor_node)
  ld.add_action(slam_indoor_node)
  ld.add_action(aggregation_node)
  # TF
  ld.add_action(tf_base_to_velodyne)
  ld.add_action(tf_base_to_wheel)
  ld.add_action(tf_base_to_ext_sensor)

  return (ld)
