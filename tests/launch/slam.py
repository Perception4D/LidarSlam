
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')

  ###############
  ## ARGUMENTS ##
  ###############
  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("test_data",    default_value="",      description="Path to the test data"),
    DeclareLaunchArgument("res_path",     default_value="/tmp",  description="Path to the folder where to store the results"),
    DeclareLaunchArgument("ref_path",     default_value="",      description="Path to the reference data folder for results comparison"),
    DeclareLaunchArgument("outdoor",      default_value="true",  description="Decide which set of parameters to use"),
    DeclareLaunchArgument("vlp16",        default_value="false", description="If true, start Velodyne VLP16 transform node."),
    DeclareLaunchArgument("wait_init",    default_value="1",     description="Wait for test node initialization to replay data"),
    DeclareLaunchArgument("use_sim_time", default_value="true", description="Sim Time, used when replaying rosbag files"),

  # LiDAR pointclouds conversions args. These are only used to generate
  # approximate point-wise timestamps if 'time' field is not usable).
  # These parameters should be set to the same values as ROS Velodyne driver's.
  DeclareLaunchArgument("rpm", default_value="600.", description="Velodyne sensor spinning speed."),
  DeclareLaunchArgument("timestamp_first_packet", default_value="false", description="If Velodyne timestamping is based on the first or last packet of each scan."),

  ])

  ##############
  ## Velodyne ##
  ##############

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

  # Convertion node
  velodyne_pcl_node = Node(
    package='velodyne_pointcloud', executable='velodyne_convert_node', output='both', name='velodyne_pcl_node',
    parameters=[params_velod_pcl],
    condition = IfCondition(LaunchConfiguration("vlp16"))
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

  # Test : catch outputs and compare with reference
  # If comparison is required, end process when the reference is finished
  # If comparison is not required, end process when the bag ends

  with open(os.path.join(get_package_share_directory('lidar_slam_test'), "params/eval.yaml"), 'r') as f:
      params_lidar_slam_test = yaml.safe_load(f)['/lidar_slam_test']['ros__parameters']
  
  params_lidar_slam_test["res_path"]      = LaunchConfiguration("res_path")
  params_lidar_slam_test["ref_path"]      = LaunchConfiguration("ref_path")
  params_lidar_slam_test["use_sim_time"]  = LaunchConfiguration("use_sim_time")

  lidar_slam_test_req = Node(
    name="test", package="lidar_slam_test", executable="lidar_slam_test_node", output="screen",
    parameters=[params_lidar_slam_test],
    condition= LaunchConfigurationEquals('ref_path', ''),
    on_exit=Shutdown(),
  )

  lidar_slam_test_not_req = Node(
    name="test", package="lidar_slam_test", executable="lidar_slam_test_node", output="screen",
    parameters=[params_lidar_slam_test],
    condition= LaunchConfigurationNotEquals('ref_path', ''),
  )

  # LiDAR SLAM : compute TF slam_init -> velodyne

  # Outdoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_outdoor.yaml"), 'r') as f:
    params_slam_out = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  # Manualy override lidar_config_outdoor_node parameters from parameter file
  params_slam_out['use_sim_time'] = LaunchConfiguration("use_sim_time")
  params_slam_out['slam.verbosity'] = 0

  slam_outdoor_node = Node(name="lidar_slam", package="lidar_slam", executable="lidar_slam_node", output="screen",
    parameters=[params_slam_out],
    condition=IfCondition(LaunchConfiguration("outdoor")),
  )

  # Indoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_indoor.yaml"), 'r') as f:
    params_slam_in = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  # Manualy override lidar_config_indoor_node parameters from parameter file
  params_slam_in['use_sim_time'] = LaunchConfiguration("use_sim_time")
  params_slam_in['slam.verbosity'] = 0

  slam_indoor_node = Node(name="lidar_slam", package="lidar_slam", executable="lidar_slam_node", output="screen",
    parameters=[params_slam_in],
    condition= UnlessCondition(LaunchConfiguration("outdoor")),
  )

  # Moving base coordinates systems description                                                 tf_FROM_to_TO
  tf_base_to_lidar = Node( package="tf2_ros",executable="static_transform_publisher",name="tf_base_to_lidar",
    #           X    Y    Z    rZ   rY   rX     FROM          TO
    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},],
  )

  # Play bag
  # If comparison is required, end process when the reference is finished
  # If comparison is not required, end process when the bag ends
  rosbag_action_req = ExecuteProcess(
    name='test_rosbag',
    cmd=['ros2', 'bag', 'play', LaunchConfiguration("test_data"), '--clock', '-d', LaunchConfiguration("wait_init")],
    output= 'screen',
    log_cmd= True,
    condition=LaunchConfigurationEquals('ref_path', ''),
    on_exit= Shutdown(),
  )

  rosbag_action_not_req = ExecuteProcess(
    name='test_rosbag',
    cmd=['ros2', 'bag', 'play', LaunchConfiguration("test_data"), '--clock', '-d', LaunchConfiguration("wait_init")],
    output= 'screen',
    log_cmd= True,
    condition=LaunchConfigurationNotEquals('ref_path', ''),
  )

  ld.add_action(velodyne_pcl_node)
  ld.add_action(velodyne_conversion_node)
  ld.add_action(lidar_slam_test_req)
  ld.add_action(lidar_slam_test_not_req)
  ld.add_action(slam_outdoor_node)
  ld.add_action(slam_indoor_node)
  ld.add_action(tf_base_to_lidar)
  ld.add_action(rosbag_action_req)
  ld.add_action(rosbag_action_not_req)

  return (ld)
