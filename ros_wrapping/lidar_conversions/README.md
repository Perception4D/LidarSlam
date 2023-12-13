# lidar_conversions

## Description

Helper package to convert raw pointclouds output by common LiDAR drivers to the pointcloud format expected by the SLAM algorithm.

The SLAM algorithm expects input pointclouds as *sensor_msgs/PointCloud2* messages. These pointclouds should have the following fields:
- **x**, **y**, **z** (`float`) : point coordinates
- **time** (`double`) : time offset to add to the pointcloud header timestamp to get approximate point-wise acquisition timestamp
- **intensity** (`float`) : intensity/reflectivity of the point
- **laser_id** (`uint16`) : numeric identifier of the laser ring that shot this point. The lowest/bottom laser ring should be 0, and it should increase upward.
- **label** (`uint8`) : optional input, not yet used.

Especially, the point-wise timestamps are necessary if undistortion is enabled in SLAM. The nodes of this package are able to compute approximate timestamps when those are not available in the input pointcloud.

SLAM expects that the lowest/bottom laser ring is 0, and is increasing upward. If this is not your case, you can use the `laser_id_mapping` to correct it in the output cloud.

Currently, this package implements the following nodes :
- **velodyne_conversion_node** : converts pointclouds output by Velodyne spinning sensors using the [ROS Velodyne driver](https://github.com/ros-drivers/velodyne) to SLAM pointcloud format.
- **robosense_conversion_node** : converts pointclouds output by RoboSense spinning sensors using the [ROS RoboSense-LiDAR driver](https://github.com/RoboSense-LiDAR/ros_rslidar) to SLAM pointcloud format. This has been tested only with RS16 sensor, and could need additional changes to support other RS sensors.
- **ouster_conversion_node** : converts pointclouds output by Ouster spinning sensors using the [ROS Ouster driver at this version](https://github.com/ouster-lidar/ouster-ros/tree/3f01e1d7001d8d21ac984566d17505b98905fa86) to SLAM pointcloud format.
- **livox_conversion_node** : converts custom pointclouds output by Livox spinning sensors using the [ROS Livox driver](https://github.com/Livox-SDK/livox_ros_driver) to SLAM pointcloud format.
- **hesai_conversion_node** : converts pointclouds output by Hesai spinning sensors using the [ROS Hesai driver](https://github.com/HesaiTechnology/HesaiLidar_General_ROS/) to SLAM pointcloud format.
- **generic_conversion_node** : converts any PointCloud2 to SLAM pointcloud format. It can recognize multiple names and types for the fields of PointCloud2, and if some are missing (like laser_id or time), it recomputes them using space coordinates only.
As it looks for multiple types/names, if the user gives a complete PointCloud2, the average conversion time will be doubled compared to using a specific conversion node. We advise to use one the nodes above if the user is certain of his PointCloud2 (types and names of each field included).
If the user is unable to provide all the necessary fields required for SLAM, we advise to use this node. It will automatically compute the missing information.

## Usage

### Direct usage

Example for velodyne :

```bash
rosrun lidar_conversions velodyne_conversion_node
```

### Direct usage with user invention to compute estimation parameters again (in case laser_id and/or time look unusual)
To be done in two seperate terminals:

1:
```bash
rosrun lidar_conversions generic_conversion_node
```

2:
- if you want to compute estimation parameters again for any other generic conversion node :
```bash
rosservice call lidar_conversions/estim_params lidar_conversions/srv/EstimParams
```

### Example of launchfile for a multi-lidar setup:

```xml
<launch>
  <!-- LiDAR pointclouds conversions.
       The 'rpm' and 'timestamp_first_packet' parameters are only used to
       generate approximate point-wise timestamps if 'time' field is not usable.
       These 2 parameters should be set to the same values as ROS Velodyne/RSLidar drivers'. -->

  <node name="velodyne_conversion" pkg="lidar_conversions" type="velodyne_conversion_node" output="screen">
    <param name="rpm" value="600."/>
    <param name="timestamp_first_packet" value="false"/>
    <remap from="lidar_points" to="velodyne_lidar_points"/>
  </node>

  <node name="robosense_conversion" pkg="lidar_conversions" type="robosense_conversion_node" output="screen">
    <rosparam param="laser_id_mapping">[0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8]</rosparam>
    <param name="rpm" value="600."/>
    <remap from="lidar_points" to="robosense_lidar_points"/>
  </node>
</launch>
```