# Kitware LiDAR SLAM

<img src="doc/ros_node.png" width="800">

- [LiDAR SLAM](#lidar-slam)
  - [Introduction and contents](#introduction-and-contents)
  - [Core SLAM lib](#core-slam-lib)
    - [Dependencies](#dependencies)
    - [Installation](#installation)
      - [With system dependencies](#with-system-dependencies)
      - [With local dependencies](#with-local-dependencies)
      - [With Superbuild](#with-superbuild)
  - [ROS wrapping](#ros-wrapping)
    - [Dependencies](#dependencies-1)
    - [Installation](#installation-1)
      - [With system dependencies](#with-system-dependencies-1)
      - [With local dependencies](#with-local-dependencies-1)
      - [With Superbuild](#with-superbuild-1)
    - [Live usage](#live-usage)

# LiDAR SLAM

## Introduction and contents

This repository contains a SLAM system developed by Kitware, primarily based on 3D LiDAR. It also supports external sensors such as INS, IMU, GPS, and cameras to enhance SLAM accurancy. Paraview and ROS1 / ROS2 wrappers are provided for easier integration.  This branch is specifically for the ROS wrapper.

**/!\ Note:  ROS Noetic will reach end-of-life in May 2025, and this branch will no longer be maintained after that date.
For the latest updates on the LiDAR SLAM library, ParaView wrapping, and ROS 2 support, please refer to the [master branch](https://gitlab.kitware.com/keu-computervision/slam).** 

If you're interested by new features, new sensors' support or any project that could be using this SLAM, do not hesitate to contact us at kitware@kitware.fr.

Our SLAM algorithm has been successfully tested on data from several common LiDAR sensors:
- Velodyne (VLP-16, VLP-32c, HDL-32, HDL-64, VLS-128)
- Ouster (OS0/1/2-32/64/128)
- RoboSense (RS-LiDAR-16 RS-LiDAR-32)
- Hesai (PandarXT16, PandarXT32, Pandar128)
- Livox (mid 360)

Have a look at our [SLAM demo video](https://vimeo.com/524848891)!

Repo contents:
- `slam_lib/`: core *LidarSlam* library containing SLAM algorithm and other utilities.
- `superbuild/`: Cross-platform installer.
- `ros_wrapping/`: ROS packages to enable SLAM use on a ROS system.
- `ci/`: continuous integration files to automatically build and check *LidarSlam* lib.
- `CMakeLists.txt`: *CMakeLists* used to call to build core *LidarSlam* lib and *paraview_wrapping*.

## Core SLAM lib

### Dependencies

Dependencies are listed in the table below along with the version used during development and testing. Minimum required versions have not been determined yet.

| Dependency | Minimum tested Version |
| :--------: | :--------------------: |
| Eigen3     | 3.3.4                  |
| Ceres      | 1.13.0                 |
| PCL        | 1.8                    |
| nanoflann  | 1.3.0                  |
| g2o*       | 1.0.0 (master)         |
| OpenMP*    | 2.0                    |
| gtsam*     | 4.2a8                  |
| OpenCV*    | 4.5.4                  |
| TEASER++*  | 2.0                    |  

(*) optional dependencies:

- If G2O is not available (or disabled), *LidarSlam* lib will still be compiled, but without pose graph optimization features.
- If GTSAM is not available (or disabled), *LidarSlam* lib will still be compiled, but without IMU processing features.
- If OpenCV is not available (or disabled), *LidarSlam* lib will still be compiled, but without camera features.
- If TEASER++ is not available (or disabled), *LidarSlam* lib will still be compiled, but without loop closure detection features. Additionally, the loop closure detection feature is not supported in the ROS wrapper.
- If OpenMP is available, it is possible to use multi-threading to run some SLAM steps in parallel and achieve higher processing speed.

**/!\ Warning:** Make sure to compile/install G2O with the same Ceres version as the one used in the SLAM compilation. To do so, disable the feature [G2O_USE_VENDORED_CERES](https://github.com/RainerKuemmerle/g2o/blob/master/CMakeLists.txt) during G2O compilation and link against the right version of Ceres.

### Installation

The *LidarSlam* lib has been tested on Linux, Windows and OS X.

First, go to your workspace directory and clone the SLAM repository.
```bash
git clone https://gitlab.kitware.com/keu-computervision/slam.git src --recursive -b ROS1/SlamRos1Wrapping
```

#### With system dependencies

To build only *LidarSlam* lib using your system dependencies, run:

```bash
cmake -E make_directory build && cd build
cmake ../src -DCMAKE_BUILD_TYPE=Release
cmake --build . -j
```

**NOTE:** On Windows, if some dependencies were installed using vcpkg, the variable `CMAKE_TOOLCHAIN_FILE` must be specified:
```
cmake ../src -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=[vcpkg-install]/scripts/buildsystems/vcpkg.cmake
```

#### With local dependencies

You can link to the local libraries you have installed by adding cmake flags. Notably with Ceres and G2O:
```bash
cmake ../src -DCMAKE_BUILD_TYPE=Release -DCeres_DIR=path/to/CeresConfig.cmake -Dg2o_DIR=path/to/g2oConfig.cmake
```

#### With Superbuild

In your workspace, run:

```bash
cmake -E make_directory build && cd build
cmake ../src/slam-superbuild -DCMAKE_BUILD_TYPE=Release
cmake --build . -j
```

**NOTE**: By default in the superbuild, mandatory dependencies are installed but optional dependencies are not. You can decide which dependencies to install with the superbuild using the options **INSTALL_XX**. For example, to not build *PCL*:
```bash
cmake ../src/slam-superbuild -DCMAKE_BUILD_TYPE=Release -DINSTALL_PCL=OFF
```

Note that installing and enabling an optional dependency is not the same. If you want to install and enable the use of an optional dependency you need to switch two variables to ON: **INSTALL_XX** and **ENABLE_XX**.

_Example:_ to install and enable *GTSAM*:
```bash
cmake ../src/slam-superbuild -DCMAKE_BUILD_TYPE=Release -DINSTALL_GTSAM=ON -DENABLE_GTSAM=ON
```

More documentation about the superbuild can be found [here](https://gitlab.kitware.com/keu-computervision/slam-superbuild).

## ROS wrapping

The ROS1 wrapping has been tested on Linux only.

### Dependencies

Ensure all *LidarSlam* mandatory dependencies are respected (see next sections to do it). Specific ROS packages' dependencies are listed in the table below along with the version used during development and testing.

| Dependency      | Tested Versions | Install (`sudo apt-get install <pkg>`)                                             | status    |
|:---------------:|:---------------:|:----------------------------------------------------------------------------------:|:---------:|
| ROS             | melodic, noetic | `ros-$ROS_DISTRO-desktop-full` and [tutorial](http://wiki.ros.org/ROS/Installation)| mandatory |
| pcl-ros         | 1.7.4           | `ros-$ROS_DISTRO-pcl-ros`                                                          | mandatory |
| geodesy         | 0.5.3           | `ros-$ROS_DISTRO-geodesy`                                                          | mandatory |
| gps_common      | 0.3.0           | `ros-$ROS_DISTRO-gps-common`                                                       | optional |
| apriltag        | 3.2.0           | `ros-$ROS_DISTRO-apriltag`                                                         | optional |
| g2o             | 5.3             | `ros-$ROS_DISTRO-libg2o`                                                           | optional |

LiDAR drivers may also need to be installed to interpret LiDAR packets and convert them into raw point clouds. Our ROS wrapper includes a helper package that transforms raw point clouds from common LiDAR drivers into the format expected by the SLAM algorithm. For more details, refer to the [ros_wrapping/lidar_conversions/README.md](ros_wrapping/lidar_conversions/README.md) for more details.

_Example:_ velodyne driver installation

The ROS Velodyne driver with minimum version 1.6 is needed. This version is not backward-compatible with previous versions.
- For Ubuntu 20 / ROS Noetic, install the updated Velodyne driver using:
```
sudo apt install ros-$ROS_DISTRO-velodyne ros-$ROS_DISTRO-velodyne-pcl`
```
- For Ubuntu 18 / ROS Melodic and earlier, you need to compile this driver from source.

For installation instructions on different LiDAR drivers, please refer to the official LiDAR drivers repositories and follow their guidelines.

### Installation

Clone this git repo directly into your catkin workspace (referred to **catkin_ws** in the following):
 ```bash
 cmake -E make_directory catkin_ws && cd catkin_ws
 git clone https://gitlab.kitware.com/keu-computervision/slam.git src/slam --recursive -b ROS1/SlamRos1Wrapping
```

The next sections describe how to install the required dependencies (both mandatory and optional) and to build the SLAM packages with the necessary features.

**Prerequisite:** Before proceeding, ensure pcl-ros and geodesy are installed. 
You can install them using apt as follows: 
```
sudo apt-get install -y ros-$ROS_VERSION-pcl-ros ros-$ROS_VERSION-geodesy
```
**NOTE**: Boost, g2o, Eigen, Ceres and PCL should be already resolved at this point.

#### With system dependencies

This method applies if you have installed all required dependencies on your system, such as through package managers like `apt`.

**NOTE**: The only missing dependency at this point should be nanoflann. You can install it using:
```
sudo apt-get install -y libnanoflann-dev
```

Run the following command to build the ROS2 SLAM packages:
```
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
``` 
or for optimized performance (highly recommended when using Eigen), use: 
```
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
```
The same can be done with `catkin build`. 

By default, this will build *LidarSlam* lib before ROS packages. If you want to use your system LidarSlam, you need to set the cmake variable `BUILD_SLAM_LIB` to `OFF`: 
```
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SLAM_LIB=OFF
```

#### With local dependencies

If you have built and installed some dependencies locally, you can specify their paths when building the SLAM library using CMake.

_Example:_ Using local Ceres and g2o:

To use locally installed versions of Ceres and g2o, pass their respective paths to CMake:
 ```bash
 catkin_make -j --cmake-args -DCMAKE_BUILD_TYPE=Release  -DCeres_DIR=path/to/CeresConfig.cmake -Dg2o_DIR=path/to/g2oConfig.cmake
  OR
 catkin build -j --cmake-args -DCMAKE_BUILD_TYPE=Release -DCeres_DIR=path/to/CeresConfig.cmake -Dg2o_DIR=path/to/g2oConfig.cmake
 ```


_Example:_ Using a local version of LidarSlam

If you want to use a local version of LidarSlam library, you can prevent the package from building it and instead provide the path to its CMake configuration file:

 ```bash
 catkin build -j --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SLAM_LIB=OFF -DLidarSlam_DIR=path/to/LidarSlam.cmake
```

#### With Superbuild

If you are missing dependencies and prefer not to manually install them, you can use the [superbuild](https://gitlab.kitware.com/keu-computervision/slam-superbuild/). The superbuild automates the process of downloading, building, and installing all necessary dependencies locally, making it easier to build the SLAM packages afterward. Superbuild supports both mandatory and optional dependencies, and you can select which ones to install using the `INSTALL_XX` variables (e.g., INSTALL_PCL=OFF for PCL).

**WARNING**: You cannot use PCL from the superbuild, as it would cause runtime conflicts with the system version.

**WARNING**: The superbuild must be installed outside of catkin workspace.

_Example:_ Full installation with superbuild

 ```bash
 # Build Superbuild to install the dependencies locally
 cmake -E make_directory SB-build && cd SB-build
 cmake ../catkin_ws/src/slam/slam-superbuild -GNinja -DCMAKE_BUILD_TYPE=Release -DINSTALL_PCL=OFF
 cmake --build . -j
 # Build Slam ROS package using superbuild installed dependencies
 cd ../catkin_ws
 catkin_make -j --cmake-args -DCMAKE_BUILD_TYPE=Release -DSUPERBUILD_INSTALL_DIR=absolute/path/to/SB-build/install
  OR
 catkin build -j --cmake-args -DCMAKE_BUILD_TYPE=Release -DSUPERBUILD_INSTALL_DIR=absolute/path/to/SB-build/install
```

**Advanced**: By default, the ROS wrapping will build the the SLAM library itself. However, if you want the superbuild to handle this instead, you can:
- Enable SLAM library installation in the superbuild (BUILD_SLAM_SHARED_LIB=ON)
- Disable SLAM library building in the ROS2 wrapping (BUILD_SLAM_LIB=OFF)


_Example:_ Using the Superbuild SLAM Library
 ```bash
 cmake -E make_directory SB-build && cd SB-build
 cmake ../catkin_ws/src/slam/slam-superbuild -GNinja -DCMAKE_BUILD_TYPE=Release -DINSTALL_PCL=OFF -DBUILD_SLAM_SHARED_LIB=ON
 cmake --build . -j
 cd ../catkin_ws
 catkin build -j --cmake-args -DCMAKE_BUILD_TYPE=Release -DSUPERBUILD_INSTALL_DIR=absolute/path/to/SB-build/install -DBUILD_SLAM_LIB=OFF
```

### Live usage

For Velodyne:
```bash
roslaunch lidar_slam slam_velodyne.launch use_sim_time:=false
roslaunch lidar_slam slam_velodyne.launch use_sim_time:=false gps:=true   # if GPS/SLAM calibration has to be run
```

For Ouster:
```bash
roslaunch lidar_slam slam_ouster.launch replay:=false
roslaunch lidar_slam slam_ouster.launch replay:=false gps:=true   # if GPS/SLAM calibration has to be run
```

Refer to [ros_wrapping/lidar_slam/README.md](ros_wrapping/lidar_slam/README.md) for detailed instructions on using SLAM features with ROS wrapping.
