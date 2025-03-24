![SLAM in LidarView](doc/paraview_plugin.png)

![SLAM in ROS](doc/ros_node.png)

# LiDAR SLAM

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
  - [ROS2 wrapping on Linux](#ros2-wrapping-on-linux)
    - [Dependencies](#dependencies-2)
    - [Installation](#installation-2)
      - [With system dependencies](#with-system-dependencies-2)
      - [With local dependencies](#with-local-dependencies-2)
      - [With Superbuild](#with-superbuild-2)
    - [Live usage](#live-usage-1)
  - [ROS2 wrapping on Windows 10](#ros2-wrapping-on-windows-10)
    - [Install dependencies](#install-dependencies)
      - [Install ROS2 and slam dependencies](#install-ros2-and-slam-dependencies)
      - [Install pcl-conversions](#install-pcl-conversions)
    - [Install SLAM package](#install-slam-package)
  - [ParaView wrapping](#paraview-wrapping)
    - [Dependencies](#dependencies-3)
    - [Installation](#installation-3)
    - [Usage](#usage)
    - [Use SLAM in LidarView](#use-slam-in-lidarview)

## Introduction and contents

This repo contains LiDAR-only visual SLAM developped by Kitware, as well as ROS and ParaView wrappings for easier use.

It has been successfully tested on data from several common LiDAR sensors:
- Velodyne (VLP-16, VLP-32c, HDL-32, HDL-64, VLS-128)
- Ouster (OS0/1/2-32/64/128)
- RoboSense (RS-LiDAR-16 RS-LiDAR-32)
- Hesai (PandarXT16, PandarXT32, Pandar128)

Have a look at our [SLAM demo video](https://vimeo.com/524848891)!

This codebase is under active development. If you're interested by new features, new sensors' support or any project that could be using this SLAM, do not hesitate to contact us at kitware@kitware.com.

Repo contents :
- `slam_lib/` : core *LidarSlam* library containing SLAM algorithm and other utilities.
- `superbuild/` : Cross-platform installer.
- `ros_wrapping/` : ROS packages to enable SLAM use on a ROS system.
- `ros2_wrapping/`: ROS2 packages to enable SLAM use on a ROS2 system.
- `paraview_wrapping/` : ParaView plugin to enable SLAM use with ParaView/LidarView.
- `ci/` : continuous integration files to automatically build and check *LidarSlam* lib.
- `CMakeLists.txt` : *CMakeLists* used to call to build core *LidarSlam* lib and *paraview_wrapping*.

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

(*) optional dependencies :

- If G2O is not available (or disabled), *LidarSlam* lib will still be compiled, but without pose graph optimization features.
- If GTSAM is not available (or disabled), *LidarSlam* lib will still be compiled, but without IMU processing features.
- If OpenCV is not available (or disabled), *LidarSlam* lib will still be compiled, but without camera features.
- If OpenMP is available, it is possible to use multi-threading to run some SLAM steps in parallel and achieve higher processing speed.
- If OpenCV is not available (or disabled), *LidarSlam* lib will still be compiled, but without camera integration.

**/!\ Warning** Make sure to compile/install G2O with the same Ceres version as the one used in the SLAM compilation. To do so, disable the feature [G2O_USE_VENDORED_CERES](https://github.com/RainerKuemmerle/g2o/blob/master/CMakeLists.txt) during G2O compilation and link against the right version of Ceres.

### Installation

The *LidarSlam* lib has been tested on Linux, Windows and OS X.

First, got to your workspace directory and clone the SLAM repository.
```bash
git clone https://gitlab.kitware.com/keu-computervision/slam.git src --recursive
```

#### With system dependencies

To build only *LidarSlam* lib using your system dependencies, run :

```bash
cmake -E make_directory build && cd build
cmake ../src -DCMAKE_BUILD_TYPE=Release
cmake --build . -j
```

**NOTE** : On Windows, if some dependencies were installed using vcpkg, the variable `CMAKE_TOOLCHAIN_FILE` must be specified :
cmake ../src -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=[vcpkg-install]/scripts/buildsystems/vcpkg.cmake

#### With local dependencies

You can link to the local libraries you have installed adding cmake flags. Notably with Ceres and G2O:
```bash
cmake ../src -DCMAKE_BUILD_TYPE=Release -DCeres_DIR=path/to/CeresConfig.cmake -Dg2o_DIR=path/to/g2oConfig.cmake
```

#### With Superbuild

In your workspace, run :

```bash
cmake -E make_directory build && cd build
cmake ../src/slam-superbuild -DCMAKE_BUILD_TYPE=Release
cmake --build . -j
```

**NOTE**: By default in the superbuild, mandatory dependencies are installed but optional dependencies are not. You can decide which dependencies to install with the superbuild using the options **INSTALL_XX**. For example, to not build *PCL*:
```bash
cmake ../src/slam-superbuild -DCMAKE_BUILD_TYPE=Release -DINSTALL_PCL=OFF
```

Note that installing and enabling an optional dependency is not the same. If you want to install and enable the use of an optional dependency you need to switch two variables to ON : **INSTALL_XX** and **ENABLE_XX**.

Example for *GTSAM* :
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

For Velodyne usage, please note that the ROS Velodyne driver with minimum version 1.6 is needed.
Be careful, this ROS Velodyne driver 1.6 is not backward-compatible with previous versions.
If you're running on Ubuntu 20 / ROS Noetic, you can install the new Velodyne driver using the command `sudo apt install ros-$ROS_DISTRO-velodyne ros-$ROS_DISTRO-velodyne-pcl`.
If running on previous versions of Ubuntu/ROS (18/Melodic and below), you need to compile this driver from source : just clone the [git repo](https://github.com/ros-drivers/velodyne) in your catkin worskpace sources, it will be automatically built with next  `catkin_make` or `catkin build`.

For Ouster usage, the driver can be found in this [git repo](https://github.com/ouster-lidar/ouster_example)

### Installation

Clone this git repo directly into your catkin workspace (called **catkin_ws** in the following), under the src directory

 ```bash
 cmake -E make_directory catkin_ws && cd catkin_ws
 git clone https://gitlab.kitware.com/keu-computervision/slam.git src/slam --recursive
```

The next sections describe how to install the dependencies (mandatory and/or optional) and to build the SLAM packages with the needed features.
The first step is to install pcl-ros and geodesy before performing any of the next sections. Example with apt : `sudo apt-get install -y ros-$ROS_VERSION-pcl-ros ros-$ROS_VERSION-geodesy`

**NOTE** : Boost, g2o, Eigen, Ceres and PCL should be already resolved at this point.

#### With system dependencies

This applies if you have installed all the dependencies on your system, e.g. when you download binaries (with apt or other).

**NOTE** : The only mandatory missing dependency at this point should be nanoflann. Example to install it with apt : `sudo apt-get install -y libnanoflann-dev`.

Run `catkin_make --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` or `catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release` (to turn on optimizations, highly recommended when using Eigen). The same can be done with `catkin build`. By default, this will build *LidarSlam* lib before ROS packages. If you want to use your system LidarSlam, you need to set the cmake variable BUILD_SLAM_LIB to OFF : `catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SLAM_LIB=OFF`

#### With local dependencies

This applies if you have built and installed some of the dependencies locally on your disk.
You can use local dependencies for Slam lib by passing their path to cmake.

_Example_ for Ceres and g2o :
 ```bash
 catkin_make -j --cmake-args -DCMAKE_BUILD_TYPE=Release  -DCeres_DIR=path/to/CeresConfig.cmake -Dg2o_DIR=path/to/g2oConfig.cmake
  OR
 catkin build -j --cmake-args -DCMAKE_BUILD_TYPE=Release -DCeres_DIR=path/to/CeresConfig.cmake -Dg2o_DIR=path/to/g2oConfig.cmake
 ```

**Advanced**: If you want to use a local version of LidarSlam library you can specify to the **lidar_slam** package not to build it and supply the path to the LidarSlam cmake file :

 ```bash
 catkin build -j --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SLAM_LIB=OFF -DLidarSlam_DIR=path/to/LidarSlam.cmake
```

#### With Superbuild

This applies if you have some missing dependencies and you don't want any of the previous solutions. The [superbuild](https://gitlab.kitware.com/keu-computervision/slam-superbuild/) that is provided allows to download, build and install them locally so it can be used to build the SLAM packages afterwards. All the mandatory and optional dependencies can be installed by the superbuild. One can choose the one he wants to install with the variables **INSTALL_XX** (example with PCL below).

**WARNING** It is not possible to use PCL from the superbuild (this would create runtime issues with system version).

**WARNING** The superbuild must be installed outside of catkin workspace.

Example in parent directory of your catkin workspace (e.g. catkin_ws/..) :
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

**Advanced** : The default behavior is that the ROS wrapping builds the SLAM library, but the superbuild can also install the SLAM library. It is possible to use the superbuild one by setting the BUILD_SLAM_SHARED_LIB variable to ON in superbuild build and BUILD_SLAM_LIB to OFF in ROS wrapping build.

_Example_ :
 ```bash
 cmake -E make_directory SB-build && cd SB-build
 cmake ../catkin_ws/src/slam/slam-superbuild -GNinja -DCMAKE_BUILD_TYPE=Release -DINSTALL_PCL=OFF -DBUILD_SLAM_SHARED_LIB=ON
 cmake --build . -j
 cd ../catkin_ws
 catkin build -j --cmake-args -DCMAKE_BUILD_TYPE=Release -DSUPERBUILD_INSTALL_DIR=absolute/path/to/SB-build/install -DBUILD_SLAM_LIB=OFF
```

### Live usage

For Velodyne :
```bash
roslaunch lidar_slam slam_velodyne.launch use_sim_time:=false
roslaunch lidar_slam slam_velodyne.launch use_sim_time:=false gps:=true   # if GPS/SLAM calibration has to be run
```

For Ouster :
```bash
roslaunch lidar_slam slam_ouster.launch replay:=false
roslaunch lidar_slam slam_ouster.launch replay:=false gps:=true   # if GPS/SLAM calibration has to be run
```

See [ros_wrapping/lidar_slam/README.md](ros_wrapping/lidar_slam/README.md) for more details.

## ROS2 wrapping on Linux

You can download the ROS2 package in [the artifacts of the CI pipeline](https://gitlab.kitware.com/keu-computervision/slam/-/pipelines) for any version.

A docker image is available on [this link](https://send.kitware.io/download.php?id=2329&token=yyoAxvfKEYECXOhxedzjkFMSVzkhKI66). It can also be found in [the artifacts of the feat/ROS2 branch](https://gitlab.kitware.com/keu-computervision/slam/-/pipelines?page=1&scope=all&source=web&ref=feat%2FROS2) if it has been triggered manually.
You can load and run this image with :

```
docker load -i slam_image.tar
docker run -it --rm slam:<CURRENT_COMMIT> bash
```

replacing CURRENT_COMMIT with the image commit sha.

### Dependencies

Ensure all *LidarSlam* dependencies are respected (see next sections to do so). Specific ROS packages dependencies are listed in the table below along with the version used during development and testing.

| Dependency      | Tested Versions | Install (`sudo apt-get install <pkg>`)                                             | status    |
|:---------------:|:---------------:|:----------------------------------------------------------------------------------:|:---------:|
| ROS             | humble/iron     | `ros-$ROS_DISTRO-desktop-full`                                                     | mandatory |
| pcl-ros         | 1.7.4           | `ros-$ROS_DISTRO-pcl-ros`                                                          | mandatory |
| gps_common      | 0.3.0           | `ros-$ROS_DISTRO-gps-common`                                                       | optional  |
| apriltag        | 3.2.0           | `ros-$ROS_DISTRO-apriltag`                                                         | optional  |
| g2o             | 5.3             | `ros-$ROS_DISTRO-libg2o`                                                           | optional  |


For Velodyne usage, please note that the ROS Velodyne driver with minimum version 1.6 is needed.
Be careful, this ROS Velodyne driver 1.6 is not backward-compatible with previous versions.
You can install the new Velodyne driver using the command `sudo apt install ros-$ROS_DISTRO-velodyne`.

For Ouster usage, the driver can be found in this [git repo](https://github.com/ouster-lidar/ouster-ros/tree/ros2). We recommand to build it on another workspace but you can also clone it under the ros2_wrapping folder and build it at the same time as the SLAM.

### Installation

Clone this git repo directly into your colcon workspace (called **colcon_ws** in the following), under the src directory

 ```bash
 cmake -E make_directory colcon_ws && cd colcon_ws
 git clone https://gitlab.kitware.com/keu-computervision/slam.git src/slam --recursive -b feat/ROS2
```

The next sections describe how to install the dependencies (mandatory and/or optional) and to build the SLAM packages with the needed features.
The first step is to install pcl-ros before performing any of the next sections. Example with apt : `sudo apt-get install -y ros-$ROS_VERSION-pcl-ros`

**NOTE** : Boost, g2o, Eigen, Ceres and PCL should be already resolved at this point with previous installations.

#### With system dependencies

This applies if you have installed all the dependencies on your system, e.g. when you download binaries (with apt or other).

**NOTE** : The only mandatory missing dependency should be nanoflann at this point. Example to install it with apt : `sudo apt-get install -y libnanoflann-dev`.

Run `colcon build --base-paths src/slam/ros2_wrapping` or `colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release` (to turn on optimizations, highly recommended when using Eigen). the variable base_paths must point to the path of the ros2_wrapping folder.
By default, this will build *LidarSlam* lib before ROS2 packages. If you want to use your system LidarSlam, you need to set the cmake variable BUILD_SLAM_LIB to OFF :
`colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SLAM_LIB=OFF`

#### With local dependencies

This applies if you have built and installed some of the dependencies locally on your disk.
You can use local dependencies for Slam lib by passing their path to cmake.

_Example_ for Ceres and g2o :

 ```bash
 colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release -DCeres_DIR=path/to/CeresConfig.cmake -Dg2o_DIR=path/to/g2oConfig.cmake
 ```

If you want to use a local version of LidarSlam library you can specify to the package not to build it and supply the path to the LidarSlam cmake file :

 ```bash
 colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SLAM_LIB=OFF -DLidarSlam_DIR=path/to/LidarSlam.cmake
```

#### With Superbuild

This applies if you have some missing dependencies and you don't want any of the previous solutions. The [superbuild](https://gitlab.kitware.com/keu-computervision/slam-superbuild/) that is provided allows to download, build and install them locally so it can be used to build the SLAM packages afterwards. All the mandatory AND optional dependencies can be installed by the superbuild. One can choose the one he wants to install with the variables **INSTALL_XX** (example with PCL below).

**WARNING** It is not possible to use PCL from the superbuild (this would create runtime issues with system version).

**WARNING** The superbuild must be installed outside of catkin workspace.

_Full installation with superbuild example_ :
 ```bash
 # Clone project
 git clone https://gitlab.kitware.com/keu-computervision/slam.git ros2_ws/src/slam --recursive
 # Build Superbuild to install dependencies locally
 cmake -E make_directory SB-build && cd SB-build
 cmake ../ros2_ws/src/slam/slam-superbuild -GNinja -DCMAKE_BUILD_TYPE=Release -DINSTALL_PCL=OFF
 cmake --build . -j
 # Build Slam ROS package pointing to the superbuild install directory
 cd ../ros2_ws
 call path\to\ros2_humble\local_setup.bat
 colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release -DSUPERBUILD_INSTALL_DIR=absolute/path/to/SB-build/install
```

The default behavior is that the ROS2 wrapping builds the SLAM library, but the superbuild can also install the SLAM library.
It is possible to use the superbuild one by setting the BUILD_SLAM_SHARED_LIB variable to ON in superbuild build and BUILD_SLAM_LIB to OFF in ROS wrapping build.

Example :
 ```bash
 mkdir SB-build && cd SB-build
 cmake ../ws_ros2/src/slam/slam-superbuild -GNinja -DCMAKE_BUILD_TYPE=Release -DINSTALL_PCL=OFF -DBUILD_SLAM_SHARED_LIB=ON
 cmake --build . -j
 cd ../ws_ros2
 colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release -DSUPERBUILD_INSTALL_DIR=absolute/path/to/SB-build/install -DBUILD_SLAM_LIB=OFF
```

### Live usage

For Velodyne :
```bash
ros2 launch lidar_slam slam_velodyne.launch.py use_sim_time:=false
ros2 lidar_slam slam_velodyne.launch.py use_sim_time:=false gps:=true   # if GPS/SLAM calibration has to be run
```

For Ouster :
```bash
ros2 launch lidar_slam slam_ouster.launch.py replay:=false
ros2 launch lidar_slam slam_ouster.launch.py replay:=false gps:=true   # if GPS/SLAM calibration has to be run
```

See [ros2_wrapping/lidar_slam/README.md](ros2_wrapping/lidar_slam/README.md) for more details.

## ROS2 wrapping on Windows 10

This wrapping has been tested on Humble version of ROS2.

**WARNINGS** :
  - ROS2 is supported on Windows 10 but many packages are not ported and the installation can be tricky.
  - The slam_visualization plugin is not available on Windows


**NOTE** : Always use administrator rights x64_x86 Cross Tools Command Prompt for VS 2019

### Install dependencies

#### Install ROS2 and slam dependencies
Install ROS2 humble version by following [official instructions](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)

Install slam dependencies with the [SLAM superbuild](https://gitlab.kitware.com/keu-computervision/slam-superbuild) on Release mode. You can use MSVC or Ninja as generator.
The instructions can be found in the section [above](#with-superbuild).

To use dependencies installed on the superbuild, you need to set the PATH env
```
set PATH=path\to\Slam_SB\install\bin;%PATH%
```

#### Install pcl-conversions
Create a workspace for pcl-conversions package
Be sure to have PCL in the PATH env

```bash
cmake -E make_directory ws_pcl/src && cd ws_pcl\src
git clone https://github.com/ros-perception/perception_pcl.git -b ros2
git clone https://github.com/ros-perception/pcl_msgs.git -b ros2
cd ..
call path\to\ros2_humble\local_setup.bat
colcon build --merge-install --packages-up-to pcl_conversions \
             --cmake-args -DCMAKE_BUILD_TYPE=Release \
                          -DBoost_DIR=path\to\SB-build\install\lib\cmake\Boost-1.76.0 \
                          -DBOOST_ROOT=path\to\SB-build\install \
                          -DPCL_DIR=path\to\SB-build\install\share\pcl-1.13 \
                          -DBoost_USE_STATIC_LIBS=OFF
```
To use pcl-conversions package, you need to source your command prompt
```bash
call path\to\ws_pcl\install\setup.bat
```

### Install SLAM package
Be sure to have the superbuild in the PATH and pcl_conversions sourced
```bash
cmake -E make_directory ws_ros2
dir ws_ros2
git clone https://gitlab.kitware.com/keu-computervision/slam -b feat/ROS2
call path\to\ros2_humble\local_setup.bat
colcon build --base-paths=src\slam\ros2_wrapping  --merge-install --cmake-args  -DCMAKE_BUILD_TYPE=Release -DENABLE_OpenCV=OFF -DSUPERBUILD_INSTALL_DIR=absolute/path/to/Slam_SB/install
```
source the SLAM
```
call install/setup.bat
```
The SLAM is ready to use. Please make sure that the superbuild bin path is in the PATH var and pcl_conversions is sourced.

## ParaView wrapping

The *LidarSlamPlugin* Paraview wrapping has been tested on Linux, Windows and OS X.

### Dependencies

Ensure all *LidarSlam* dependencies are respected. Specific dependencies are listed in the table below along with the version used during development and testing.

| Dependency | Tested Version    |
| :--------: | :------------:    |
| ParaView   | 5.11.0            |

Be careful to use and link to the same libraries as ParaView/LidarView's (especially with VTK, Eigen, PCL, Ceres, nanoflann, etc.). Otherwise, if different flags or modules were enabled, some troubles may arise at build time, or it could lead to version mismatch and segfault at runtime.

For example, if PCL is built with `pcl_visualization` module, it must link to the same VTK than the one used by ParaView.

### Installation

To build *LidarSlam* lib and this ParaView plugin *LidarSlamPlugin*, just `cd` to this repo root dir and run :

```bash
cmake -E make_directory build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSLAM_PARAVIEW_PLUGIN:BOOL=ON
cmake --build . -j
```

Set XX_DIR variables to the same path dependencies of ParaView/LidarView when they exist (Eigen, Ceres, nanoflann, etc...).

### Usage

- Open ParaView
- **Tools** tab > **Manage Plugins** > **Load New**
- Browse to the `<install>/lib/` dir and select `libLidarSlamPlugin.so` or `LidarSlamPlugin.dll`
- Load LiDAR frames and LiDAR calibration to use
- Select the frames in the Pipeline Browser, instantiate a SLAM filter, and apply it.

Currently, all features are not available in ParaView plugin. Features such as GPS/LiDAR calibration, pose graph optimization or temporal logging are only supported in ROS wrapping. However, ParaView plugin is useful to play with SLAM, interactively try out parameters, visualize and export results.

### Use SLAM in LidarView

This *LidarSlamPlugin* is natively included in [LidarView](https://lidarview.kitware.com/). For more detailed information on how to enable and use SLAM filter in LidarView, see [paraview_wrapping/Plugin/doc/How_to_SLAM_with_LidarView.md](paraview_wrapping/Plugin/doc/How_to_SLAM_with_LidarView.md).

Pre-built binaries of LidarView with this SLAM plugin are available for download [here](https://gitlab.kitware.com/LidarView/lidarview/-/releases).