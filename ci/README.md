# Continuous Integration
- [Continuous Integration Coverage](#continuous-integration-coverage)
- [CI in a local environment](#continuous-integration-in-a-local-environnement)
  - [Setup a local runner](#setup-a-local-runner)
  - [About Windows](#about-windows)
- [CI ROS in a special environment](#ros-environment)
  - [Docker image](#docker-image)
  - [Setup a Runner](#register-a-new-docker-runner)
  - [ROS test](#ros-test)

## Continuous Integration Coverage

List of domains covered by CI :
- SuperBuild build and usage
- Slam library build
- ROS wrapping build
- ROS wrapping execution
- Paraview wrapping build

## Continuous Integration in a local environnement

Most of the tests are done on a local system except ROS tests. The list includes :
- SuperBuild build and usage
- Slam library build
- Paraview wrapping build

Tests are conducted on both Linux and Windows systems.
The Slam library and the wrapping require a lot of dependencies, which are resolved using a Superbuild that builds dependencies on the GitLab runner system.

### Setup a local runner

- [Register](https://docs.gitlab.com/runner/register/) a new GitLab runner.
- Ensure all the SLAM dependencies (the [core SLAM lib's](../README.md#dependencies), the [ROS wrapping's](../README.md#dependencies-1) and/or the [Paraview wrapping's](../README.md#dependencies-2)) are installed on it. You can partially use the [SuperBuild](https://gitlab.kitware.com/keu-computervision/slam-superbuild) to install them.

  _**NOTE**: Be careful with system dependencies. If using them, one must be sure that they do not conflict with other dependencies._

- If all these dependencies are not installed system-wide or not visible by CMake, it is necessary to indicate where to find them on the runner. To do so, the following variables can be optionally defined in the `environment` field of the `[[runners]]` section, defined in the `config.toml` file. These variables are CMake options that will be forwarded for the configuration step. For example, on a Linux runner, we can set:

```
environment = ["slam_cmake_option_Eigen_DIR=-DEIGEN3_INCLUDE_DIR=/home/ci/deps/install/include/eigen3",
               "slam_cmake_option_Ceres_DIR=-DCeres_DIR=/home/ci/deps/install/lib/cmake/Ceres",
               "slam_cmake_option_nanoflann_DIR=-Dnanoflann_DIR=/home/ci/deps/install/lib/cmake/nanoflann",
               "slam_cmake_option_PCL_DIR=-DPCL_DIR=/home/ci/deps/install/share/pcl-1.10",
               "slam_cmake_option_g2o_DIR=-Dg2o_DIR=/home/ci/deps/install/lib/cmake/g2o",
               "slam_cmake_option_ParaView_DIR=-DParaView_DIR=/home/ci/deps/build/paraview",
               "slam_cmake_option_Boost_DIR=-DBOOST_ROOT=/home/ci/deps/install",
               "slam_cmake_option_Qt5_DIR=-DQt5_DIR=/home/ci/deps/install/lib/cmake/Qt5"]
```

**Note** : The jobs in the stage "superbuild" build a superbuild that can be used to resolve the dependencies.
First, one must build the superbuild by launching manually the corresponding job : **windows_build_superbuild** for windows and **linux_build_superbuild** for linux.
This new superbuild will be used to build the slam library.
Eventually, the superbuild dependencies can be used in the gitlab config.toml. as described above

### About Windows

As Paraview must currently be built with MSVC 2015, this current CI script imposes to use MSVC 2015 for all jobs, for sake of coherence. However, the Core Lidar SLAM lib should build with recent MSVC versions.

## ROS environment

ROS build and tests are run in a Docker container to avoid encountering some issues linked to system libraries or other local dependencies conflict with ROS environment.

### Docker image

The [Dockerfile](ros1_noetic/Dockerfile) file defines the image used for a Linux runner with necessary dependencies to build the SLAM and the ROS wrapping.

The image is stored in gitlab at https://gitlab.kitware.com/keu-computervision/slam/container_registry/223 as ros1_noetic.

The commands to update the image are :
```
docker build path_to_dockerfile -t gitlab.kitware.com:4567/keu-computervision/slam:ros1_noetic
docker push gitlab.kitware.com:4567/keu-computervision/slam:ros1_noetic
```

**WARNING** The docker image use data called **test1** and **test2** from wheezy /Vision/Data/test_SLAM, these data need to be downloades into the folder **ci/ros1_noetic**

### Register a new docker runner

To register a new docker runner:

- Install [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-repository)
- Get the image either from the [Package/Container Registery](../../container_registry), build it from the [Dockerfile](Dockerfile) or from an available image on the [Docker Hub](https://hub.docker.com/search?q=&type=image) .
- Change the runner [docker pull policy](https://docs.gitlab.com/runner/executors/docker.html#using-the-if-not-present-pull-policy) to `if-not-present`. This will enable to use the local image you just get. To do so, open your [runner configuration file](https://docs.gitlab.com/runner/configuration/advanced-configuration.html).


### ROS test

Architecture choices :
- The test data are included in the image using the COPY command
- Build data are stored in cache after build job for next test job
- When the pipeline is triggered for master branch, the reference log data are stored in an artifact available in the pipeline and from the web interface
- When the pipeline is triggered on another branch, the test log data is computed and stored in an artifact. Subsequently, the master log artifact is downloaded, and the pipeline log is compared to the master log

Therefore, these are the detailed steps to add a test :
1. Add a *testk* (for test #**k**) folder to the wheezy  /Vision/Data/test_SLAM folder containing the new bag file on which to test the SLAM
2. Copy this file inside of /ci/ros1_noetic folder in local
4. Add a COPY line to the Dockerfile for these new test data
5. Build the new docker image and push it using the above command (see [Docker image](#docker-image) subsection)
6. Add a make_reference job with the *testk* variable and its specific parameters (vlp16, outdoor...)
7. Add a test job with the same config as the one used in reference creation
