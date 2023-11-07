FROM osrf/ros:noetic-desktop-full
LABEL maintainer="Arthur Bourbousson arthur.bourbousson@kitware.com"

WORKDIR /tmp

# Install Slam ROS dependencies
RUN apt update && \
    apt install -y ros-$ROS_DISTRO-geodesy ros-$ROS_DISTRO-gps-common ros-$ROS_DISTRO-velodyne-pointcloud

# Install Slam dependencies
COPY install_deps.sh install_deps.sh
RUN sh install_deps.sh

COPY install_ceres.sh install_ceres.sh
RUN sh install_ceres.sh

COPY install_nanoflann.sh install_nanoflann.sh
RUN sh install_nanoflann.sh

COPY install_g2o.sh install_g2o.sh
RUN sh install_g2o.sh

# Install gitlab CI utilities
RUN apt install -y jq unzip

# Graphic display on host
RUN apt -y install libgl1-mesa-glx libgl1-mesa-dri

# Erase apt cache
RUN rm -rf /var/lib/apt/lists/*

# Testing data
WORKDIR /usr/src/app
# Dockerfile must be run in a folder containing all data folders (e.g test1, test2...)
RUN mkdir -p ./data/test1
RUN mkdir -p ./data/test2
COPY ./test1/test1.bag ./data/test1
COPY ./test2/test2.bag ./data/test2

WORKDIR /root
