FROM osrf/ros:humble-desktop-full
LABEL maintainer="Arthur Bourbousson arthur.bourbousson@kitware.com"

WORKDIR /tmp

RUN apt update

# Install Slam ROS dependencies
RUN apt install -y ros-$ROS_DISTRO-velodyne

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
COPY ./test1 ./data/test1
COPY ./test2 ./data/test2

WORKDIR /root