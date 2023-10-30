
# Display Slam ROS with Docker

#### Create network
sudo docker network create ros_net

### Launch roscore on network
sudo docker run --rm -it \
  --net ros_net \
  --name roscore_cont \
  ros_noetic roscore

### Launch Slam node with display options

#### Create Slam container
sudo docker run -it \
  --net ros_net \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  -e ROS_MASTER_URI=http://roscore_cont:11311 \
  -e "DISPLAY=$DISPLAY" \
  --name slam_cont \
  ros_noetic

#### Launch slam
cmake -E make_directory catkin_ws && cd catkin_ws && \
git clone https://gitlab.kitware.com/keu-computervision/slam.git src/slam --recursive && \
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release && \
source devel/setup.bash && \
roslaunch lidar_slam slam_velodyne.launch

### Launch data

#### Create rosbag container

sudo docker run -it \
  --net ros_net \
  -e ROS_MASTER_URI=http://roscore:11311 \
  --name rosbag_cont \
  ros_noetic

##### In an other terminal
sudo docker cp "/path/to/bag/name.bag" rosbag_cont:/root

##### In the container
rosbag play name.bag -d1 --clock