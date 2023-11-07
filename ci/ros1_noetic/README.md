
## Create image on gitlab registry

To create a container image on gitlab, you first need to build the image locally and then you can push it on the remote repository.

Example to create an image in the Slam repository under the name slam/my_image
```bash
docker build . -t gitlab.kitware.com:4567/keu-computervision/slam:my_image
docker push gitlab.kitware.com:4567/keu-computervision/slam:my_image
```

Now, you can use the image in the CI by adding the image name to your job:
```yaml
my_job:
  image: gitlab.kitware.com:4567/keu-computervision/slam:my_image
```


## Display Slam output with Docker and ROS

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