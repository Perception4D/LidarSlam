# ROS2 Dockerfile

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

## Connect ROS2 nodes in Docker :

### Create network
sudo docker network create ros_net

### Launch Slam node with display options

#### Create Slam container
sudo docker run -it \
  --net ros_net \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  -e "DISPLAY=$DISPLAY" \
  --name ros2_cont \
  ros_humble


#### Launch slam
cmake -E make_directory colcon_ws && cd colcon_ws && \
git clone https://gitlab.kitware.com/keu-computervision/slam.git src/slam --recursive -b feat/ROS2
colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release && \
source install/setup.bash \
ros2 launch lidar_slam slam_velodyne.py


### Launch data

#### Create replay container

sudo docker run -it \
  --net ros_net \
  -e ROS_MASTER_URI=http://roscore:11311 \
  --name replay_cont \
  ros_humble

##### In an other terminal
sudo docker cp "/path/to/bag" replay_cont:/root

##### In the replay container
ros2 bag play bag -d1 --clock