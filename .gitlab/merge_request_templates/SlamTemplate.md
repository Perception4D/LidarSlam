# Goal

Before, on master, the SLAM was doing this. The goal of this MR is to do this and that.

# Changes

* A function called Myfunction is added to [this file](https://gitlab.kitware.com/keu-computervision/slam_epita/-/blob/master/slam_lib/src/LocalOptimizer.cxx?ref_type=heads). It takes that as input and this as output. The parameters are bla and bli because. It does this and that following this method
  1. First it does that
  2. Then it does this
  3. And finally it does that
* The previous function is used in foo() to do that. See image below for an example

![myImage](/uploads/fd3c7b2e05117ab69ee0fb1990994465/myImage.png)

# Results

## Commands to reproduce the results

`colcon build --base-paths slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release`

* Change the parameters :
  * leaf_size=5
  * ...

`ros2 launch my_launch.launch param1:=2`

`ros2 bag play --clock path/to/this_bag.bag`

## Results

* Here is a resulting map
* Here is a resulting trajectory
* Here is an example of input frame and its keypoints (optional)
* Here is a screenshot of the SLAM result

## Performances (optional)

* The overall SLAM process before (on master) was taking 10s
* The overall process takes 9s

# What remains to solve

* [ ] this
* [ ] this
* [ ] that
