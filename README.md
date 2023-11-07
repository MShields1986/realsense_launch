# realsense_launch
ROS Noetic package for launching the Intel RealSense driver and getting an RGB pointcloud as output.

## Installation
```bash
sudo apt-get update
sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install ros-noetic-realsense2-description
sudo apt-get install ros-noetic-image-pipeline
```

Working in your catkin_ws/src directory...
```bash
git clone https://github.com/MShields1986/realsense_launch.git
```

...and build...
```bash
cd ..
catkin_make
```

## Usage
Source you environment and run the launch file using the following command.
```bash
roslaunch realsense_launch realsense.launch
```
The default args launch a d415 with visualisation of the pointcloud, RGB image and d415 camera model in RViz.

The device type and visualisation can be toggled with arguements.
```bash
roslaunch realsense_launch realsense.launch device_type:=d435 rviz:=false
```

## TODO
- Add descriptions for other models to launch file
