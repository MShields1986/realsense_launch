# realsense_launch
ROS Noetic package for launching the Intel RealSense driver into a nodelet manager. Nodelets are also launched to convert the depth images to an RGB pointcloud, crop, voxelise, removal outliers and estimate normals on the pointcloud, all within a single nodelet manager for efficiency.

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
The default args launch a d435i with visualisation in RViz.
```bash
roslaunch realsense_launch realsense.launch
```

The device type and visualisation can be toggled with arguements.
```bash
roslaunch realsense_launch realsense.launch device_type:=d415 rviz:=false
```
