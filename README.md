# GPD-ROS-Docker image
A docker container for using Grasp Pose Detection in combination with ROS (melodic)

Based on [pcl docker by DLopezMadrid](https://github.com/DLopezMadrid/pcl-docker)

Adapted for GPD-ROS use ([GPD Lib](https://github.com/atenpas/gpd) , [ROS Wrapper](https://github.com/atenpas/gpd_ros/))

## Overview
This image is based on Ubuntu 18.04 and has the following packages installed:
- CUDA-10.2.89  
- nvidia-docker  
- CMake-3.10.2  
- VTK-8.2.0  
- PCL-1.11.2  
- OpenCV-4.5.2  
- GPD-2.0.0  
- Eigen  
- Flann  
- Boost  
- gdb  
- ROS Melodic (Desktop Full)  
- terminator  

### Build the image
run  
```
./build_image.sh
```

### Creating & starting a container
run (NOT using sudo)  
```
./setup_container.sh
```

If you get an error like (you probably ran the setup with sudo earlier):  
```
touch: setting times of '/tmp/.docker.xauth-n': Permission denied
```
or   
```
xauth:  unable to link authority file /tmp/.docker.xauth, use /tmp/.docker.xauth-n
```
Please delete the file first  
```
sudo rm /tmp/.docker.xauth 
```

### Starting an existing container
run  
```
./start_container.sh
```

This should start a terminator window.

## GPD ROS Wrapper
This image comes with the full ros desktop installation. The example ros ws is found in `/home/gpd/docker_dir/ws_gpd_tiago`.  

This workspace contains:   
- A modified version of [the gpd ROS Wrapper](https://github.com/atenpas/gpd_ros/) adopted for use with the TIAGo robot.  
- the *PCL TIAGo tutorial* from the [PAL tiago tutorials repo](https://github.com/pal-robotics/tiago_tutorials)  
- the [robot_description](https://github.com/pal-robotics/tiago_robot) to be able to visualize the robot in rviz.  

Find the workspace, build it with `catkin_make` (might need to build multiple times as it auto-generates some of the header files).
source the workspace, and run the gpd package using:  
```
roslaunch gpd_ros gpd_tiago.launch
```