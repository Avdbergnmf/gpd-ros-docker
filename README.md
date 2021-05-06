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

### Test GPD installation
`cd /home/gpd/gpd/build`, then
```
./detect_grasps ../cfg/eigen_params.cfg ../tutorials/krylon.pcd
```


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

### Start TIAGo gazebo sim
If used in combination with a TIAGo docker image, start a gazebo simulation in its docker container:  

launch tiago_gazebo with the world: Good options are `objects_on_table`, and `tabletop_cube`, and with model `steel`
```bash
roslaunch tiago_134_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=tabletop_cube
```

To look down at the objects:  
Load up the pcl motions parameters into the `play_motion` namespace (in the gpd-ros container)
```
rosparam load `rospack find tiago_pcl_tutorial`/config/pcl_motions.yaml
```
- To check if they are loaded, find them using `rosparam list | grep look_down`

In the TIAGo Docker container:  
```
rosrun actionlib axclient.py /play_motion
```  
and run the `look_down` motion.  
```
motion_name: 'look_down'
skip_planning: True
priority: 0
```

### PCL Options
The topic from which the point cloud is read is defined in the `gpd_tiago` launch file.  

You can either feed the complete pointcloud to GPD package, make sure to set the cloud_topic line 

```
    <param name="cloud_topic" value="/xtion/depth_registered/points" />
```

Alternatively, we can modify what point cloud data is fed to GPD. *The TIAGo PCL Tutorial* package provides some examples on how to do this.

#### Table segmentation
We can filter out the table, start the launch file:  
```
roslaunch tiago_pcl_tutorial segment_table.launch
```
- To prevent rviz from loading, add `show_rviz:=false`

The points published on `/segment_table/nonplane` are the objects that we found on the table. To feed this data to the gpd package, modify the `gpd_tiago.launch` *cloud_topic* param to look for this topic.

#### Cylinder Detector
If we specifically want to pick up a cylinder shape, launch the launch file:  
```
roslaunch tiago_pcl_tutorial cylinder_detector.launch
```
- To prevent rviz from loading, add `show_rviz:=false`

There are some topics published under the `/cylinder_detector` namespace, which are also visualized in rviz. To feed this data to the gpd package, modify the `gpd_tiago.launch` *cloud_topic* param to look for this topic.

#### Region Segmentation
We can segment the point cloud into regions, so that we can specify a certain region to feed to gpd. Launch the launch file:  
```
roslaunch tiago_pcl_tutorial pcl_region.launch
```
- To prevent rviz from loading, add `show_rviz:=false`

Also start a *rqt_reconfigure* window. In this window you can play with the parameters untill you get the desired results.

Find the topic of the region you want to use and feed this data to the gpd package, modify the `gpd_tiago.launch` *cloud_topic* param to look for this topic.
