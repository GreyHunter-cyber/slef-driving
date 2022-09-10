#!/bin/sh
. /opt/ros/melodic/setup.sh
. /home/nvidia/iau_ros_legoloam/devel/setup.sh
rsync --daemon & sleep 10
roscore & sleep 10

roslaunch yesense_driver yesense_driver.launch & sleep 6
roslaunch lego_loc cutrun.launch & sleep 5
roslaunch rslidar_pointcloud rs_lidar_16.launch & sleep 2
roslaunch global_path_sim global_path_sim.launch & sleep 2
roslaunch frenet_planner frenet_planner.launch & sleep 2
roslaunch simple_follower simple_follower_pure.launch & sleep 1
roslaunch carTop driver.launch & sleep 2

