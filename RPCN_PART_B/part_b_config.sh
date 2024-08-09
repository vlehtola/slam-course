#!/bin/bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/opt/cartographer_ros/setup.bash"

mkdir -p /home/rpcn/catkin_ws/src
cp -rf /rpcn_part_b /home/rpcn/catkin_ws/src
cd /home/rpcn/catkin_ws
catkin_make
rm -rf /rpcn_part_b
sudo apt-get update --fix-missing
apt-get -y install vim nano
# apt-get -y install dialog apt-utils vim sudo wget usbutils nano curl git vim-gtk3 rviz

# apt-get install ros-noetic-rviz


