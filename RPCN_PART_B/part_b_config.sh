#!/bin/bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/opt/cartographer_ros/setup.bash"

mkdir -p /home/rpcn/catkin_ws/src
cp -rf /rpcn_part_b /home/rpcn/catkin_ws/src
cp -rf /bagfiles /home/rpcn/catkin_ws/src/rpcn_part_b/
cd /home/rpcn/catkin_ws
catkin_make
rm -rf /rpcn_part_b /bagfiles
sudo apt-get update --fix-missing
apt-get -y install dialog apt-utils vim sudo wget usbutils nano curl git vim-gtk3 rviz

apt-get install ros-noetic-rviz


