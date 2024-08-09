# syntax=docker/dockerfile:1
## This is a dockerfile for installing ROS perception tools for RPCN course practicals along with
## Hokyuo lidar drivers, usb web cam drivers, and xsens mti IMU drivers 
## 2022 ++Ville Lehtola, University of Twente

## See docs from 
## http://wiki.ros.org/noetic
## https://docs.docker.com/

# This line installs ubuntu focal fossa with ROS noetic and perception packages
FROM ros:noetic-perception-focal
RUN sudo apt-get update --fix-missing

# some essentials
RUN apt-get -y install dialog apt-utils vim sudo wget usbutils nano curl git
RUN apt-get -y install libgtkmm-2.4-1v5 libglademm-2.4-1v5 libglade2-0 libgtkglextmm-x11-1.2-dev libgtkmm-2.4-dev libglademm-2.4-dev libglade2-dev libgtkglextmm-x11-1.2-0v5 libgtkglext1-dev  libglibmm-2.4-dev libpangomm-1.4-dev libsigc++-2.0-dev libgtkglext1 libpangox-1.0-dev libxmu-dev libpangox-1.0-0 libatkmm-1.6-dev libcairomm-1.0-dev libpangox-1.0-0 libxmu-headers 

# sensor drivers and rviz
RUN apt-get -y install ros-noetic-usb-cam ros-noetic-urg-node ros-noetic-xsens-mti-driver rviz

# config files
ADD README /home/rpcn/
ADD calib.urdf vis.rviz /home/rpcn/backpack/

# 2D SLAM, Google cartographer
WORKDIR /home/rpcn
RUN mkdir -p /home/rpcn/backpack/bagfiles/
RUN git clone https://github.com/cartographer-project/cartographer

# Dataset files
ADD test1.bag /home/rpcn/backpack/bagfiles/   
