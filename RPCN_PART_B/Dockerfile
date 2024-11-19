# This section is from official google cartographer which is used as a tool in this assignment 
FROM osrf/ros:noetic-desktop

ARG CARTOGRAPHER_VERSION=master
ARG CARTOGRAPHER_SHA=LATEST

# Prevent any interaction required by apt-get.
# https://stackoverflow.com/questions/22466255
ARG DEBIAN_FRONTEND=noninteractive

# ROS Noetic's base image doesn't ship with sudo and git.
RUN apt-get update && apt-get install -y sudo git

# wstool needs the updated rosinstall file to clone the correct repos.
COPY cartographer_ros/cartographer_ros.rosinstall cartographer_ros/
COPY cartographer_ros/scripts/prepare_catkin_workspace.sh cartographer_ros/scripts/

# First, we invalidate the entire cache if cartographer-project/cartographer has
# changed. CARTOGRAPHER_SHA ARG content changes whenever master changes. See:
# http://stackoverflow.com/questions/36996046/how-to-prevent-dockerfile-caching-git-clone
RUN CARTOGRAPHER_SHA=$CARTOGRAPHER_SHA \
    CARTOGRAPHER_VERSION=$CARTOGRAPHER_VERSION \
    cartographer_ros/scripts/prepare_catkin_workspace.sh && \
    sed -i -e "s%<depend>libabsl-dev</depend>%<\!--<depend>libabsl-dev</depend>-->%g" catkin_ws/src/cartographer/package.xml

# rosdep needs the updated package.xml files to install the correct debs.
COPY cartographer_ros/cartographer_ros/package.xml catkin_ws/src/cartographer_ros/cartographer_ros/
COPY cartographer_ros/cartographer_ros_msgs/package.xml catkin_ws/src/cartographer_ros/cartographer_ros_msgs/
COPY cartographer_ros/cartographer_rviz/package.xml catkin_ws/src/cartographer_ros/cartographer_rviz/
COPY cartographer_ros/scripts/install_debs.sh cartographer_ros/scripts/
RUN cartographer_ros/scripts/install_debs.sh

# Install Abseil.
RUN /catkin_ws/src/cartographer/scripts/install_abseil.sh

# Build, install, and test all packages individually to allow caching. The
# ordering of these steps must match the topological package ordering as
# determined by Catkin.
COPY cartographer_ros/scripts/install.sh cartographer_ros/scripts/
COPY cartographer_ros/scripts/catkin_test_results.sh cartographer_ros/scripts/

RUN cartographer_ros/scripts/install.sh --pkg cartographer && \
    cartographer_ros/scripts/install.sh --pkg cartographer --make-args test

COPY cartographer_ros/cartographer_ros_msgs catkin_ws/src/cartographer_ros/cartographer_ros_msgs/
RUN cartographer_ros/scripts/install.sh --pkg cartographer_ros_msgs && \
    cartographer_ros/scripts/install.sh --pkg cartographer_ros_msgs \
        --catkin-make-args run_tests && \
    cartographer_ros/scripts/catkin_test_results.sh build_isolated/cartographer_ros_msgs

COPY cartographer_ros/cartographer_ros catkin_ws/src/cartographer_ros/cartographer_ros/
RUN cartographer_ros/scripts/install.sh --pkg cartographer_ros && \
    cartographer_ros/scripts/install.sh --pkg cartographer_ros \
        --catkin-make-args run_tests && \
    cartographer_ros/scripts/catkin_test_results.sh build_isolated/cartographer_ros

COPY cartographer_ros/cartographer_rviz catkin_ws/src/cartographer_ros/cartographer_rviz/
RUN cartographer_ros/scripts/install.sh --pkg cartographer_rviz && \
    cartographer_ros/scripts/install.sh --pkg cartographer_rviz \
        --catkin-make-args run_tests && \
    cartographer_ros/scripts/catkin_test_results.sh build_isolated/cartographer_rviz

COPY cartographer_ros/scripts/ros_entrypoint.sh /

RUN rm -rf /var/lib/apt/lists/*
# A BTRFS bug may prevent us from cleaning up these directories.
# https://btrfs.wiki.kernel.org/index.php/Problem_FAQ#I_cannot_delete_an_empty_directory
# RUN rm -rf cartographer_ros catkin_ws || true

#This section will copy and build the ros enviornment for the RPCN assignment part B
COPY part_b_config.sh / 
COPY rpcn_part_b /rpcn_part_b
#Note: do not copy bagfiles, mount the directory instead to docker when running the image
RUN chmod +x /part_b_config.sh
RUN  bash /part_b_config.sh
WORKDIR /home/rpcn
