#!/bin/bash
trap : SIGTERM SIGINT

# Ensure the X server is accessible (for running GUI apps like rviz)
export XSOCK=/tmp/.X11-unix
export XAUTH=$HOME/.Xauthority
xhost +local:root

# Run the docker command with the 16-channel LiDAR configuration
docker run \
-it \
--rm \
--net=host \
--privileged \
-e DISPLAY=$DISPLAY \
-v $XSOCK:/tmp/.X11-unix \
-v $HOME/.Xauthority:/root/.Xauthority:rw \
-v ~/slam-course/bagfiles/nya_01:/home/rpcn/catkin_ws/src/bagfiles/nya_01/ \
rpcnc \
/bin/bash -c \
"source /home/rpcn/catkin_ws/devel/setup.bash; \
roslaunch aloam run_ntuviral.launch"

# Reset xhost permissions for security
xhost -local:root
