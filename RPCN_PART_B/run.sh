## special file to run on a remote machine
## fixed the problem of opening the rviz from inside the docker container
#xauth add $DISPLAY . `mcookie`
xhost +local:docker
docker run --name assignment_part_b -it --rm --privileged -e DISPLAY=$DISPLAY --net=host -v $XSOCK -v $XAUTH --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
-v ~/slam-course/RPCN_PART_B/bagfiles:/home/rpcn/catkin_ws/src/rpcn_part_b/bagfiles/ -v /tmp/.X11-unix/:/tmp/.X11-Unix  rpcn_partb  bash
