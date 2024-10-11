xhost +"local:docker@"
docker run --name assignment_part_a --rm -it --net=host -e DISPLAY=unix:$DISPLAY --privileged -v $XSOCK -v $XAUTH --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
-v ~/slam-course/bagfiles:/home/rpcn/backpack/bagfiles/ -v /tmp/.X11-unix/:/tmp/.X11-unix rpcna bash
