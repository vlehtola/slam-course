#!/bin/bash
trap : SIGTERM SIGINT

# ===== X11 Display Access (for RViz) =====
export XSOCK=/tmp/.X11-unix
export XAUTH=$HOME/.Xauthority
xhost +local:root

# ===== Host paths =====
BAG_DIR=$(pwd)/bagfiles/nya_01
RESULT_DIR=$(pwd)/results/nya_01
mkdir -p $RESULT_DIR

# ===== Run Docker Container =====
docker run \
-it \
--rm \
--net=host \
--privileged \
-e DISPLAY=$DISPLAY \
-v $XSOCK:/tmp/.X11-unix \
-v $HOME/.Xauthority:/root/.Xauthority:rw \
-v $BAG_DIR:/home/rpcn/catkin_ws/src/bagfiles/nya_01 \
-v $RESULT_DIR:/home/rpcn/output \
rpcnc \
/bin/bash -c "
source /home/rpcn/catkin_ws/devel/setup.bash;

# 1. Launch A-LOAM normally (as before)
roslaunch aloam run_ntuviral.launch &

# 2. Wait a few seconds for initialization
sleep 10;

# 3. Record important topics (until you stop the container)
rostopic echo -p /aft_mapped_to_init_high_frec_horz > /home/rpcn/output/opt_odom_horz.csv &

# 4. Keep container running until user stops it manually (Ctrl+C)
echo 'Recording... Press Ctrl+C to stop when finished.';
wait
"

# ===== Reset Display Access =====
xhost -local:root

