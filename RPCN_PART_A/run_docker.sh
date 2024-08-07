sudo xhost +"local:docker@"
sudo docker run --rm -it --net=host -e DISPLAY=unix$DISPLAY --privileged rpcn bash
