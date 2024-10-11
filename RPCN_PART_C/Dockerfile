FROM ros:kinetic-perception

# Set environment variables
ENV CERES_VERSION="1.12.0"
ENV PCL_VERSION="1.8.0"
ENV CATKIN_WS=/home/rpcn/catkin_ws

# Setup processors number used to compile library
RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi && \
    # Install dependencies
    apt-get update && apt-get install -y \
      cmake \
      git \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf \
      ros-${ROS_DISTRO}-rviz \
      libgl1-mesa-glx \
      libgl1-mesa-dri && \
    rm -rf /var/lib/apt/lists/* && \
    # Build and install Ceres
    git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j${USE_PROC} install && \
    cd ../.. && \
    rm -rf ./ceres-solver && \
    # Build and install pcl
    git clone https://github.com/PointCloudLibrary/pcl.git && \
    cd pcl && \
    git checkout tags/pcl-${PCL_VERSION} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j${USE_PROC} install && \
    cd ../.. && \
    rm -rf ./pcl && \
    # Setup catkin workspace
    mkdir -p $CATKIN_WS/src/

# Clone the A-LOAM repository from GitHub
RUN git clone https://github.com/brytsknguyen/A-LOAM.git $CATKIN_WS/src/A-LOAM
RUN echo "Current working directory:" && pwd && ls -la

#RUN chmod 700 slam-course/RPCN_PART_C/.
COPY RPCN_PART_C/run_ntuviral.launch /home/rpcn/catkin_ws/src/A-LOAM/launch
# Set the working directory to the catkin workspace
WORKDIR $CATKIN_WS

# Set environment variables
ENV TERM=xterm
ENV PYTHONIOENCODING=UTF-8

# Build the A-LOAM package
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build && \
    # Modify the ros_entrypoint.sh to source the workspace
    sed -i '/exec "$@"/i \
            source "/home/rpcn/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

# Set the default command to run bash
CMD ["bash"]



