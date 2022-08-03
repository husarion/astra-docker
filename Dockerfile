FROM ros:melodic-ros-core 

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
        ros-$ROS_DISTRO-rgbd-launch \
        ros-$ROS_DISTRO-libuvc \
        ros-$ROS_DISTRO-libuvc-camera \
        ros-$ROS_DISTRO-libuvc-ros \
        build-essential \
        git && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws

# create ROS2 workspace and clone Orbbec Astra package
RUN mkdir -p src && \
	git clone https://github.com/orbbec/ros_astra_camera --branch=master src/ros_astra_camera && \
    cd src/ros_astra_camera && git checkout 78186a3ffbf8a67cfdf409abeebc68c402b406a0

# build ROS2 workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release --pkg astra_camera

# setup entrypoint
COPY ./ros_entrypoint.sh /
