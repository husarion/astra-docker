ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
        libusb-1.0-0-dev \
        git \
        git-lfs \
        ros-$ROS_DISTRO-rgbd-launch \
        ros-$ROS_DISTRO-camera-info-manager \
        # ros-$ROS_DISTRO-libuvc \
        ros-$ROS_DISTRO-libuvc-camera \
        ros-$ROS_DISTRO-libuvc-ros && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/libuvc/libuvc.git && \
    cd libuvc && \
    git checkout d3318ae && \
    mkdir build && cd build && \
    cmake .. && make -j4 && \
    make install && \
    ldconfig

WORKDIR /ros_ws

RUN mkdir -p src && \
	git clone https://github.com/orbbec/ros_astra_camera --branch=master src/ros_astra_camera

# # build ROS workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	catkin_make --pkg astra_camera

COPY ./ros_entrypoint.sh /