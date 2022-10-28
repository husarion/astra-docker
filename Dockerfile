ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
        git \
        libgflags-dev \
        ros-$ROS_DISTRO-image-geometry \
        ros-$ROS_DISTRO-camera-info-manager \
        ros-$ROS_DISTRO-image-transport \
        ros-$ROS_DISTRO-image-publisher \
        ros-$ROS_DISTRO-tf2-ros \
        ros-$ROS_DISTRO-tf \
        libgoogle-glog-dev \
        libusb-1.0-0-dev \
        libeigen3-dev && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/libuvc/libuvc.git && \
    cd libuvc && \
    git checkout a4de53e && \
    mkdir build && cd build && \
    cmake .. && make -j4 && \
    make install && \
    ldconfig

WORKDIR /ros_ws

RUN mkdir -p src && \
	git clone https://github.com/orbbec/ros_astra_camera.git --branch=main src/ros_astra_camera

# # build ROS workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	catkin_make --pkg astra_camera

COPY ./ros_entrypoint.sh /