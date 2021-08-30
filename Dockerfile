from ros:melodic-ros-core 

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    ros-$ROS_DISTRO-rgbd-launch \
    ros-$ROS_DISTRO-libuvc \
    ros-$ROS_DISTRO-libuvc-camera \
    ros-$ROS_DISTRO-libuvc-ros \
    build-essential \
    git

WORKDIR /app

# create ROS2 workspace and clone Orbbec Astra package
RUN mkdir -p ros_ws/src \
    && git clone https://github.com/orbbec/ros_astra_camera.git --branch=master ros_ws/src/ros_astra_camera

# build ROS2 workspace
RUN cd ros_ws \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# clear ubuntu packages
RUN apt clean && \
    rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]