from ros:melodic-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    ros-melodic-rgbd-launch \
    ros-melodic-libuvc \
    ros-melodic-libuvc-camera \
    ros-melodic-libuvc-ros \
    build-essential \
    git

WORKDIR /app

RUN mkdir -p ros_ws/src && \
    git clone https://github.com/orbbec/ros_astra_camera.git --branch=master ros_ws/src/ros_astra_camera

RUN cd ros_ws && \
    source /opt/ros/melodic/setup.bash && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# setup entrypoint
COPY ./ros_entrypoint.sh /


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]