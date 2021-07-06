from ros:melodic

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /opt/ros/ros_ws/src

RUN git clone -b master https://github.com/orbbec/ros_astra_camera.git /opt/ros/ros_ws/src/ros_astra_camera

RUN apt update && apt install -y ros-melodic-rgbd-launch ros-melodic-libuvc ros-melodic-libuvc-camera ros-melodic-libuvc-ros

RUN cd /opt/ros/ros_ws && \
    source /opt/ros/melodic/setup.bash && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release