# docker run --rm -it -v ${PWD}:/ros2_ws ros:galactic bash

FROM ros:galactic

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
        libusb-1.0-0-dev \
        git \
        wget \
        libgflags-dev \
        nlohmann-json3-dev \
        ros-galactic-image-transport \
        ros-galactic-image-publisher && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz \
        -O glog-0.6.0.tar.gz && \
    tar -xzvf glog-0.6.0.tar.gz && \
    cd glog-0.6.0 && \
    mkdir build && cd build && \
    cmake .. && make -j4 && \
    make install && \
    ldconfig

RUN wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz \
        -O  magic_enum-0.8.0.tar.gz && \
    tar -xzvf magic_enum-0.8.0.tar.gz && \
    cd magic_enum-0.8.0 && \
    mkdir build && cd build && \
    cmake .. && make -j4 && \
    make install && \
    ldconfig

RUN git clone https://github.com/libuvc/libuvc.git && \
    cd libuvc && \
    # git checkout d3318ae && \
    mkdir build && cd build && \
    cmake .. && make -j4 && \
    make install && \
    ldconfig

WORKDIR /ros2_ws

RUN wget -c https://dl.orbbec3d.com/dist/openni2/ROS2/OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux.tar.gz \
        -O OpenNI_SDK_ROS2.tar.gz && \
    tar -xf OpenNI_SDK_ROS2.tar.gz && \
    mkdir src && \
    mv ros2_astra_camera src && \
    rm -rf OpenNI_SDK_ROS2*

RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && \
    colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
	echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

COPY ros_entrypoint.sh /ros_entrypoint.sh