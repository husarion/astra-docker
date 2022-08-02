# docker run --rm -it -v ${PWD}:/ros2_ws ros:galactic bash

FROM ros:galactic

RUN apt update && apt install -y \
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

WORKDIR /ros2_ws

RUN wget -c https://dl.orbbec3d.com/dist/orbbecsdk/ROS2/v1.0/OrbbecSDK_ROS2_v1.0.4_20220713_776487_linux_release.tar.gz \
        -O OrbbecSDK_ROS2.tar.gz && \
    tar -xf OrbbecSDK_ROS2.tar.gz && \
    cp -r OrbbecSDK_ROS2/* /ros2_ws && \
    rm -rf OrbbecSDK_ROS2*

RUN ldconfig && colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release