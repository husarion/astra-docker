ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-base AS pkg-builder

ARG PREFIX

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
        libusb-1.0-0-dev \
        git \
        wget \
        libgflags-dev \
        nlohmann-json3-dev \
        ros-$ROS_DISTRO-image-transport \
        ros-$ROS_DISTRO-image-transport-plugins \
        ros-$ROS_DISTRO-image-publisher && \
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

# https://orbbec3d.com/index/download.html
RUN wget -c https://dl.orbbec3d.com/dist/openni2/ROS2/OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux.tar.gz \
        -O OpenNI_SDK_ROS2.tar.gz && \
    tar -xf OpenNI_SDK_ROS2.tar.gz && \
    mkdir src && \
    mv ros2_astra_camera src && \
    rm -rf OpenNI_SDK_ROS2*

# Fix TF parameters
RUN cd /ros2_ws/src/ros2_astra_camera/astra_camera/src/ && \
    sed -i \
    's/calcAndPublishStaticTransform();/if (!publish_tf_) return; calcAndPublishStaticTransform();/g' \
    ob_camera_node.cpp

# Create health check package
RUN cd src/ && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    ros2 pkg create healthcheck_pkg --build-type ament_cmake --dependencies rclcpp sensor_msgs && \
    sed -i '/find_package(sensor_msgs REQUIRED)/a \
            add_executable(healthcheck_node src/healthcheck.cpp)\n \
            ament_target_dependencies(healthcheck_node rclcpp sensor_msgs)\n \
            install(TARGETS healthcheck_node DESTINATION lib/${PROJECT_NAME})' \
            /ros2_ws/src/healthcheck_pkg/CMakeLists.txt

COPY ./healthcheck.cpp /ros2_ws/src/healthcheck_pkg/src/

RUN MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    apt update && \
    source "/opt/$MYDISTRO/$ROS_DISTRO/setup.bash" && \
    # without this line (using vulcanexus base image) rosdep init throws error: "ERROR: default sources list file already exists:"
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Save version
    echo $(cat /ros2_ws/src/ros2_astra_camera/astra_camera/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') > /version.txt && \
    # Size optimalization
    rm -rf build log src && \
    rm -rf /var/lib/apt/lists/*

# Add namespaca to launch file
COPY ./astra_mini.launch.py /ros2_ws/install/astra_camera/share/astra_camera/launch/astra_mini.launch.py
COPY rosbot-astra-params.yaml /ros2_ws/install/astra_camera/share/astra_camera/params/astra_mini_params.yaml

COPY ./healthcheck.sh /
HEALTHCHECK --interval=3s --timeout=1s  --start-period=10s --retries=1 \
    CMD ["/healthcheck.sh"]

# Without this line Astra doesn't stop the camera on container shutdown. Default is SIGTERM.
STOPSIGNAL SIGINT

# The commented section doesn't work (2nd stage just for size optimization)
# FROM ros:$ROS_DISTRO-ros-core

# # select bash as default shell
# SHELL ["/bin/bash", "-c"]

# RUN apt updarosbot-astra-paramste && apt install -y \
#         build-essential \
#         make \
#         libusb-1.0-0-dev \
#         libgflags-dev \
#         nlohmann-json3-dev \
#         ros-$ROS_DISTRO-tf2-msgs \
#         ros-$ROS_DISTRO-tf2-ros \
#         ros-$ROS_DISTRO-tf2-sensor-msgs \
#         ros-$ROS_DISTRO-tf2 \
#         ros-$ROS_DISTRO-image-transport \
#         ros-$ROS_DISTRO-image-publisher \
#         ros-$ROS_DISTRO-rmw-fastrtps-cpp \
#         ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
#     apt-get autoremove -y && \
#     apt-get clean && \
#     rm -rf /var/lib/apt/lists/*

# COPY --from=pkg-builder /glog-0.6.0 /glog-0.6.0
# COPY --from=pkg-builder /magic_enum-0.8.0 /magic_enum-0.8.0
# COPY --from=pkg-builder /libuvc /libuvc
# COPY --from=pkg-builder /ros2_ws /ros2_ws

# RUN cd /glog-0.6.0/build && make install && ldconfig && \
#     cd /magic_enum-0.8.0/build && make install && ldconfig && \
#     cd /libuvc/build && make install && ldconfig

# RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
# 	echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# COPY ros_entrypoint.sh /
