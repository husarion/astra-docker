ARG ROS_DISTRO=humble
ARG PREFIX=

# =========================== package builder ===============================
FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-base AS pkg-builder

ARG PREFIX

WORKDIR /ros2_ws

RUN apt update && apt install -y \
        libgoogle-glog-dev \
        libusb-1.0-0-dev \
        libeigen3-dev \
        libgflags-dev \
        nlohmann-json3-dev \
        ros-$ROS_DISTRO-camera-info-manager \
        ros-$ROS_DISTRO-image-geometry \
        ros-$ROS_DISTRO-image-publisher \
        ros-$ROS_DISTRO-image-transport \
        ros-$ROS_DISTRO-image-transport-plugins \
        ros-$ROS_DISTRO-tf2 \
        ros-$ROS_DISTRO-tf2-eigen \
        ros-$ROS_DISTRO-tf2-sensor-msgs \
        ros-$ROS_DISTRO-cv-bridge

# Install libuvc
RUN git clone https://github.com/libuvc/libuvc.git src/libuvc && \
    cd src/libuvc && \
    mkdir build && cd build && \
    cmake .. && make -j4 && \
    make install && \
    ldconfig

# Install ros2_astra_camera (fork connected with https://github.com/orbbec/ros2_astra_camera/issues/1)
RUN cd src && \
    git clone https://github.com/rafal-gorecki/ros2_astra_camera.git && \
    git clone https://github.com/ros-misc-utilities/ffmpeg_image_transport.git && \
    cd .. && \
    vcs import src < src/ffmpeg_image_transport/ffmpeg_image_transport.repos && \
    # Fix publish_tf_ parameter
    sed -i 's/calcAndPublishStaticTransform();/if (!publish_tf_) return; calcAndPublishStaticTransform();/g' \
        /ros2_ws/src/ros2_astra_camera/astra_camera/src/ob_camera_node.cpp

# Create healthcheck package
RUN MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    source "/opt/$MYDISTRO/$ROS_DISTRO/setup.bash" && \
    cd src/ && \
    ros2 pkg create healthcheck_pkg --build-type ament_cmake --dependencies rclcpp sensor_msgs && \
    sed -i '/find_package(sensor_msgs REQUIRED)/a \
            add_executable(healthcheck_node src/healthcheck.cpp)\n \
            ament_target_dependencies(healthcheck_node rclcpp sensor_msgs)\n \
            install(TARGETS healthcheck_node DESTINATION lib/${PROJECT_NAME})' \
            /ros2_ws/src/healthcheck_pkg/CMakeLists.txt

COPY ./husarion_utils/healthcheck.cpp /ros2_ws/src/healthcheck_pkg/src/healthcheck.cpp

# Backward compatibility
COPY ./husarion_utils/astra.launch.py /ros2_ws/src/ros2_astra_camera/astra_camera/launch/astra_mini.launch.py

# Build
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -y && \
    MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    source "/opt/$MYDISTRO/$ROS_DISTRO/setup.bash" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Save version
    echo $(cat /ros2_ws/src/ros2_astra_camera/astra_camera/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') > /version.txt && \
    # Size optimalization
    rm -rf build log src

# =========================== final stage ===============================
FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core AS final-stage

# Add architecture specific packages conditionally
ARG TARGETPLATFORM
RUN if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        apt update && apt install -y libraspberrypi-bin; \
    fi

RUN apt update && apt install -y \
        libgoogle-glog-dev \
        ros-$ROS_DISTRO-image-geometry \
        ros-$ROS_DISTRO-image-publisher \
        ros-$ROS_DISTRO-image-transport \
        ros-$ROS_DISTRO-image-transport-plugins \
        ros-$ROS_DISTRO-tf2-ros \
        ffmpeg \
        ros-$ROS_DISTRO-cv-bridge && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY --from=pkg-builder /ros2_ws /ros2_ws
COPY --from=pkg-builder /version.txt /version.txt
COPY ./husarion_utils /husarion_utils

HEALTHCHECK --interval=2s --timeout=1s --start-period=20s --retries=1 \
    CMD ["/husarion_utils/healthcheck.sh"]

# Without this line Astra doesn't stop the camera on container shutdown. Default is SIGTERM.
STOPSIGNAL SIGINT
