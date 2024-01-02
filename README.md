<h1 align="center">
  Docker Images for Astra
</h1>

The repository includes a GitHub Actions workflow that automatically deploys built Docker images to the [husarion/astra-docker](https://hub.docker.com/r/husarion/astra) Docker Hub repositories. This process is based on [ros2_astra_camera](https://github.com/orbbec/ros2_astra_camera.git) repository.

[![ROS Docker Image](https://github.com/husarion/astra-docker/actions/workflows/ros-docker-image.yaml/badge.svg)](https://github.com/husarion/astra-docker/actions/workflows/ros-docker-image.yaml)

## Add Namespace

The original launch has been improved with the ability to add parameters: device_namespace and rosbot_namespace
which result in the following changes:

- Topic: `/<robot_namespace>/<device_namespace>/<default_topic>`
- Topic TF: `/<robot_namesace>/tf`
- URDF Links: `<device_namespace>_link`

If any of the namespaces are missing, the field with `/` is omitted for topics, or replaced with default ones for URDF links.

## Prepare Environment

1. Plugin the Device

    You can use `lsusb` command to check if the device is visible.

## Demo

1. Clone the Repository

    ```bash
    git clone https://github.com/husarion/astra-docker.git
    cd astra-docker/demo
    ```

2. Activate the Device

    ```bash
    docker compose up astra
    ```

3. Launch Visualization

    ```bash
    xhost local:root
    docker compose up rviz
    ```

> [!NOTE]
> To use the latest version of the image, run the `docker compose pull` command.
