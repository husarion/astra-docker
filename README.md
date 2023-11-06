<h1 align="center">
  Docker Images for Astra
</h1>

The repository includes a GitHub Actions workflow that automatically deploys built Docker images to the [husarion/astra-docker](https://hub.docker.com/r/husarion/astra) Docker Hub repositories. This process is based on [official documentation](https://www.orbbec.com/developers/openni-sdk/).

[![ROS Docker Image](https://github.com/husarion/astra-docker/actions/workflows/ros-docker-image.yaml/badge.svg)](https://github.com/husarion/astra-docker/actions/workflows/ros-docker-image.yaml)

Dockerized Orbbec ROS 2 Astra package: `OpenNI_SDK_ROS2_v1.0.2` built for ROS 2 Humble, Galactic and Foxy distros.


## Prepare Environment

**1. Plugin the Device**

You can use `lsusb` command to check if the device is visible.

## Demo

**1. Clone the Repository**

```bash
git clone https://github.com/husarion/astra-docker.git
cd astra-docker/demo
```
**2. Activate the Device**

```bash
docker compose up astra
```

**3. Launch Visualization**

```bash
xhost local:root
docker compose up rviz
```

> [!NOTE]
> You can run the visualization on any device, provided that it is connected to the computer to which the sensor is connected.
