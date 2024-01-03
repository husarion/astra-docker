<h1 align="center">
  Docker Images for Astra
</h1>

The repository includes a GitHub Actions workflow that automatically deploys built Docker images to the [husarion/astra-docker](https://hub.docker.com/r/husarion/astra) Docker Hub repositories. This process is based on [ros2_astra_camera](https://github.com/orbbec/ros2_astra_camera.git) repository.

[![ROS Docker Image](https://github.com/husarion/astra-docker/actions/workflows/ros-docker-image.yaml/badge.svg)](https://github.com/husarion/astra-docker/actions/workflows/ros-docker-image.yaml)

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

## Parameters

The original launch has been modified with the new parameters:

| **Product Name**   | **Description**                                                                                                                             | **Default Value**                              |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------- |
| `params_file`      | Full path to the Astra parameters file lidar                                                                                                | `/husarion_utils/astra_params.yaml`            |
| `robot_namespace`  | Namespace which will appear in front of all topics (including `/tf` and `/tf_static`).                                                      | `env("ROS_NAMESPACE")` (`""` if not specified) |
| `device_namespace` | Sensor namespace that will appear before all non absolute topics and TF frames, used for distinguishing multiple cameras on the same robot. | `""`                                           |

Using both `device_namespace` and `robot_namespace` makes:

- Topic: `/<robot_namespace>/<device_namespace>/<default_topic>`
- Topic TF: `/<robot_namesace>/tf`
- URDF Links: `<device_namespace>_link`

If any of the namespaces are missing, the field with `/` is omitted for topics, or replaced with default ones for URDF links.
