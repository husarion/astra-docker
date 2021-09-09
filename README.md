# astra-docker
Dockerized Orbbec Astra package from [orbbec/ros_astra_camera ](https://github.com/orbbec/ros_astra_camera) repository.

## Determinating which device is Astra
Find Astra's Bus and Device number by running *lsusb*.
```bash
Bus 001 Device 006: ID 2bc5:0401 Orbbec(R) ORBBEC Depth Sensor
```

## Running a Docker image


```bash
sudo docker run --rm -it \
    --device /dev/bus/usb/001/006 \
    husarion/astra \
    roslaunch astra_camera astra.launch
```

## ROS node 

For more information on what is being published by ROS node in this docker refer to it's [README.md](https://github.com/orbbec/ros_astra_camera#important-topics)

Some of the topics are shown below.
### Publishes

- `*/image_raw` *(sensor_msgs/Image)*
- `*/camera_info` *(sensor_msgs/CameraInfo)*
- `/camera/depth/points` *(sensor_msgs/PointCloud)*

## Examples

### Astra container + rviz container

Connect Orbbec Astra camera, to your laptop, change device in *docker-compose.yaml* like in previous step and deppending you have NVIDIA GPU or not, choose the right example:

#### NVIDIA GPU

```bash
cd examples/rviz/nvidia

xhost local:root
docker-compose up
```

#### Intel GPU

```bash
cd examples/rviz/intel

xhost local:root
docker-compose up
```
