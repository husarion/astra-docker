# astra-docker
Dockerized Orbbec Astra package from [orbbec/ros_astra_camera ](https://github.com/orbbec/ros_astra_camera) repository.

## Building a Docker image

```bash
sudo docker build -t astra .
```

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
