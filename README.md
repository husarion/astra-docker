# astra-docker
Dockerized Orbbec Astra package from https://github.com/orbbec/ros_astra_camera repository

This repository contains a GitHub Actions workflow for auto-deployment of built Docker image to https://hub.docker.com/r/husarion/astra repository.

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
    astra \
    roslaunch astra_camera astra.launch
```

## Examples (using Docker Compose)

### Astra container + rviz container

Connect Orbbec Astra camera, to your laptop, change device in *docker-compose.yaml* like in previous step and deppending you have NVIDIA GPU or not, choose the right example:

#### [option 1] NVIDIA GPU

```bash
cd examples/rviz/nvidia

xhost local:root
docker-compose up --build
```

#### [option 2] Intel GPU

```bash
cd examples/rviz/intel

xhost local:root
docker-compose up --build
```