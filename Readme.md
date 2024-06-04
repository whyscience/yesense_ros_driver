# ROS Driver fo Yesense IMU

## Introduction

This is a ros driver for yesense imu sensor on ROS2 

## Install Dependence

Assume that you are using **ROS Humble** 

```shell
mkdir -p yesense_ws/src
cd yesense_ws/src
#sudo apt install ros-humble-serial-driver
git clone https://github.com/RoverRobotics-forks/serial-ros2.git
```

## Build

```
cd yesense_ws/
colcon build --symlink-install
```

## Usage

```shell
ros2 launch yesense_imu yesense.launch.py
```

**note: **change the params in launch to your own
