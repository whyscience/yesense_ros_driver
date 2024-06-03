# ROS Driver fo Yesense IMU

## Introduction

This is a ros driver for yesense imu sensor for ROS2 

## Install Dependence

Assume that you are using **ROS Humble** 

```shell
sudo apt install ros-humble-serial-driver
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
