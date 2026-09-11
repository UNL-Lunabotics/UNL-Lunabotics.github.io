---
title: RealSense
parent: Launch
nav_order: 2
---

## Introduction

RealSense2 cameras are an important part of our setup since they allow us to get image data, imu data, and depth data without the use of an internal compass or GPS. Due to this these cameras are ideal for autonomy and teleop use.

RealSense2 cameras have excellent documentation and have header libraries that can be used in both C++ and Python code snippets. We are just going to focus on C++ usage because the Python library is just a C++ wrapper.

## Setup & Calibration

### Calibration

Before you use the RealSense's built in IMU, make sure to calibrate it. There is very good documentation on how to do this inside the [RealSense Library Repository](https://github.com/realsenseai/librealsense/tree/master/tools/rs-imu-calibration) on how to do this. Basically do this clone the RealSense Repo and run the `rs-imu-calibration.py` script and follow the CLI instructions.

### Setup

#### ROS Middleware

Before running this node make sure to run the command `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` before launching the node. This command tells ROS2 which ROS Middleware (RMV) implementation to use. A RMV handles communication (publish, subscribe, services, actions) between nodes. By default ROS2 uses `rmv_fastrtps_cpp`, however `rmv_cyclonedds_cpp` is better for the following reasons:

- Better multicast support
- Fewer seg faults in high-frequency data steams like point clouds
- More predictable behavior in multi-node networks
- Handles high volumes of data
  - Especially useful if using multiple RealSense Cameras
- Can fix segfaults, dropped frames, and unresponsive nodes

#### Composable Nodes

A **composable node** is a ROS2 node designed to be loaded into a single process with another composable node. Instead of running each node as a separate executable, multiple composable nodes share a shared process. This allows ROS2 to do a zero-copy, intra-process communication between the nodes automatically.

When two nodes are inside the same container process, ROS2 can pass messages by reference, so you can avoid the process of sending the data over the network, which is often our greatest bottle neck.

So if a node needs to read the camera data frequently, consider composing the nodes in order avoid the network overhead. Read more on the [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/Tutorials/Demos/Intra-Process-Communication.html)

## How To Use

An example of how to use this code is in this [repo](https://github.com/UNL-Lunabotics/scripts_and_prebuilts/blob/main/prebuilts/camera/src/realsense_node.cpp). Essential to use this you are going to want to run this node which frequently publishes the data for every node to use.

To view and test this node you can use foxglove, a RViz2 alternative. This program allows you to view the topic data and visualize the RealSense's image and point cloud data. To run this program run `ros2 run foxglove_bridge foxglove_bridge` while the camera node is running. After then go to your PC's apps and search up the "FoxGlove" application to view the RealSense2 data.

> Author: Raegan Scheet (<https://github.com/cscheet2>)
