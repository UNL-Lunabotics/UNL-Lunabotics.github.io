---
title: MuJoCo Overview
parent: Conceptual
nav_order: 1
---

## Table of contents

{:toc}

## Overview

MuJoCo ROS2 Control, as the name suggests, is a ROS2 Control hardware interface that bridges ROS2 topics and configuration to something that MuJoCo can work with. This hardware interface defines robot movement, sensor input, and optional plugins. There are three main steps to get a MuJoCo simulation running:

1. Define and configure the ROS2 Control hardware interface
2. Define and configure preprocessing on certain joints and sensors
3. Define a launch file to start up MuJoCo ROS2 Control

Preprocessing is especially important. MuJoCo does not natively work with ROS2, nor does it use --- or know about --- ROS2 topics. During start up, a conversion script is used to convert the completed URDF into a MJCF, MuJoCo's file format that defines objects in a world. Because of this, preprocessing is needed to ensure certain joints and sensors are configured properly while conversion is happening.

Once the conversion script completes, MuJoCo launches and places the robot into a world, with an option to use an existing scene file as a base.

Each step of this will be broken down into their respective parts.

## Differences From Gazebo

Gazebo and MuJoCo have lots of differences in how they work, from different file expectations to ROS2 topics and reliability.

### Robot Description

Gazebo will natively use the `/robot_description`. Since MuJoCo works with MJCF instead of URDF, a script is required to convert the URDF to MJCF, and therefore uses a separate topic (e.g. `/mujoco_robot_description`).

### Meshes

As mentioned before, MuJoCo ROS2 Control has a difficult time with `.stl`, whereas Gazebo has 1st party support with it. Both MuJoCo and Gazebo can work with `.obj` files, which is why that file type is recommended.

When adding meshes, Gazebo will imply that any empty space in a mesh file will be treated as open air, and will not put a collision box. MuJoCo, on the other hand will define the collision to the entire mesh by default. Because of this, split up any meshes to singular components, then import and puzzle-piece them together. This way, collision will be more accurate. This will work both in MuJoCo and Gazebo.

### Bridging

Gazebo uses a special `gz_bridge.yaml` file to select any ROS2 topics that need to be bridged from Gazebo to ROS2 (or vise-versa). MuJoCo ROS2 Control instead converts sensors, movement, and position to MuJoCo, then sends that data to a ROS2 topic. This way, a bridge file is not needed. For advanced sensors, use `mujoco_plugins.yaml` to add extra functionality.

### Simulation

In Gazebo, you can add or remove objects while the simulation is running. You can also save that as a new, or existing world. MuJoCo does not behave like this. It will use a static, predefined MJCF, and is not editable while the simulation is running. To add or remove objects, modify the MJCF itself.

Both Gazebo and MuJoCo can move objects that are able to, however. This way, you can move the robot to wherever. This may impact reliability in visualization software.

### Reliability

We have found that Gazebo can be unreliable at times, where it crashes while simulation is running, with little to no explanation as to why. In testing, MuJoCo seems to perform better. It runs faster and will work reliably for an extended period of time. MuJoCo seems to also be more extendable, easily able to add plugins if more functionality is required.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
