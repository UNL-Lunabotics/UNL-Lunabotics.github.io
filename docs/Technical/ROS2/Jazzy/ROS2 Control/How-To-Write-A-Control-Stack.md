---
title: How to Write a Control Stack
parent: Jazzy
nav_order: 1
---

## How to Write a Control Stack

This page is meant to be your one stop for everything you need to code a custom ROS2 (Jazzy) control stack from scratch. It will include a full step by step process of what you should do, mostly by linking to other pages already written.

Things this documentation **WILL** cover:

- Setting up the ROS2 Control package
- Setting up the `ros2_control.xacro` URDF file
- Writing a controller (custom and built in)
- Writing a hardware component
- Connecting the hardware component to a microcontroller
- Writing the launch files for everything
- Setting up gamepad input
- Writing a proper `CMakeLists.txt` and `package.xml` for the control package
- Setting up CAN communication
- Setting up custom ROS2 services for gamepad input
- Setting up mock hardware components for testing
- Telemetry scaffolding
- Encoder best practices (software and hardware)
- C++ syntax that confuses new people so people can read the code better (not necessarily write it)

Things this documentation will **NOT** cover:

- Writing any part of the URDF that isn't for ROS2 Control
- Writing any part of the URDF that is meant for simulation
- Setting up a repository or any of the rest of the ROS2 packages besides Control
- Installing software
- Setting up Docker
- Controlling the robot in simulation
- Setting up any sort of robot visualization software like RViz2 or Foxglove
- Setting up any physical controller that isn't a gamepad (no keyboard or anything like that)
- Literally anything about autonomy
- How to write in C++
- How to program in general
- Basic ROS2 concepts

This is not to say that there does not exist ANY documentation on the above items, just that this guide will not cover it. Before starting this process, you should already have the rest of your ROS2 packages set up for development, including but not limited to description and bringup. You are expected to already have things like Docker or whatever development environment you are using. You are expected to have all required software already installed (it should all be in the Dockerfile). You are expected to have a basic knowledge of how ROS2 works and how to program in general. You are expected to already understand concepts like URDF.

{: .warning}
This is a highly technical guide, not a conceptual one. If you do not understand a concept written above, this documentation will in all likelihood will not explain it. You should consult the Curriculum section to learn more.

### 1. Creating the Control Package

Read the documentation on [ROS2 Packages]({% link docs/Technical/ROS2/Jazzy/Packages/Creating-Packages.md %}) and create an ament_cmake package named control.

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
