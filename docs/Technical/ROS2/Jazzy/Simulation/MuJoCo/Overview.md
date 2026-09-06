---
title: Overview
parent: MuJoCo
nav_order: 1
---

## Overview

MuJoCo ROS2 Control, as the name suggests, is a ROS2 Control hardware interface, that bridges ROS2 topics and configuration to something that MuJoCo can work with. This hardware interface defines robot movement, sensor input, and optional plugins. There are three main steps to get a MuJoCo simulation running:

1. Define and configure the ROS2 Control hardware interface
2. Define and configure preprocessing on certain joints and sensors
3. Define a launch file to start up MuJoCo ROS2 Control

Preprocessing is especially important. MuJoCo does not natively work with ROS2, nor does it use --- or know about --- ROS2 topics. During start up, a conversion script is used to convert the completed URDF into a MJCF, MuJoCo's file format that defines objects in a world. Because of this, preprocessing is needed to ensure certain joints and sensors are configured properly while conversion is happening.

Once the conversion script completes, MuJoCo launches and places the robot into a world, with an option to use an existing scene file as a base.

Each step of this will be broken down into their respective parts.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
