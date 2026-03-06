---
title: ROS2 Control Overview
parent: ROS2 Control
nav_order: 1
---

## ROS2 Control Overview

ROS2 Control is a library for exactly what it sounds like, controlling the robot. It handles things from kinematics to joystick input to communicating with the microcontrollers and other hardware. It is a large topic to tackle, kind of complicated, and the documentation on the topic is also not very good at all when it comes to actual implementation. The conceptual explanations for ROS2 Control are well done.

Firstly, read this documentation page starting at [Architecture](https://control.ros.org/jazzy/doc/getting_started/getting_started.html#architecture) and ending before the Hardware Description in URDF section.

Basically, there's three main parts of ROS2 Control that are relevant for our purposes: Controllers, interfaces, and hardware components. All of these parts are managed by the Controller Manager, which is mostly relevant for writing launch files. This documentation aims to inform you just enough to be able to write ROS2 Control code, but not enough to understand the intricate workings of it. There are some parts of the ROS2 Control feature base that you can only really understand by looking at the source code, so a comprehensive explanation of features is not feasible.

## Interfaces

## Controllers

## Hardware Components

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
