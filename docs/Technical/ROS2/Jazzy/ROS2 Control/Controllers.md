---
title: Controllers
parent: ROS2 Control
nav_order: 2
---

## Introduction

You can find a list of already existing ROS2 Controllers on the [ros2_controllers](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html) page. The rest of this guide will concern making a custom controller.

All controller logic will be written in C++, including extensive use of header files. If you are not familiar with C++, I recommend either crying or going to [LEARN C++](https://www.learncpp.com/) and doing some tutorials.

## Architecture of a Custom Controller

Firstly, you will need to make a CMake ROS2 package using the following command: `ros2 pkg create --built-type ament_cmake control`. Note that it HAS to be a CMake package; a Python package will not work.



> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
