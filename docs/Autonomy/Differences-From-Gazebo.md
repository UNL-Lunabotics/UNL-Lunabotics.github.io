---
title: Differences From Gazebo
parent: MuJoCo
nav_order: 5
---

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
