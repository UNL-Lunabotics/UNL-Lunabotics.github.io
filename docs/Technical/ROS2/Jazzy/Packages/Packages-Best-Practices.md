---
title: Packages Best Practices
parent: Packages
nav_order: 2
---

## Packages Best Practices

{: .warning}
These are a mix of official ROS2 best practices and practices that just UNL Lunabotics likes to follow. Do not consider this 100% official in any way, and these practices can be revised at any point.

1. All ROS2 packages should be subfolders of a folder named either `src/` or `ros2/` and this parent folder should be at the root of the repository.
2. Packages that can be Python packages should be, since it is easier to maintain and create.
3. Basic standard packages should be:
   1. bringup/
      1. This is a Python package where the launch Python files are written, and the config files for RViz2 or Foxglove are stored.
   2. control/
      1. CMake package where all the ROS2 Control code lives, including microcontroller and CAN comms, all written in C++. This is also where config YAML's for the joystick and controller are stored.
   3. description/
      1. This is a Python package where all the URDF for the robot is, including meshes.
   4. interfaces/
      1. This is a CMake package where custom ROS2 service interfaces are.
   5. autonomy/
      1. This is a CMake package all the logic for autonomy (such as behavior trees and any extra scripts) are stored.
   6. More can be added if needed
4. packages_should_be_lower_snake_case
5. The name of the package should be descriptive and tell you exactly what is contained in that package (without being verbose)

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
