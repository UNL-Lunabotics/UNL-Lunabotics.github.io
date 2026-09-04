---
title: ROS2 Control File
parent: Files
nav_order: 1
---

## ROS2 Control File

MuJoCo ROS2 Control, as the name suggests, is a hardware interface for ROS2 Control. To set up MuJoCo ROS2 Control, put the following in the URDF:

```xml
<!-- Robot URDF/Xacro File -->
<ros2_control name="MujocoSystem" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
    <!-- Optional: Defaults to /mujoco_robot_description -->
    <param name="mujoco_model_topic">/mujoco_robot_description</param>
    <!-- Other configuration -->
  </hardware>
  <joint name="front_left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10</param>
      <param name="max">10</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- Other joints -->
</ros2_control>
```

Here, we define the ROS2 Control hardware plugin. Next, just like any other ROS2 Control system, define any joints with command interfaces and/or state interfaces, if needed.

The `mujoco_model_topic` parameter is different from the main Robot State Publisher topic, which is usually `/robot_description`. `/mujoco_robot_description` is specific to MuJoCo, but more on this later. It is an optional parameter if defaults are used.

There are other configuration parameters that can be set. See the [MuJoCo ROS2 Control Hardware Interfaces](https://control.ros.org/jazzy/doc/mujoco_ros2_control/mujoco_ros2_control/docs/hardware_interface.html) documentation for further configuration.

{: .note}
Currently, the documentation says to configure `<sensor>` tags inside the ROS2 Control configuration. This has begun to be deprecated in favor of a separate plugin setup (more on this later). Unless there is no other way, do not define any `<sensor>` tags here.

> Author: Aiden Kimmerling <https://github.com/TheKing349>
