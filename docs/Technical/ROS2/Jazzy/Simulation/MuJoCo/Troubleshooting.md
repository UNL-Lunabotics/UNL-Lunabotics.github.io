---
title: Troubleshooting
parent: MuJoCo
nav_order: 4
---

## Troubleshooting

Sometimes MuJoCo ROS2 Control can error out, with little to no information about why, especially when it segfaults. Since a lot of different files are used, it may be hard to track down what the cause of the error is.

### ROS2 Control Hardware Interface

If there is an issue with the ROS2 Control hardware interface, it may segfault. Ensure that the hardware plugin is defined and is correct. Also ensure that each moveable joint is properly defined and has a `<state_interface>` and/or `<command_interface>`.

### MuJoCo Inputs

If improperly defined, or not defined at all, MuJoCo ROS2 Control may result in a segmentation fault. Make sure `<default>`, `<actuator>`, and `<sensor>` tags are properly defined in the `<raw_inputs>` section. Verify any `<processed_inputs>` are properly defined.

### MuJoCo Plugins

The `mujoco_plugins.yaml` file may result in a segmentation fault, or an explicit error, depending on what the issue is. Ensure that the file has the correct starting format, and all necessary plugin parameters are defined.

### Simulation Launch File

If improperly formatted, the simulation launch file could result in a segmentation fault. For our use cases, ensure the `robot_description_to_mjcf.sh` node is defined, with the `--add_free_joint` and `--publish_topic` parameters. Remember that if the `--publish_topic` is changed from the default, the topic will need to be added to the ROS2 Control hardware interface. For the `mujoco_ros2_control` node, make sure the `controller.yaml` is included, as well as the optional `mujoco_plugins.yaml`.

### MuJoCo Scene File

If invalid syntax, the MJCF may result in a segmentation fault or explicit error. Ensure it follows proper MJCF rules. If linking to another MJCF or mesh file, make sure the path to the file(s) are the right path.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
