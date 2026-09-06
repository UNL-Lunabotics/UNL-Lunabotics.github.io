---
title: Simulation Launch File
parent: Files
nav_order: 4
---

## Simulation Launch File

The launch file will need two different `Node`s in order to work.

### Converting Robot Description to MJCF

MuJoCo, as mentioned before, cannot work with ROS2 topics directly. Instead, a conversion script creates a MJCF, which MuJoCo can interact with. This conversion script converts the URDF (or `/robot_description` topic) into an MJCF (or `/mujoco_robot_description` topic), and optionally combines it with a given scene. Below is an example:

```python
mujoco_robot_description = Node(
  package="mujoco_ros2_control",
  executable="robot_description_to_mjcf.sh",
  output="both",
  arguments=[
    "--add_free_joint",
    "--scene", PathJoinSubstitution([FindPackageShare("sim"), "worlds", "my_mujoco_scene"]),
    "--publish_topic", "/mujoco_robot_description",
  ],
)
```

Here, we call the `robot_description_to_mjcf.sh` script, and give it some optional arguments:

- `--add_free_joint` allows the robot to move independently from the rest of the MuJoCo scene. Otherwise, the robot will be stuck in place.
- `--scene` appends a MuJoCo scene to the generated MJCF, allowing to test in different environments (such as the competition arenas).
- `--publish_topic` will publish the MJCF to a topic with a given name (default is `/mujoco_robot_description`). Otherwise, the conversion script places the MJCF into a static file.
  
{: .note}
If a different topic is used, the `<param name="mujoco_model_topic>` tag will need to be updated in the MuJoCo ROS2 Control hardware interface.

All of these arguments are technically optional, but for a live session with a moveable robot, they are needed.

This script is also responsible for going through the optional `mujoco_inputs.xacro` file to add on the `raw_inputs` at the end of the MJCF, and substitutes parameters from the `processed_inputs`.

For more information, see the [URDF to MJCF Conversion](https://control.ros.org/jazzy/doc/mujoco_ros2_control/mujoco_ros2_control/docs/tools.html) documentation.

### Start the ROS2 Control Node

Once conversion completes, we can start the MuJoCo ROS2 Control Node. Below is an example:

```python
control_node = Node(
  package="mujoco_ros2_control",
  executable="ros2_control_node",
  output="both",
  parameters=[
    {"use_sim_time": True},
    ParameterFile(PathJoinSubstitution([FindPackageShare("bringup"), "config", "controllers.yaml"])),
    ParameterFile(PathJoinSubstitution([FindPackageShare("sim"), "config", "mujoco_plugins.yaml"])),
  ],
)
```

Here, we execute the `ros2_control_node` from the `mujoco_ros2_control` package --- in place of the normal one --- with some arguments:

- `use_sim_time` is set to `True`.
- Specify the `controllers.yaml` file.
- Optionally specify the `mujoco_plugins.yaml` file, which loads in any defined plugins.

This Node is also responsible for starting up the MuJoCo GUI application, once everything starts up and configures properly.

Now, if everything is configured properly thus far, it should be possible to launch the simulation file and see MuJoCo running!

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
