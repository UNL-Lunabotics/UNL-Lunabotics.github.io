---
title: MuJoCo Plugins File
parent: Files
nav_order: 3
---

## MuJoCo Plugins File

An optional `mujoco_plugins.yaml` configuration file can be defined to register any MuJoCo ROS2 Control plugins. An example is as follows:

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      mujoco_3d_lidar_plugin:
        type: "mujoco_ros2_control_plugins/Mujoco3dLidarPlugin"
        lidar:
          frame_name: lidar_link_optical
          topic: /scan
```

This file must start with the first three lines of code:

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      # Plugin name here...
        # type: ...
        # Plugin parameters here...
```

After that, each plugin is defined by a name and a type. The name can be changed, while the type must reference the exact plugin path. Then, specify the plugin's parameters, which will be different from plugin to plugin.

Another example with multiple plugins is shown here:

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      mujoco_3d_lidar_plugin:
        type: "mujoco_ros2_control_plugins/Mujoco3dLidarPlugin"
        lidar:
          frame_name: lidar_link_optical
          topic: /scan
      mujoco_camera_plugin:
        type: "mujoco_ros2_control_plugins/CameraPlugin"
        camera_publish_rate: 6.0
        camera:
          policy: streaming
          frame_name: camera_link_optical
          info_topic: /camera/camera_info
          image_topic: /camera/image
          depth_topic: /camera/depth_image
```

Here, a 3D LiDAR plugin, as well as a Camera plugin are defined, both from the `mujoco_ros2_control_plugins` package. Each plugin has their own parameters to define.

There are other plugins as well. For a full list, see the [MuJoCo ROS2 Control Plugins](https://control.ros.org/jazzy/doc/mujoco_ros2_control/mujoco_ros2_control_plugins/doc/plugins.html) documentation.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
