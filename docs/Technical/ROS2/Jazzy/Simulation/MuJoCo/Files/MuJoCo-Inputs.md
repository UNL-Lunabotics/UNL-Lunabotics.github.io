---
title: MuJoCo Inputs File
parent: Files
nav_order: 2
---

## MuJoCo Inputs

A separate `mujoco_inputs.xacro` will need to be defined as well, which links to the main URDF. This file handles any preprocessing MuJoCo ROS2 Control does before it starts up. This file has two main parts.

### Raw Inputs

Raw inputs define any arbitrary MJCF that will be copied into the generated MJCF from MuJoCo ROS2 Control (more on this later). Here, you can define any `<actuator>`, `<sensor>`, `<extension>`, or `<default>` tag. For most cases, `<default>` and `<actuator>` tag(s) must be defined. A comprehensive example is as follows:

```xml
<!-- Robot URDF/Xacro File -->
<mujoco_inputs>
  <!-- The contents of `raw_inputs` will be copied and pasted directly into the MJCF -->
  <raw_inputs>
    <option integrator="implicitfast"/>
    
    <!-- Required. Segfaults without -->
    <default>
      <default class="visual">
        <geom group="2" type="mesh" contype="0" conaffinity="0"/>
      </default>
      <default class="collision">
      <geom group="3" type="mesh"/>
      </default>
    </default>
    
    <!-- Matches any <joint> in the ROS2 Control configuration -->
    <actuator>
      <velocity name="front_left_wheel_joint" joint="front_left_wheel_joint" />
      <velocity name="front_right_wheel_joint" joint="front_right_wheel_joint"/>
    </actuator>
    
    <!-- Only required for LiDAR setup -->
    <extension>
      <plugin plugin="mujoco.plugin.lidar">
        <instance name="lidar">
          <config key="resolution" value="360 1"/>
          <config key="azimuth_range" value="-3.14159 3.14159"/>
          <config key="elevation_range" value="0.0"/>
          <config key="max_range" value="10.0"/>
          <config key="min_range" value="0.1"/>
        </instance>
      </plugin>
    </extension>
    
    <!-- Define sensors -->
    <sensor>
      <plugin name="lidar" instance="lidar" objtype="site" objname="lidar_link_optical" />
    </sensor>
  </raw_inputs>
```

Here, we set visual and collision groups inside a `<default>` tag. Without this, MuJoCo ROS2 Control errors out with a segmentation fault.

Then, `<actuator>` tags are defined, which should match the `<joint>` tags in the ROS2 Control section.

Then, an `<extension>` tag is used to add a LiDAR sensor in. This comes from the [3D Lidar Plugin](https://control.ros.org/jazzy/doc/mujoco_ros2_control/mujoco_ros2_control_plugins/doc/plugins.html#mujoco-3d-lidar-plugin) (more on this later).

Finally, a `<sensor>` tag defines any optional sensors to be used.

### Processed Inputs

Processed inputs are largely optional, unless a camera is used. These inputs define any `<modify_elements>` or `<camera>` tag. An example is as follows:

```xml
<!-- Robot URDF/Xacro File -->
<mujoco_inputs>
  <!-- Specific inputs that require processing from the conversion script -->
  <processed_inputs>
    <!-- The camera with the specified values will be added at the specified site name.
    The position and quaterinion will be filled in by the converter -->
    <camera site="camera_link_optical" name="camera" fovy="58" mode="fixed" resolution="640 480"/>
    
    <!-- The element with the type and name as listed below will be modified with the remaining attributes -->
    <!-- If an attribute already existed, it will be overwritten -->
    <!-- Note that the type and name elements are required -->
    <modify_element type="joint" name="front_left_wheel_joint" damping="0.1" armature="0.1"/>
    <!-- Other `<modify_elements>` -->
  </processed_inputs>
</mujoco_inputs>
```

Here, we set a `<camera>` tag. While camera is a sensor, it needs to be processed by MuJoCo ROS2 Control because MuJoCo doesn't know what a ROS2 link is. We set the "site" to the name of the link the camera is attached to, as well as other optional configuration.

Then, we set a `<modify_element>` tag, saying that we want the "front_left_wheel_joint" to have additional attributes. These tags are completely optional.

There are a lot of other options as well. To learn more, visit the [Embedding MuJoCo Inputs inside URDF](https://control.ros.org/jazzy/doc/mujoco_ros2_control/mujoco_ros2_control/docs/tools.html#embedding-mujoco-inputs-inside-urdf) documentation.

> Author: Aiden Kimmerling <https://github.com/TheKing349>
