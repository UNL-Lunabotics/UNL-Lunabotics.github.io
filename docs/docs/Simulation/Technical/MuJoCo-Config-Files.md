---
title: MuJoCo Config Files
parent: Technical
nav_order: 2
---

## Table of contents

{:toc}

## Files

There are different files used to get MuJoCo ROS2 Control working properly, from URDF to configuration and launch files.

See the Table of Contents to get started.

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
Currently, the documentation says to configure `<sensor>` tags inside the ROS2 Control hardware interface. This has begun to be deprecated in favor of a separate plugin setup (more on this later). Unless there is no other way, do not define any `<sensor>` tags here.

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

## MuJoCo Scene File

MuJoCo scene files, known as an MJCF, defines which objects are placed in the world when the simulation is running. It is very configurable and customizable, and similar to a URDF.

### Basics

Each MJCF will be a little different, depending on how it is set up, but every MJCF will start with the following:

```xml
<mujoco model="model_name">
  <!-- Content here -->
</mujoco>
```

Then, you can optionally define any `<compiler>`, `<visual>`, `<default>`, or `<option>` tags if needed. These would specify global values such as gravity, global friction, etc.:

```xml
<mujoco model="example">
  <compiler angle="radian" />
  <option timestep="0.002" gravity="0 0 -9.81" />
  <default>
    <geom friction="0.01 0.01 0.01" condim="4" />
    <joint damping="0.1" frictionloss="0.01" />
  </default>
</mujoco>
```

Here, we specify to use radians as our angles, then define friction, gravity, and the timestep.

Then, to add objects into the world, add a `<worldbody>` tag, then `<body>` tags, like the following:

```xml
<mujoco model="example">
  <worldbody>
    <body name="ground_plane" pos="0 0 0">
      <inertial mass="1" pos="0 0 0" diaginertia="1 1 1" />
      <geom type="plane" size="100 100 0.1" friction="0.01 0.01 0.01" />
    </body>
  </worldbody>
</mujoco>
```

Here a `<body>` tag is set with the name of "ground_plane", positioned at the world origin. Each `<body>` can have a `<geom>` and `<intertial>` tag, defining the type of object it is, along with its visual and collision. In the example, we set the object type to a plane, with a mass of 1, size of 100 on the x and y, and set some friction. Note that multiple `<body>` tags can be specified.

A complete simple example is as follows:

```xml
<mujoco model="example">
  <worldbody>
    <body name="ground_plane" pos="0 0 0">
      <inertial mass="1" pos="0 0 0" diaginertia="1 1 1" />
      <geom type="plane" size="100 100 0.1" friction="1.0 0.8 0.01" />
    </body>
    <body name="box" pos="2.8417219933466216 2.3179857352986479 0.49999999990199806">
      <inertial mass="1" pos="0 0 0" diaginertia="0.16666 0.16666 0.16666" />
      <geom type="box" size="0.5 0.5 0.5" friction="1.0 0.8 0.01" />
    </body>
    <body name="sphere" pos="1.7558753273133232 -0.14177392862852622 0.499999999902001">
      <inertial mass="1" pos="0 0 0" diaginertia="0.1 0.1 0.1" />
      <geom type="sphere" size="0.5" friction="1.0 0.8 0.01" />
    </body>
    <body name="cone" pos="-0.45892397368634047 -1.1120748308789254 0.49999999993296756">
      <inertial mass="1" pos="0 0 0" diaginertia="0.075 0.075 0.075" />
      <geom type="cylinder" size="0.5 0.5" friction="1.0 0.8 0.01" />
    </body>
    <body name="cylinder" pos="-3.4181480821354935 0.11779292196676421 0.49999942638036005">
      <inertial mass="1" pos="0 0 0" diaginertia="0.14580 0.14580 0.125" />
      <geom type="cylinder" size="0.5 0.5" friction="1.0 0.8 0.01" />
    </body>
  </worldbody>
</mujoco>
```

Here, a plane, box, sphere, cone, and cylinder are defined with mass, size, friction and position values.

### Including other MJCF

For more complicated worlds, it may be preferable to split up the MJCF into different files. To do this, define another MJCF, then in the main MJCF, add the following:

```xml
<mujoco model="example">
  <include file="/path/to/other.mjcf" />
</mujoco>
```

Then, MuJoCo will reference the `other.mjcf`, found in the path, when launching the simulation. The file path can be relative or absolute.

### Defining Meshes and Materials

Instead of predefined simple objects or colors, you can include meshes and materials to get custom shapes and colors. MuJoCo works best with `.obj` files for meshes, and `.mtl` files for materials, though others may work. Note that `.stl` files are not supported.

{: .note}
While MuJoCo does STL files, there seems to be some compatibility issues with `mujoco_ros2_control`, and it may error out when launching. It is preferable to use `.obj` files instead.

To add a mesh to an MJCF, first make the mesh available to be used:

```xml
<mujoco model="example">
  <asset>
    <material name="MyCustomMaterial" specular="0.5" shininess="0.5" rgba="1 1 1 1.0" />

    <mesh file="my_custom_mesh.obj" />
    <mesh file="/path/to/another_custom_mesh.obj" />
  </asset>
</mujoco>
```

Here we set an `<asset>` tag to define any custom meshes or materials. For materials, specify the name, then the material properties, such as color. For meshes, set the file path, relative or absolute.

Then, a `<body>` tag can reference these later:

```xml
<mujoco>
  <worldbody>
    <body name="my_body" pos="3 1 1.5">
      <geom material="MyCustomMaterial" mesh="my_custom_mesh" class="visual" />
      <geom mesh="another_custom_mesh" rgba="0.3 0.0 0.3 1" class="collision" />
    </body>
  </worldbody>
</mujoco>
```

Here, we set a material and mesh geometry. The material references a mesh to use and will be used by the entire `<body>` tag. The mesh defines an individual mesh to use as part of the `<body>`. Note that multiple mesh geometry can be defined.

There is a lot of other configuration that can be specified. To learn more, visit the [MuJoCo XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html) documentation.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
