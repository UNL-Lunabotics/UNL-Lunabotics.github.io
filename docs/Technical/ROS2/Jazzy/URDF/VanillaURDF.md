---
title: Vanilla URDF
parent: URDF
nav_order: 1
---

## Xacro

xacro is just a way to do XML Macros. It works with URDF and is HIGHLY recommended. It allows you to do things like write your URDF across multiple files instead of only one, have conditionals, use variables, evaluate mathematical expressions, and accept launch parameters. There is no good reason not to use xacro just use it.

Any xacro files will need to import xacro (shown in [In File Structure](#in-file-structure)). It is just assumed that you will be using xacro so it is part of the basic setup process.

Notably, it doesn't really matter what extension you use so if you create a `file.xacro` or a `file.urdf` or a `file.xacro.urdf` or a `file.urdf.xacro` all of them will work just fine. There are conventions that will be mentioned later.

[GitHub Repository](https://github.com/ros/xacro)
[GitHub Wiki](https://github.com/ros/xacro/wiki)

## Structure (with Xacro)

### File Structure

When working with URDF, you will absolutely want to split up the URDF between multiple files. All of this should be contained inside a `description/urdf/` ROS2 package (it can be a Python package since nothing here is compiled).

You should have one main file that is `robotname.urdf.xacro` and this one will be the one read for the topic robot_description and will import all the other files you make. Every other file you make should follow the convention of `name.xacro`. The `.urdf.xacro` helps identify which one is the main file. It is recommended that the master file contains all the setup. This includes accepting launch arguments, handling conditional logic when applicable, defining any needed variables, and including all the other files. It should not include any actual link/joint logic **except you should define your `base_link` here**.

You should have an individual `gazebo.xacro` and `ros2_contro.xacro` file. All of the necessary information for Gazebo, including basic color tags, should be included here. The ROS2 Control xacro should include ALL the necessary information both for teleop, teleop, AND simulated versions of both (which means this xacro will include some Gazebo information necessary to make ROS2 Control work with it).

Depending on how many sensors you have, you could make a master `sensors.xacro`, or if you have a lot of sensors you could make individual files for each. For example, `camera1.xacro`, `camera2.xacro`, `lidar.xacro`, etc. It is up to personal preference.

Depending on how many variables you have to keep track of, it might be worth it to create a `config.xacro` file whose only purpose is to hold variables. This is usually not necessary but an option.

The actual link/joint information for your bot should be stored in `robotname_core.xacro`. If you have a really complicated robot core with many different manipulators, you can also split those up. The `robotname_core.xacro` should always contain at least the chassis and usually the wheels, and the arms or manipulators of any sort can be moved elsewhere if necessary. I recommend keeping it all in this file unless it becomes excessive.

Inside the `description/` folder, there will also be another folder for meshes. This is where the collision and visual info for the robot will be exported into. This will be needed later when defining a link.

Your final file structure should be something like

```txt
description/
    urdf/
        robotname.urdf.xacro
        robotname_core.xacro
        sensors.xacro
        ros2_control.xacro
        gazebo.xacro
    meshes/
        mesh.STL
        moremesh.STL
```

### In File Structure

First, you should always specify which version of XML and what UTF encodings it uses at the top of the file. Line one in every URDF/xacro file should be `<?xml version="1.0" encoding="utf-8"?>`. This does not need to be closed, it is a one line thing.

Next, you need to declare the robot tag and import xacro. The robot tag is kind of like a class tag, EVERYTHING goes inside it. You should do this in every file no matter if it's specifically for ROS2 Control or the regular robot description.

```XML
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    your stuff goes here :)
</robot>
```

Optionally, you can give the robot a name. This isn't really important at all but you can if you want to. It would just change the line to `<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="Robot Name">`.

#### Example: robotname.urdf.xacro

```XML
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Declare any variables (including ones set by launch files) -->
    <xacro:arg name="use_sim" default="false"/>
    <xacro:arg name="use_control" default="false"/>



    <!-- Define base_link -->
    <link name="base_link"/>        <!-- You could also declare base_sensor here if you decide to use that -->



    <!-- Include needed files (utilizing conditions) (these are relative filepaths) -->
    <xacro:include filename="robotname_core.xacro"/>
    <xacro:include filename="sensors.xacro"/>

    <xacro:if value="$(arg use_control)">
        <xacro:include filename="ros2_control.xacro"/>
    </xacro:if>

    <xacro:if value="$(arg use_sim)">
        <xacro:include filename="gazebo.xacro"/>
    </xacro:if>

</robot>
```

#### Example: robotname_core.xacro

Notably, this example also works if you split the robot core into separate files (for example, one of chassis and multiple for manipulators). Basically, any comment block in the below example could be its own file if you wanted. **Keep the chassis in robotname_core.xacro**.

```XML
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Define the joint/link for chassis -->

    <!-- Define the joint/link for the wheels -->
    
    <!-- Define the joint/link for any manipulators -->

</robot>
```

#### Example: sensors.xacro

This is basically identical to `robotname_core.xacro`. You should view the specific documentation (there is a separate page) on specifically how to define sensors.

```XML
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Define the joint/link for the sensors -->

</robot>
```

#### Example: ros2_control.xacro

```XML
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Load the real robot ROS2 Control information -->
    <xacro:unless value="$(arg sim_mode)">
        <!-- Almost everything should go in this ros2_control tag -->
        <ros2_control name="name" type="system">  <!-- This should almost always be system -->

            <hardware>  <!-- This defines all the params needed for the hardware component -->
            <!-- This includes wheel names, which serial device/port to listen to, baud_rate, loop_rate, etc -->
            </hardware>

            <!-- Define all the joints that will have a command or state interface -->

        </ros2_control>
    </xacro:unless>



    <!-- Load the ROS2 Control information for just sim purposes -->
    <xacro:if value="$(arg use_sim)">
        <ros2_control name="name" type="system">

        <hardware>
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>

        <!-- Define all joints and their command/state interfaces necessary for sim -->

        <gazebo>
            <plugin name="gz_ros2_control::GazeboSimROS2ControlPlugin" filename="libgz_ros2_control-system.so">
                <parameters>$(find control)/config/controllersname.yaml</parameters>
            </plugin>
        </gazebo>

        </ros2_control>
    </xacro:if>

</robot>
```

#### Example: gazebo.xacro

```XML
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Add any relevant data to links/joints including color and friction values -->
    <gazebo reference="chassis_link">
        <material>Gazebo/White</material>
    </gazebo>

    <gazebo reference="Castor">
        <material>Gazebo/Blue</material>
        <mu1 value="0.0"/>  <!-- This is friction -->
        <mu2 value="0.0"/>
    </gazebo>

    <!-- Define Gazebo info for any sensors -->
    <gazebo reference="laser_frame">
        <material>Gazebo/Red</material>
        <sensor name="laser" type="gpu_lidar">
            <pose> 0 0 0 0 0 0 </pose>
            <visualize>true</visualize>
            <update_rate>10</update_rate>
            <lidar>
                <scan>
                    <horizontal>
                        <samples>360</samples>
                        <min_angle>-3.14</min_angle>
                        <max_angle>3.14</max_angle>
                    </horizontal>
                </scan>
                <range>
                    <min>0.3</min>
                    <max>12</max>
                </range>
            </lidar>
            <topic>scan</topic>
            <gz_frame_id>laser_frame</gz_frame_id>
        </sensor>
    </gazebo>

</robot>
```

#### Example: sensor.xacro

```XML
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Define the joint/link for all the sensors -->
    <!-- Specific sensor implementation details are in a separate page -->

</robot>
```

## Elements

TODO

## Conditionals (with Xacro)

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
