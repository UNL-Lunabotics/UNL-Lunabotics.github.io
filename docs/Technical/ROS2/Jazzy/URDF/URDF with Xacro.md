---
title: URDF with Xacro
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

### Example: robotname.urdf.xacro

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

### Example: robotname_core.xacro

Notably, this example also works if you split the robot core into separate files (for example, one of chassis and multiple for manipulators). Basically, any comment block in the below example could be its own file if you wanted. **Keep the chassis in robotname_core.xacro**.

```XML
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Define the joint/link for chassis -->

    <!-- Define the joint/link for the wheels -->
    
    <!-- Define the joint/link for any manipulators -->

</robot>
```

### Example: sensors.xacro

This is basically identical to `robotname_core.xacro`. You should view the specific documentation (there is a separate page) on specifically how to define sensors.

```XML
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Define the joint/link for the sensors -->

</robot>
```

### Example: ros2_control.xacro

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

### Example: gazebo.xacro

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

### Example: sensor.xacro

```XML
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Define the joint/link for all the sensors -->
    <!-- Specific sensor implementation details are in a separate page -->

</robot>
```

## Xacro Elements and Features

### Tags

XML works kind of like HTML so it has a variety of different tags that you can open with `<tag>` and close with `</tag>` or do it all in one line with `<tag/>`. The specific tags you need are highly context dependent, so please refer to the above examples to see some. In general, unless it's an int, and even sometimes when it is an int, everything should be in quotes when giving something a value. That is basically it, there are elements and values and that is 90% of a URDF.

### Property, Property Blocks, and Args

Xacro properties are equivalent to regular variables in any other programming language. A property block is something that has more than one value, so more similar to an object in an object orientated programming language. After you create a property or property block, you use `"${property_name}"` to evaluate it.

An example of using properties is shown here (taken from the xacro wiki):

```XML
<xacro:property name="the_radius" value="2.1" />
<xacro:property name="the_length" value="4.5" />

<geometry type="cylinder" radius="${the_radius}" length="${the_length}" />
```

An example of using a property block is shown here (taken from the xacro wiki):

```XML
<xacro:property name="front_left_origin">
  <origin xyz="0.3 0 0" rpy="0 0 0" />
</xacro:property>

<wheel name="front_left_wheel">
  <xacro:insert_block name="front_left_origin" />
</wheel>
```

The above example actually utilizes a macro, which is expanded upon later in [Macros](#macros)

### Math Expressions

When using xacro and inside a tag, you can do `"${math equation}"` in order to do evaluate that value. This could be something like `"${pi/2}"`. This is extremely helpful when calculating a LOT of things, especially rotations and inertia.

Xacro (in recent veresions) specifically uses the [Python math module](https://docs.python.org/3/library/math.html) in order to compute this, so anything available there is available inside a `"${math equation}"` expression. This includes a lot of functions and constants like pi, e, inf, and nan.

Math operators like *, /, +, and - are available.

### Conditional Blocks

Xacro allows two conditional blocks: if and unless. If is a standard if conditional and unless executes if the given value is NOT true. These expresions accept values of 1, 0, true, or false.

```XML
<xacro:if value="expression or var">
  <... some xml code here ...>
</xacro:if>
<xacro:unless value="expression or var">
  <... some xml code here ...>
</xacro:unless>
```

This is particularly useful when dealing with Gazebo. Typically, as shown in some previous xacro examples, the Gazebo file is only included and loaded if a value like use_sim is true.

### Rospack commands

Xacro supports all of the rospack commands that you use in [roslaunch](https://wiki.ros.org/roslaunch/XML#substitution_args). Particularly, substitution arguments are incredibly important. Substitution arguments are variables that can be set by the launch files. This is where we get values like use_sim or use_control.

You can declare them like this `<xacro:arg name="myvar" default="false"/>` and evaluate them at any time with `"$(arg thingsname)"`.

Typically, they are declared in the main `urdf.xacro` file and can be used in any other file that the main file includes.

Additionally, one of the main uses for these rospack commands is to find meshes. You can use the find keyword in this context and then put a ros2 package file path. For example, if I have the ros2 package description and it has a folder named meshes/ with meshes inside of it, I could use the below argument to give the chassis its mesh. You have to preface this all with file:// which is the modern solution in Jazzy (older versions are different).

```XML
<mesh filename="file://$(find rover_description)/meshes/chassis.STL"/>
```

A full practical example of this is shown below:

```XML
<collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
        <mesh filename="file://$(find rover_description)/meshes/chassis.STL" />
    </geometry>
</collision>
```

There are other commands which you can view at the hyperlink at the start of this section, but these are the most common and most useful ones.

### Macros

One of the most powerful things you can do in xacro is use macros, but it is also highly unlikely you will actually end up using them. This is because most URDFs will be exported without them, and you would have to go in and manually edit things a LOT in order to utilize them to the fullest.

It is easiest to conceptualize macros as just a way to declare objects in xacro. They can do other things, but one of the main uses is to be able to reuse a lot of code. You can declare a macro named 'wheel' with a lot of different parameters, and later in the URDF you can use this macro, give it the parameters it needs, and not have to rewrite everything but slightly different for every wheel.

You can write out the code and only accept value params, or you can accept whole blocks of XML as a param.

Here is an example using just value params:

```XML
<!-- Create a simple box xacro object with params width height depth -->
<xacro:macro name="box" params="width height depth">
  <link name="generated_link">
    <visual>
      <geometry>
        <box size="${width} ${height} ${depth}"/>
      </geometry>
    </visual>
  </link>
</xacro:macro>

<!-- Create a box using this macro -->
<xacro:box width="2" height="1" depth="0.5"/>
```

An example of using whole XML block params is given below:

```XML
<!-- Create the macro (aka object) with params origin, content, and anothercontent -->
<xacro:macro name="pr2_caster" params="suffix *origin **content **anothercontent">
  <joint name="caster_${suffix}_joint">
    <axis xyz="0 0 1" />
  </joint>
  <link name="caster_${suffix}">
    <xacro:insert_block name="origin" />
    <xacro:insert_block name="content" />
    <xacro:insert_block name="anothercontent" />
  </link>
</xacro:macro>

<xacro:pr2_caster suffix="front_left">
  <pose xyz="0 1 0" rpy="0 0 0" />
  <container>
    <color name="yellow"/>
    <mass>0.1</mass>
  </container>
  <another>
    <inertial>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
    </inertial>
  </another>
</xacro:pr2_caster>
```

There is more to macros but it is unlikely you will ever have to use them heavily, so don't worry about it. If you do need to do more research, you can start at the [ROS Xacro Deep Wiki](https://deepwiki.com/ros/xacro/5.1-macros-and-parameters).

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
