---
title: ROS2 Control in URDF
parent: URDF
nav_order: 5
---

## ROS2 Control with Xacro

A LOT of ROS2 Control depends on having a solid URDF to base it on. This URDF is used by the hardware component, the controllers, and Gazebo. If you don't know what that means, visit the ROS2 Control technical document.

This page will describe the different components of a `ros2_control.xacro` file and the different ways you can go about structuring it.

## Basic Structure

Taken from URDF with Xacro, here is a basic scaffolding for the file that we will be expanding on in the rest of this page.

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

## ROS2 Control for the Physical Rover

Let's dive into the real robot ROS2 Control information section. This is the part that the hardware component and controllers will need. Gazebo does not access this part. A lot of this information was pulled from the [Hardware Component Overview](https://control.ros.org/rolling/doc/getting_started/getting_started.html#overview-hardware-components) ROS2 Control wiki page. Unfortunately, there is very little documentation on what the controllers need from this section.

```XML
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
```

Note that the xacro unless is not strictly necessary if you are not planning on using Gazebo, but in general it should always be there. The arg name is subject to change depending on the project.

Let's examine the very first tag first, `<ros2_control name="name" type="system">`

First, notice the ros2_control tag. This is where you will describe ALL of the relevant ros2 control information. It is kind of like the robot tag in that way. Second, notice that we declare the ros2_control tag as type system. There are two other types, actuator and sensor, but you will likely never really use them unless you split up the URDF a LOT (which isn't necessary). A type of system means that it has multiple Degrees of Freedom (DOF) and a lot of moving parts. This is why we use it so much, since it basically encapsulates all of a robotics system. Sensor is exactly what it sounds like, link/joint pairs only and read only. You wouldn't use this unless you were writing ros2 control information for just sensors, but that isn't really useful because you can just write it all in the same ros2 control block as everything else if you use system. Actuator is used with precisely one joint, which makes it only useful if you are making each joint individually controllable by a controller (why).

The rest of the ros2_control tag will have two parts: the hardware tag and the joint declaration.

The hardware tag contains all the information needed for the hardware component. The very first tag inside the hardware tag specifies which hardware interface you are using. This is specifically the one exported in the `robot_hwc.xml` and should look something like `<plugin>robot_hwc/RobotHWC</plugin>`. This should match the path exported by the interface XML exactly. The rest of the information in the hardware tag typically includes information for the microcontroller like the device (usually `/dev/ttyACM0`), the baud rate, etc. They will all be declared like `<param name="device">/dev/ttyACM0</param>`.

The joint declarations are just taking any joint from the base URDF that will move and giving it interfaces and limits. For example,

```XML
<joint name="Wheel">
    <command_interface name="velocity">
    <param name="min">-10</param>
    <param name="max">10</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
</joint>
```

Command interfaces are inputs that this joint can receive. For the above example, it has both a velocity and position command interface, which means we can send it velocity and position commands (via PWM or encoder positions). State interfaces are things that you can read from the joints, so giving it a position and velocity state interface means that we can read the position and velocity it is current at (with encoders usually). You should give any joint that moves a declaration here and the name needs to match what was declared in the other URDF file it was defined in.

## ROS2 Control for Gazebo Sim

Now, let's examine the second half of the example URDF. A lot of the boilerplate below should not change from project to project. The plugin will be the same for every robot (in this version of ROS2 and Gazebo) and the setup for the gazebo plugin will also be the same.

```XML
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
```

Really, the only thing for you to do here is define the joints the same way that you do in the physical robot section. You define a joint for every movable joint in the actual descriptive URDF and give it command and state interfaces. There is more detail about this in the above section.

The hardware plugin section should be the same as long as you are using Jazzy, and the main thing to note besides the joints is that you need to substitute the actual controllers yaml in `<parameters>$(find control)/config/controllersname.yaml</parameters>`. This should be your main controller, not just for joysticks or anything like that.

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
