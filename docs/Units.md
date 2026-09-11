---
title: Units
parent: ROS2 Control
nav_order: 6
---

## ROS2 Control's Weird Units

If you are getting started writing a controller or hardware component, you might be thinking to yourself, "what units does ROS2 Control send commands in? I can't find documentation on this anywhere." Correct! There isn't really documentation on this (that is easily accessible), because ROS2 Control is unit-agnostic. It is up to the programmer to convert its values into real units for the program to use.

The [controller]({% link docs/Technical/ROS2/Jazzy/ROS2 Control/Controllers.md %}) for the ROS2 Control stack is what determines the units of everything. However, **this is not as open as it sounds like.** There are certain standards of units associated with types of joints in the URDF.

- **revolute/continuous joints should be radians and radians/second**
- **prismatic joints should be meters and meters/second**
- Fixed does not have any relevance

You should convert these metric units to more hardware specific units like PWM or 0-127 or encoder positions or whatever the hardware demands in the [hardware component]({% link docs/Technical/ROS2/Jazzy/ROS2 Control/Hardware-Components.md %}). These values should then be forwarded to the microcontroller where it will blindly accept them as the right units to control the motors.

## Control Stack Unit Walkthrough

joystick movement -> joy node (unitless floats ranging from -1.0 to 1.0 representing stick's axis) -> teleop_twist_joy or similar multiplies the unitless -1.0 to 1.0 by maximum speed -> publishes this to cmd_vel in m/s or rad/s -> controller manager is running the robot_controller and subscribes to cmd_vel -> controller takes m/s and rad/s and applies kinematics (now in joint specific units, like how many meters this specific LA should extend) -> controller writes results into resource manager variables like left_wheel/velocity -> the hardware component executes the write() loop, which reads from the resource manager memory and converts it to things like PWM or RPM or position, etc -> Teensy forwards commands blindly to motors

(TODO: make a diagram for that nonsense)

Basically the joystick pipeline outputs **desired units**, the controller pipeline outputs the **necessary units (joint SI units) to achieve desired units**, and lastly the hardware component pipeline outputs the **necessary raw hardware units to achieve desired units**.

The joystick output of -1.0 to 1.0 is an EFFORT value, so when you multiply it by its maximum speed, you get the actual speed it should be going. So, if you want to go 50% speed, you just multiply 0.50 x max_speed. **This is where the initial SI conversion happens**. It is a REP standard (specifically REP 103) that requires the joysticks max speed to be defined in SI units, so multiply this effort value by the max speed in SI units is where everything actually gets a unit. This is why when the controller gets it, it is already in SI, and it can go straight to calculating kinematics.

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
