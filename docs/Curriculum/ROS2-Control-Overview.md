---
title: ROS2 Control Overview
parent: Curriculum
nav_order: 1
---

## ROS2 Control Overview

ROS2 Control is a library for exactly what it sounds like, controlling the robot. It handles things from kinematics to joystick input to communicating with the microcontrollers and other hardware. It is a large topic to tackle, kind of complicated, and the documentation on the topic is also not very good at all when it comes to actual implementation. The conceptual explanations for ROS2 Control are well done.

Firstly, read this documentation page starting at [Architecture](https://control.ros.org/jazzy/doc/getting_started/getting_started.html#architecture) and ending before the Hardware Description in URDF section.

Basically, there are three main parts of ROS2 Control that are relevant for our purposes: Controllers, interfaces, and hardware components. All of these parts are managed by the Controller Manager, which is mostly relevant for writing launch files. This documentation aims to inform you just enough to be able to write ROS2 Control code, but not enough to understand the intricate workings of it. There are some parts of the ROS2 Control feature base that you can only really understand by looking at the source code, so a comprehensive explanation of features is not feasible.

## Joint Interfaces

Interfaces are a really important concept both in URDF design and in understanding how ROS2 Control views the system. You will not find a whole page dedicated to interfaces in the ROS2 Control docs, so we are explaining them in detail here.

Interfaces are attributes that you give to joints that are controlled by motors, servos, or otherwise move in the ROS2 Control URDF. Generally, to determine what joints need interfaces, ask yourself what you actually need to control on the robot. More details can be found at [ROS2 Control URDF]({% link docs/Technical/ROS2/Jazzy/URDF/ROS2-Control-URDF.md %}). There are two kinds of interfaces: command interfaces and state interfaces. An example is given below:

```XML
<joint name="Wheel">
    <command_interface name="velocity"/>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
</joint>
```

A **state interface** is information you should be allowed to be READ from a joint. Typically, this comes in the form of a position and velocity state interface, which means that during the robots' control loop, you can read what the position and velocity of this joint is (typically through encoders).

A **command interface** is information you should be allowed to WRITE to a joint. This also usually comes in the form of a position and velocity command interface. So, during the robots' control loop, you can command the robot to go this specific velocity or go to this specific position. This is how you actually control the robot.

In the controller, you will read the state interfaces and based on the logic you have written (and probably joystick or navigation stack input) you will output to the command interfaces. The hardware component will handle the specific low-level implementation of how exactly you write/read from the hardware level.

## Controllers

Controllers are where the vast majority of functionality with ROS2 Control you write will be located. They are both an incredibly broad and incredibly narrow topic. Controllers are simply where you define the behavior you want your robot to exhibit while you give it commands. For example, if I have just a simple XBox controller, and I press the 'B' button, the controllers are where I define the behavior I want to happen after the 'B' button is pressed. If I want to open the robot arm claw by pressing 'B', I could write out some logic in my controller that says that when the 'B' button is pressed, change the command interface for the claw's position to be open.

Controllers can be for a whole robot or for specific parts of the robot. For example, you could have a controller just to control the drivetrain of the robot and another controller for all the attachments, or you could put all of these in the same controller for one super controller. Things can get really complicated and you can have chained controllers which we will not explore in depth as the only way you can figure out how they work is by looking at the source code.

There are also a vast amount of pre-made controllers that you don't even have to write, but those are really easy to look up and there is extremely little documentation on how to write custom controllers, so we will be talking about custom controllers the entire time.

More details on controllers (including actual code examples) can be found at [Controllers]({% link docs/Technical/ROS2/Jazzy/ROS2 Control/Controllers.md %})

## Hardware Components

Hardware components are exactly what they sound like, the hardware component of this control stack. The hardware component is the abstraction layer between the controllers and the actual hardware of the robot. It is the thing that reads from encoders, tells the motors how fast to spin, and more. There will be a lot more going on with this, so you can read more about it at [Hardware Components]({% link docs/Technical/ROS2/Jazzy/ROS2 Control/Hardware-Components.md %}).

Notably, this hardware component communicates with the microcontroller for the system via serial ports. The microcontroller is what ultimately has to read encoder values and send PWM signals to motors, it is a middleman between the hardware component and the actual hardware since the computers cannot communicate on a low enough level.

## ROS2 Control's Weird Unit Handling

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
