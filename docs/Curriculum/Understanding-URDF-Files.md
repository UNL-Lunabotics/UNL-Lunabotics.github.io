---
title: Understanding URDF Files
parent: Curriculum
nav_order: 3
---

## Understanding URDF Files

Unified Robot Description Format (URDF) is the standard for representing a robot in ROS2. It serves as a machine-readable blueprint that allows ROS2 programs to "visualize" your robot in order to carry out its various tasks, such as control, sensing, and autonomy.

The purpose of this guide is to help you understand the very basics of what a URDF represents, how they work, and how to get started with creating a basic URDF of a robot from scratch. As part of this process, this guide will spend a lot of time going over individual URDF tags and what they are used to represent. For a more detailed look at URDF standards and features, as well as guides on URDF integration for different libraries and softwares that work alongside ROS2, see our technical [URDF documentation]({% link docs/Technical/ROS2/Jazzy/URDF/index.md %}). Note that documentation was written for ROS2 Jazzy, so if you are using a different version, there may be differences in structure.

The robot model you will construct in this guide is heavily based on a [video tutorial](https://www.youtube.com/watch?v=BcjHyhV0kIs&t=1292s) made by [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics) in 2022. While this is a fantastic guide and I strongly recommend watching it, the tutorial was made for ROS2 Foxy (EoL 2023), making it a little outdated. This guide will use a very similar model with some tweaks to the overall design to make it a little easier to understand, as well as bringing it in line with ROS2 Jazzy standards, as a few things have changed in the newer versions of ROS2 that can influence how your URDF is structured.

By the end of this guide, you should have a basic model of a robot in URDF format. If you visualize your URDF, either using a VSCode extension, running your ROS2 package with a launch file, or some other means, you should get a 3D model that looks something like this:

![Render of Finished URDF]({% link attachments/urdf/TootleFinished.png %}){: width="49%" }
![Top-down Render of Finished URDF]({% link attachments/urdf/TootleFinishedTop.png %}){: width="49%" style="margin-left:1%;" }

*Note: I am using Foxglove Studio to render the URDF for this tutorial, if you are using something else, your model may look slightly different, but it should maintain the same core shapes*

## What is a URDF?

As previously stated, URDF, which stands for Unified Robot Description Format, is the standard for representing a robot's physical model through a ROS2 topic. When you launch your ROS2 package, the `/robot_description` topic publishes the information from your URDF and can be subscribed to and rendered by a visualizer like Rviz, Foxglove, or Rerun. These files effectively specify the size and shape of the robot, which is very helpful when it comes to things like remote navigation and autonomy.  

The URDF file is also used to specify the location of any cameras and sensors on the robot, which is a critical requirement for localization. If the robot does not know where sensor data is coming from relative to itself, it cannot use that data to orient itself or navigate autonomously.

## Structure of a URDF

URDF files use Markup Language syntax (similar to languages like XML and HTML) to define a series of elements that go on to define components of your robot, with each individual element defining a different "part" of the robot (i.e. wheels, chassis, a robotic arm, cameras, sensors, etc.). Each element has its own configurations, and its own specified placement in the final model. Each of these elements come together when ROS2 builds your URDF to define a complete model of your robot, either for the purposes of simulation in software such as Gazebo or MuJoCo, or for use in managing an actual robot.

While you can place all of your components into one large URDF file, this is generally not good practice, as they can get very large and difficult to manage relatively quickly. Instead, its a good idea to use Xacros (XML Macros) to split your URDF into multiple smaller files that can be compiled together using special tags. For a detailed look at how Xacros work, see [URDF with Xacro Templates]({% link docs/Technical/ROS2/Jazzy/URDF/URDF-With-Xacro-Templates.md %})

