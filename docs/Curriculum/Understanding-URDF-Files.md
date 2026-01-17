---
title: Understanding URDF Files
parent: Curriculum
nav_order: 3
---

# Understanding URDF Files

Unified Robot Description Format (URDF) is the standard for representing a robot in ROS2. It serves as a machine-readable blueprint that allows ROS2 programs to "visualize" your robot in order to carry out its various tasks, such as control, sensing, and autonomy. 

## Basic URDF Rundown / Construction

Inside a URDF, each "part" of a robot (i.e. wheels, chassis, a robotic arm, cameras, sensors, etc.) is defined as a separate component, with its own configurations, and its own specified placement in the final model.Each of these components come together when ROS2 builds your URDF to define a complete model of your robot, either for the purposes of simulation in software such as Gazebo, Unity, or Godot, or for use in managing an actual robot.

There is very good documentation available online for understanding the inner-workings of a URDF file as well as how to get started building your own first URDF. For starters, I recommend starting with the official ROS2 documentation for getting a good understanding of the basics of how to build a URDF.

**NOTE:** This tutorial will walk you through installing some software for reading & visualizing URDF files, as well as constructing a URDF for a basic robot. If you want to save the file(s) you make in this tutorial and/or share them with others in the future, making a Git repository and saving your progress to [GitHub](https://github.com/) or another platform is *strongly* recommended.

[Building a visual robot from scratch](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)

## Movement

Currently (assuming you have followed the previous tutorial) you have a visual model of a robot built using a URDF file. That is a great start, but as of right now the robot has no moving parts. In order for ROS2 to properly simulate your robot in 3D environments or even for live testing, you need to tell it what parts of your robot are movable and how they move. To do this, I am again going to point you to the official ROS2 documentation, as they have a great tutorial explaining how to update your current robot with movement capabilities.

[Building a movable robot](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Building-a-Movable-Robot-Model-with-URDF.html)

## Collison

I'm going to keep this intro short to save on repettition. Now that you have a movable robot, you will need to add collision to your model so simulated environments will know how to react when your robot runs into something. 

[Adding physical and collision properties](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)

You should now having a fully functional basic URDF for a robot. The next step is optional but strongly recommended for the purposes of readability.

## Xacro

Xacro, a conjunction of the phrase "XML Macro" is a format for breaking up XML files into more manageable sections, rather than having one giant file that stores everything. These macro files are then compiled into a full XML file when the program runs. The tutorial linked below, from the official ROS2 Documentation, goes over how to break up your URDF file (which uses the XML format) into multiple xacro files, as well as how to add support for xacro files to your ROS2 Launch file. 

[Using Xacro to clean up your code](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)

> Author: Jesse Mills (<https://github.com/JesseMills0>)
