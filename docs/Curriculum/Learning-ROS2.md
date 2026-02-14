---
title: Learning ROS2
parent: Curriculum
nav_order: 1
---

## Learning ROS2

Robot Operating System (ROS) is a set of open source algorithms, hardware driver software and tools developed to develop robot control software. Even though it has operating system in its name it is not an operating system. It is

- Communication System (Publish Subscribe and Remote Method Invocation),
- Framework & Tools (Build system & dependency management, Visualization, Record and Replay)
- Ecosystem (Language bindings, Drivers, libraries and simulation (Gazebo)).

(Quoted from <https://medium.com/software-architecture-foundations/robot-operating-system-2-ros-2-architecture-731ef1867776>)

Basically, ROS2 is just how we communicate and coordinate information and commands throughout a large and complicated robotics system with a lot of moving parts

### The Basics

The basics concepts of ROS2 have shockingly good documentation that would be difficult to outdo, so for this part it will mostly be links to go do the tutorials yourself. Please [download ROS2]({% link 404.html %}) if you haven't already

Below are the links for the sections you should go through IN ORDER. The sub bullet points are the specific tutorials you should do, and the crossed out ones are not required though you can still do them if you want

#### Beginning Concepts

The links below will teach you the basics concepts of ROS2. You can follow along with the turtlebot examples if that would help you, but feel free to just read through the sections

- [Beginner: CLI Tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
  - [Configuring environment](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
  - [Using turtlesim, ros2, and rqt](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
  - [Understanding nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
  - [Understanding topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
  - [Understanding services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
  - [Understanding parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
  - [Understanding actions](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
  - [Launching nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)

#### Coding the Basic Concepts

These tutorials are when you actually start to get into programming these things. It is HIGHLY recommended that you set up a temporary repository and follow along with creating all this code. I personally recommend typing out the code yourself and adding more comments as you go rather than copy/pasting

What language you choose to write things in is up to you. You do not have to do both the Python and C++ versions of tutorials, though it may be useful to do both

- [Beginner: Client Libraries](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html)
  - [Using colcon to build packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
  - [Creating a workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
  - [Creating a package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
  - [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
  - [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
  - [Writing a simple service and client (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
  - [Writing a simple service and client (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
  - [Creating custom msg and srv files](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
  - [Implementing custom interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)
  - [Using parameters in a class (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
  - [Using parameters in a class (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
  - [Creating and using plugins (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Pluginlib.html)

### Intermediate

This is where things start to get technical. Some of these sections are complicated enough to split into a more dedicated tutorial, so pay attention to which ones are crossed out as optional

- [Intermediate](https://docs.ros.org/en/jazzy/Tutorials/Intermediate.html)
  - [Managing Dependencies with rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html)
  - [Creating an action](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Creating-an-Action.html)
  - [Writing an action server and client (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
  - [Writing an action server and client (Python)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
  - [Composing multiple nodes in a single process](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html)
  - [Using the Node Interfaces Template Class (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Using-Node-Interfaces-Template-Class.html)
  - [Monitoring for parameter changes (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html)
  - [Monitoring for parameter changes (Python)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-Python.html)
  - [Launch](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)
  - [RViz](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-Main.html)

### Advanced

For the most part, advanced topics shouldn't be necessary for you to learn right now. It is recommended that you briefly look through what's in the list just to know the option to research it is there if needed, but you do not have to go through any of them in depth unless you feel like it

<https://docs.ros.org/en/jazzy/Tutorials/Advanced.html>

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
