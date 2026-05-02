---
title: Foxglove Setup and Usage
parent: Launch
nav_order: 3
---

This page will go over the basics of what Foxglove is, how to set it up with your project, as well as the basics of how to configure some of the built in tools.  
Note: In this text and in other documentation online, the terms "Foxglove" and "Foxglove Studio" are used largely interchangeably. They effectively mean the same thing.

## What is Foxglove?

Foxglove Studio is an application for observing robotics data. It provides an extensive selection of highly configurable tools for visualizing and understanding what the robot is doing, all in real time. Most tools available in Foxglove support visualizing ROS2 topics in some form. Effectively, it is a much more customizable and more reliable alternative to Rviz.

At the time of writing, Foxglove Studio is available as a downloadable application for Windows, MacOS, and Debian-based Linux distributions via the official [download page](foxglove.dev/download). It is also available to download on [Canonical Snapcraft](https://snapcraft.io/foxglove-studio) if you want to use it on other linux distributions.

If you can't (or don't want to) use the downloadable version, you can also access Foxglove as a [web application](app.foxglove.dev).  
Disclaimer: Officially, the Foxglove Studio web app is only supported on the Google Chrome browser. We have been using it on Firefox with little to no issue, but your milage may vary.

## Setup

Setting up Foxglove Studio on its own is a fairly simple process. That being said, there are a few tweaks you will and/or might have to make in order to get the visualization to function consistently. The following is a step-by-step process that should hopefully get your ROS2 project set up to work inside Foxglove.

1. To start, if you haven't already, make an account on the [Foxglove website](https://app.foxglove.dev/signup).  
Note: UNL Lunabotics already has a Foxglove account set up. If you are working on a UNL Lunabotics project, sign in with the UNL Lunabotics Programming Google account.

2. If you plan on using the downloadable version of Foxglove Studio, download it using one of the download links [above](#what-is-foxglove). If you are using the web app version (also found in the link [above](#what-is-foxglove)), move on to step 3.

3. Install the `foxglove_bridge` node on the machine you will be using as your Groundstation (i.e. whichever machine you have/would have run Rviz on).  
   Installation Command (apt):

   ```bash
   sudo apt install ros-jazzy-foxglove-bridge
   ```

   Verify the installation by launching:

   ```bash
    source /opt/ros/jazzy/setup.bash # if necessary
    ros2 launch foxglove_bridge
   ```

   Verify that the node successfully launches, then shut it down with `ctrl + c`

   The purpose of `foxglove_bridge` is to automatically subscribe to all of the topics in your ROS2 system, and make them available via a localhost connection. This is how Foxglove Studio receives the topics for visualization.

4. Include the `foxglove_bridge` node in your ROS2 launch file:  
   To make foxglove_bridge launch when the rest of your nodes start, include this block of code in your launch file:

   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node

   # define robot nodes...

   foxglove_bridge = Node(
      package='foxglove_bridge',
      executable='foxglove_bridge',
      name='foxglove_bridge',
   ),

   return LaunchDescription([

      # list nodes...

      foxglove_bridge,
   ])
   ```

5. This step is completely optional. If you want Foxglove Studio to automatically start when you execute your ROS2 launch file, include one of the following code blocks in your launch file. If you are not interested in this, move on to step 6.  

   If you are using the downloadable application, include this code:

   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   # imports...

   def generate_launch_description():

      # define robot nodes...
      # foxglove_bridge

      foxglove_studio = ExecuteProcess(
         cmd=['foxglove-studio'],
         output='screen',
      ),

      return LaunchDescription([

         # list nodes...

         foxglove_studio,
      ])
   ```

   If you are using the web app, include this code:  
   Note: If you are (somehow) using Windows or MacOS, `'xdg-open'` will need to be replaced with the browser command corresponding to your operating system.  
   For Windows, use `'start'`. For MacOS, use `'open'`  
   TODO: Test how this works in Docker

   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   from launch.actions import ExecuteProcess
   # imports...

   def generate_launch_description():

      # define robot nodes...
      # foxglove_bridge

      foxglove_studio_web = ExecuteProcess(
         cmd=['xdg-open', 'https://foxglove.dev'],
         output='screen',
      ),

      return LaunchDescription([

         # list nodes...

         foxglove_studio_web,
      ])
   ```

6. Run your launch file to verify everything works.  
   If you did not add an autostart node, you will need to manually start foxglove once your ROS2 launch file is executing.

Contrats! You should now have foxglove set up and ready to begin using.  
The next section will go over some basic usage of the Foxglove User Interface, since it can be a little bit overwhelming to get used to.

> Author: Jesse Mills (<https://github.com/JesseMills0>)