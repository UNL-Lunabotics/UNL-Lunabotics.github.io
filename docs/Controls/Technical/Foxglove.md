---
title: Foxglove
parent: Technical
nav_order: 5
---

## Table of contents

{:toc}

## Foxglove

This folder contains all of the information needed to get started with Foxglove, including a basic overview of what Foxglove is, an installation tutorial, and a guide to utilizing some of the key UI features Foxglove Offers. This is by no means a comprehensive overview, but it should give you an idea of how to navigate the UI and visualize important data from your bot.<!-- #TODO: refers to the old folder structure -->

## Foxglove Setup  

This page will go over the basics of what Foxglove is, how to set it up with your project.
Note: In this text and in other documentation online, the terms "Foxglove" and "Foxglove Studio" are used largely interchangeably. They effectively mean the same thing.

## What is Foxglove?

Foxglove Studio is an application for observing robotics data. It provides an extensive selection of highly configurable tools for visualizing and understanding what the robot is doing, all in real time. Most tools available in Foxglove support visualizing ROS2 topics in some form. Effectively, it is a much more customizable and reliable alternative to Rviz.

At the time of writing, Foxglove Studio is available as a downloadable application for Windows, MacOS, and Debian-based Linux distributions via the official [download page](https://foxglove.dev/download). It is also available to download on [Canonical Snapcraft](https://snapcraft.io/foxglove-studio). While this version should theoretically be usable on other, non debian-based distributions, we have not had success getting this to work.

If you can't (or don't want to) use the downloadable version, you can also access Foxglove as a [web application](https://app.foxglove.dev).  

Disclaimers:  

- Officially, the Foxglove Studio web app is only supported on the Google Chrome browser. We have been using it on Firefox with little to no issue, but your milage may vary.  
- When running ROS2 through a Docker container, you will need to install and add [Zenoh](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html) to your project in order for Foxglove to be able to receive and visualize all of your topics.

## Setup

Setting up Foxglove Studio on its own is a fairly simple process. That being said, there are a few tweaks you may have to make in order to get the visualization to function consistently. The following is a step-by-step process that will get your ROS2 project set up to display data inside Foxglove.

1. To start, if you haven't already, make an account on the [Foxglove website](https://app.foxglove.dev/signup).  
Note: UNL Lunabotics already has a Foxglove account set up. If you are working on a UNL Lunabotics project, sign in with the UNL Lunabotics Programming Google account.  

2. If you plan on using the downloadable version of Foxglove Studio, download it using one of the download links [above](#what-is-foxglove). If you are using the web app version (also found in the link [above](#what-is-foxglove)), move on to step 3.  

3. Install the `foxglove_bridge` node on the machine you will be using as your groundstation (i.e. whichever machine you would normally run Rviz on).  
   For Debian-based systems, the installation command probably looks something like this:

   ```bash
   sudo apt install ros-jazzy-foxglove-bridge
   ```

   Verify the installation by launching:

   ```bash
    source /opt/ros/jazzy/setup.bash # if necessary
    ros2 launch foxglove_bridge
   ```

   Verify that the node launches without error, then shut it down by pressing `Ctrl + C` while the terminal is in focus.

   The purpose of `foxglove_bridge` is to automatically subscribe to all of the topics in your ROS2 system, and make them available via a localhost connection. This is how Foxglove Studio receives the topics for visualization.

4. Include the `foxglove_bridge` node in your ROS2 launch file:  
   To make foxglove_bridge launch when the rest of your nodes start, you must specify it as a Node in your launch file. That inclusion should look similar to this:

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

5. This step is completely optional. If you want Foxglove Studio to automatically start when you execute your ROS2 launch file, include one of the following processes in your launch file. If you are not interested in this, move on to step 6.  

   If you are using the downloadable application, include this process:

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

Well Done! You should now have foxglove set up and ready to begin using.  
The next section will go over some basic usage of the Foxglove User Interface, since it can be a little bit overwhelming to get used to.

## UI Overview

As stated in the previous section, launch your ROS2 package and open Foxglove (either through the application or the website) if it is not already open. Upon opening, you may be greeted by a login screen. If this is the case, go ahead and log in with the account you made in the last section. You will then be brought to the Dashboard, which should look something like this:

![image]({% link attachments/foxglove/Dashboard.png %})

Click on the box labeled "Open Connection":

![image]({% link attachments/foxglove/Connection.png %})

Make sure "Foxglove Websocket" is selected, and the localhost port is the same as the one your bridge node is using (the default should be fine if you didn't change it.). Once you verify the localhost URL is correct, click "Open".

Depending on your active topics or if you have used Foxglove in the past, your screen may look a little different from this, but the layout should be relatively the same.

![image]({% link attachments/foxglove/Default-Layout-Startup.png %})

## Panels

Foxglove contains dozens of different panels that can be used to visualize different types of data received by your ROS2 topics. For the sake of this tutorial, we will only be looking at a few of these panels. To start, click on the "3D" Panel.

The 3D panel is very similar to the robot visualization you will see if you run Rviz. It serves as a way for you to visualize what your robot can see/understand about the world around it. If you click on the 3d panel, you should see a list of options appear in the menu on the left side of the screen. Here, you can configure things like what topics are visualized and how those topics are visualized. I strongly recommend experimenting with the different options available here to see what they all do, but for the sake of this tutorial, you can scroll down to the "Topics" section in the panel settings to see a list of all the topics Foxglove can see. You can then click on the little eyeball icon to the right of each topic to toggle it's visibility.  

![image]({% link attachments/foxglove/3d-Panel.png %})

The Foxglove UI is highly customizable, allowing you to pick and choose which panels you have open at any given time. For example, if I want to view the depth camera image alongside the 3D model I currently have visible, I can do so by clicking on the "Add panel" button in the top right corner, and select the "Image" panel.  

![image]({% link attachments/foxglove/Add-Image-Panel.png %})

I can then click on the image panel and configure its settings just as I did for the 3D panel. In this case, the most important setting to look at is what topic the Image panel is displaying. This can be found under the "General" tab in the panel settings.

![image]({% link attachments/foxglove/Image-Topic-Selection.png %})

Important Note: Not all of the settings will affect every panel. For example, toggling the visibility of the `/robot_description` topic will not affect anything in the Image panel. Only settings pertaining to what each panel shows will affect that panel.  

All of the panels in foxglove are modular, so you can move them around the window however you like by simply clicking and dragging the panel to the location you want it to go, similar to how you might move panels around in VSCode. This allows you to customize the layout of the panels however you want.

![image]({% link attachments/foxglove/Panel-Customization.png %})

If you want to close one of the panels you have pulled up, you can simply do so by clicking on the vertical line of three dots in the top right corner of a panel and selecting "Remove panel".  

![image]({% link attachments/foxglove/Remove-Panel.png %})

## Import/Export Layouts

Now lets say you have a panel layout you like and want to save it for later use. You can do this by saving the layout configuration, and even downloading it as a JSON file for use on other systems. To do this, click on the dropdown menu in the top right corner of the screen and select "Manage layouts".  

![image]({% link attachments/foxglove/Manage-Layouts.png %})

This should bring you to the layout manager. Here you can configure layouts for yourself and your organization. To configure a particular layout, click on the vertical line of 3 dots to the far right of the layout you want to modify. For this tutorial, we are simply going to download the layout so we can import it in the future. To do so, click the "Download" button in the menu, and save the resulting JSON file to your PC wherever you like.

![image]({% link attachments/foxglove/Download-Layout.png %})

Later, if you want to import the layout you saved, you can do so from the main panels screen, by clicking the "Import from file" button in the same dropdown menu you used to access the layout manager. You can then simply select the JSON file you previously saved, and it will load the corresponding layout.

![image]({% link attachments/foxglove/Import-Layout.png %})

This concludes our basic overview of how to navigate and utilize Foxglove. It is important to mention once again that this is nowhere near everything Foxglove is capable of visualizing, as there are dozens of panels available for both sending and receiving data from your robot. For a much more detailed overview of each available panel in Foxglove and how they work, I recommend looking at the [official documentation](https://docs.foxglove.dev/docs/visualization/panels).  

> Author: Jesse Mills (<https://github.com/JesseMills0>)
