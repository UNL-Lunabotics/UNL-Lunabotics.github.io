---
title: Installation and Setup
parent: Gazebo
nav_order: 1
---

## Installation and Setup  

{: .important}
This guide assumes you are already somewhat familiar with the structure and functionality of URDF files, and that you already have a URDF of a robot you would like to simulate in Gazebo. For more detailed information on the formatting of URDF files, see the [URDF section]({% link docs/Technical/ROS2/Jazzy/URDF/index.md %}). This is also where you will learn how to add SDF references to your URDF, which is critical to a Gazebo simulation.

## What is Gazebo?

Gazebo is a free robot simulation environment run by Open Robotics, the same group that manages ROS2. That being said, these two projects are managed entirely separately from each other and Open Robotics seems strangely determined to keep their identities separated.

There are two distinct versions of Gazebo you will come across if you read about this software online. These versions work completely differently from each other, but both were at one point just called "Gazebo", which can create a lot of confusion when researching. The first version, now known as Gazebo Classic, is the original Gazebo simulator, and is what most of the documentation about Gazebo you find will be referring to. Gazebo Classic is no longer updated, and only supports up to ROS2 Humble (EoL May 2027).  

The second version was originally called Ignition, but is now called "New" Gazebo, or simply "Gazebo" It is the only version of Gazebo you are able to use with the more modern distributions of ROS2, such as Jazzy, which is what this guide will focus on. While there is a decent amount of documentation on migrating projects from Gazebo Classic to New Gazebo, there appears to be little to no documentation about creating new projects in New Gazebo. That is what the purpose of this guide is.

## Installing Gazebo with ROS2

To install the default version of Gazebo for the ROS2 distribution you are running, you can simply run this command in your terminal:

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

For example, for ROS2 Jazzy, this command will install Gazebo Harmonic and its appropriate libraries.

{: .note}
If you have already sourced your ROS2 installation, `${ROS_DISTRO}` should be automatically replaced with the distribution you are using. If this doesnt work, you can manually replace `${ROS_DISTRO}` with the distribution you are using, such as `jazzy`.

To verify the installation was successful, run this command in your terminal:

```bash
apt-cache depends ros-${ROS_DISTRO}-ros-gz
```

Verify your output matches (or at least is similar to) this:

```text
ros-jazzy-ros-gz
  Depends: ros-jazzy-ros-gz-bridge
  Depends: ros-jazzy-ros-gz-image
  Depends: ros-jazzy-ros-gz-sim
  Depends: ros-jazzy-ros-gz-sim-demos
  Depends: ros-jazzy-ros-workspace
```

## Getting Started

