---
title: Linux Setup
parent: Archive
nav_order: 1
---

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

## Linux Setup

Here you will find how to get set up with development tools for Linux.

First, [Install VSCode]({% link docs/Non-Programmer Friendly Zone/How-to-Use-VSCode.md %}).

You will find two setups. The DevContainer Setup is recommended, but you can install everything on your Linux operating system if you want to.

## DevContainer Setup

This is the recommended way to develop for Linux.

### Installing Docker

Docker will let us create a virtual environment to use when developing for ROS2. This is recommended so ROS2 has a stable, isolated environment that won't break your Linux Installation.

Follow the [Docker Guide](https://docs.docker.com/engine/install/) to install Docker, then type `docker ps -a` to confirm it runs.

If you get an error saying `Got permission denied while trying to connect to the Docker daemon socket` then view the [Post-Installation Guide](https://docs.docker.com/engine/install/linux-postinstall) provided by Docker.

### Setting up the DevContainer

DevContainers will use Docker to create the virtual environment mentioned before.

#### Cloning the Repository

If comfortable with `git` and the terminal (recommended), sign into GitHub and type `git clone https://github.com/UNL-Lunabotics/terrence_2.0`

{: .note}
You may need to use `gh` and type `gh auth login` to gain access to cloning into the `UNL-Lunabotics` organization.

Otherwise, open VSCode and click on "Clone Git Repository". Sign in with GitHub, and clone the `UNL-Lunabotics/terrence_2.0` repo.

#### Starting the DevContainer

Once the repository is cloned, open VSCode and point it to the repo.

Then do `Ctr+Shift+P` and type `Rebuild and reopen in Container`. This will launch the DevContainer.

{: .note }
Rebuilding the DevContainer is only needed if any changes are made to the files in the `./devContainer/` folder. After this is run, you may use `Reopen in Container` instead.

The DevContainer should open successfully, with all your files in the "Explorer" tab on the left.

Once the DevContainer launches, you are done setting up the dev tools! Have fun developing!

{: .important }
If running on an ARM-based system, you **must** use new Gazebo. Gazebo Classic does not have an executable for ARM as far as I know.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)

## Bare Metal Setup

If you do not want to use a DevContainer, you can try running everything on your installation of Linux itself. Note that this is not the recommended way to develop and mileage may vary.

### Installing ROS2

Follow the [ROS2 Install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) guide, and make sure to select the correct version of ROS2. At the time of writing, we are using `ROS2 Jazzy`, but this may change. You will also want to install the desktop variant of ROS2.<!-- #TODO: time-sensitive statement; verify it is still current -->

After ROS2 is installed, you'll want to add an entry into the `~/.bashrc`. This will ensure the `ros2` command is recognized any time you open a terminal. Type `sudo nano ~/.bashrc`. Using your arrow keys, navigate until you reach the end of the script. Then, add the following entry: `source /opt/ros/jazzy/setup.bash`.

{: .note}
If you are on a different version of ROS2, you'll need to change the `jazzy` to whatever version in the command.

Finally, install extra ROS2 packages:

```bash
sudo apt-get update
sudo apt-get upgrade -y
export ROS_CODENAME=jazzy
sudo apt-get install -y python3 python3-pip ros-dev-tools \
    ros-${ROS_CODENAME}-xacro ros-${ROS_CODENAME}-joint-state-publisher-gui \
    ros-${ROS_CODENAME}-twist-mux ros-${ROS_CODENAME}-twist-stamper \
    ros-${ROS_CODENAME}-ros2-control ros-${ROS_CODENAME}-ros2-controllers \
    ros-${ROS_CODENAME}-ros-gz ros-${ROS_CODENAME}-gz-ros2-control \
    joystick jstest-gtk evtest ros-${ROS_CODENAME}-slam-toolbox libserial-dev \
```

### Cloning the Repository

If comfortable with `git` and the terminal (recommended), sign into GitHub and type `git clone https://github.com/UNL-Lunabotics/terrence_2.0`

{: .note}
You may need to use `gh` and type `gh auth login` to gain access to cloning into the `UNL-Lunabotics` organization.

Otherwise, open VSCode and click on "Clone Git Repository". Sign in with GitHub, and clone the `UNL-Lunabotics/terrence_2.0` repo.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
