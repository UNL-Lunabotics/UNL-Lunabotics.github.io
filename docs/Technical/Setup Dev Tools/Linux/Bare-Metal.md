---
title: Bare Metal Setup
parent: Linux Setup
nav_order: 2
---

## Bare Metal Setup

If you do not want to use a DevContainer, you can try running everything on your installation of Linux itself. Note that this is not the recommended way to develop and mileage may vary.

### Installing ROS2

Follow the [ROS2 Install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) guide, and make sure to select the correct version of ROS2. At the time of writing, we are using `ROS2 Jazzy`, but this may change. You will also want to install the desktop variant of ROS2.

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
