---
title: DevContainer Setup (Recommended)
parent: Linux Setup
nav_order: 1
---

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
