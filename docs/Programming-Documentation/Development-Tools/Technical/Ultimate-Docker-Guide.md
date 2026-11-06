---
title: Ultimate Docker Guide
parent: Technical
grand_parent: Development Tools
nav_order: 2
---

# Ultimate Docker Guide

## Table of Contents
{: .no_toc .text-delta }

- Table of Contents
{:toc}

## Introduction

This guide assumes you already vaguely understand what Docker is and just need to install it. [Docker](https://www.docker.com/) is a containerization software that is used to get an isolated environment to program in. For more information about how Docker works, see our [documentation]({% link docs/Programming-Documentation/Development-Tools/Conceptual/Understanding-Docker.md %}).

While Docker works (mostly) out-of-the-box in Linux on our repositories, that is not the case with macOS. macOS cannot natively run graphical apps on Docker. For this, we will use two programs on macOS to bridge GUI apps from Docker to macOS, along with special configuration in Docker to link it all together.

{: .important}
There are a lot of variations in what you need to do depending on what operating system your computer is and what GPU you have. The headings will try to make this VERY clear, read them!

## Windows Initial Setup

### Windows: Installation

Ensure Git is installed.

Ensure WSL is installed. If it is not, follow the [WSL Guide]({% link docs/Programming-Documentation/Development-Tools/Technical/WSL.md %})

Go to the official [Docker Windows Installation](https://docs.docker.com/desktop/setup/install/windows-install/) page and download the `Docker Desktop Windows x86_64` version.

{: .important}
When prompted by the installer, select `WSL 2` as your backend.

### Windows: Post Installation

Enter your WSL instance with either the app or by typing `wsl` in a terminal or Powershell. All these post installation steps are meant to be executed **inside** WSL.

Download the docker setup script at [docker_setup.sh](https://github.com/UNL-Lunabotics/scripts_and_prebuilts/blob/main/scripts/docker_setup.sh). Run the command `chmod +x docker_setup.sh` to make it executable and then `./docker_setup.sh -w`. Answer the questions when prompted and it will automatically set up USB Passthrough for you.

{: .important}
All instructions given will be for your WSL instance. If something is meant to execute on Windows, it will specifically say so. **Always work in WSL from now on.**

## MacOS Initial Setup

{: .note}
Docker on macOS does **not** currently support USB passthrough. If USB passthrough is required, you must use the [Virtual Machine Setup]({% link docs/Programming-Documentation/Development-Tools/Technical/macOS-Virtual-Machine.md %}). See the [open Docker issue](https://github.com/docker/roadmap/issues/511) to learn more.

### MacOS: Installation

To install Docker Desktop, click [here](https://docs.docker.com/desktop/setup/install/mac-install/) to download. Then run the installer.

There are two applications required for macOS Docker. The first one is [XQuartz](https://www.xquartz.org/index.html). This will let Linux apps run on macOS. You can either install it from the link, or use the following if [Homebrew](https://brew.sh/) is installed:

```bash
brew install xquartz
```

The second program needed is [VirtualGL](https://virtualgl.org/). This will pass any graphics calls from Docker to macOS. You can either install it from the link, or use:

```bash
brew install virtualgl
```

### MacOS: Post Installation

By default, Docker Desktop disables containers to run web-apps on your local network. This is required for certain apps (e.g. Rerun, MuJoCo Scene Editor, etc.). So, host networking needs to be enabled.

Open Docker Desktop. It may prompt questions for initial setup. Note that you can skip account creation. Once on the main dashboard, click the Settings gear on the top-right, then select the "Resources" category, then the "Network" sub-category. Click the "Enable host networking" checkbox.

## Linux Initial Setup

### Linux: Installation

Follow the [Docker Guide](https://docs.docker.com/engine/install/) to install Docker, then type `docker ps -a` to confirm it runs.

{: .important}
If you have an NVIDIA GPU, you also need to install NVIDIA tools to allow Docker containers to use your GPU. The following information is NVIDIA exclusive.

First, [Install the Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installation), selecting your Linux distro and package manager.

Then, [Configure Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker). Ensure you restart the Docker daemon, otherwise changes will not be applied.

### Linux: Post Installation

If you get an error saying `Got permission denied while trying to connect to the Docker daemon socket` then view the [Post-Installation Guide](https://docs.docker.com/engine/install/linux-postinstall) provided by Docker.

## Compose Setup

Our team uses [Docker Compose](https://docs.docker.com/compose/) to configure our containers and run them.

**If you are working in an existing repository**, just clone it.

**If you are making a new repository**, you can start with the templates available to you at our [Docker Templates](https://github.com/UNL-Lunabotics/scripts_and_prebuilts/tree/main/prebuilts/docker). Ensure you have a dedicated `docker/` folder since all of our code assumes that is where it is placed.

### NVIDIA GPU OVERRIDE YOU HAVE TO DO

{: .important}
If you have an NVIDIA GPU, regardless of operating system, you have to do the below steps.

Now that the NVIDIA container toolkit is installed, you will need to tell Docker to use it. Create a `compose.override.yaml` inside the `docker` folder. Add the following content:

{: .important}
The file **MUST** be named `compose.override.yaml`, with the correct service name. The override will not be applied otherwise.

```yaml
services:
  <CONTAINER_NAME>:
    gpus: all
```

Replace `<CONTAINER_NAME>` with the "name" from the main `compose.yaml` file.

{: .note}
> If using profiles, use the following template instead:
>
> ```yaml
> services:
>  <CONTAINER_NAME>_<PROFILE_NAME>:
>    gpus: all
> ```
>
> Replace `<CONTAINER_NAME>` and `<PROFILE_NAME>` with their respective names.

Now, the NVIDIA GPU should be passed through properly, and you are able to run GUI apps within the Docker container.

### MACOS OVERRIDE YOU HAVE TO DO

Before starting the Docker Container, we will need to add a file in the repository, which will let Docker pass information to XQuartz and VirtualGL. Create a `compose.override.yaml` inside the `docker` folder. Add the following content:

{: .important}
The file **MUST** be named `compose.override.yaml`, with the correct service name. The override will not be applied otherwise.

```yaml
services:
  <CONTAINER_NAME>:
    environment:
      - DISPLAY=host.docker.internal:0
      - VGL_DISPLAY=egl
```

Replace `<CONTAINER_NAME>` with the "name" from the main `compose.yaml` file.

{: .note}
> If using profiles, use the following template instead:
>
> ```yaml
> services:
>  <CONTAINER_NAME>_<PROFILE_NAME>:
>    environment:
>      - DISPLAY=host.docker.internal:0
>      - VGL_DISPLAY=egl
>
>    extra_hosts:
>      - "host.docker.internal:host-gateway"
> ```
>
> Replace `<CONTAINER_NAME>` and `<PROFILE_NAME>` with their respective names.

## Running and Managing Containers

### Recommended VSCode Extensions

If you haven't already, go to [Setting up VSCode]({% link docs/Programming-Documentation/Development-Tools/Technical/Setting-Up-VSCode.md %}).

{: .important}
There are a few extensions that make using and managing containers way easier. It is **HIGHLY** recommended you download and use them.

- [Remote Development by Microsoft](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
- [Container Tools by Microsoft](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-containers)

Remote Development is a bundle of extensions that will allow you to interact with Docker and WSL in VSCode. Container Tools lets you more easily manage containers (starting, stopping, etc) and has better functionality with Docker Compose.

### Building Containers

If you are using the terminal, you can run the command `cd docker/ && docker compose --profile <PROFILE> up` from the root of the repository.

OR if you have installed container tools, you can right click on the `compose.yaml` file and select composed up and it should do it for you. Select just compose up if there is a default profile or select services if there is not default

![Compose up example]({% link attachments/compose_up_example.png %})

### Working in the Containers

You have two options for working in the containers: attaching VSCode or attaching a shell. It is highly recommended that you attach a shell rather than VSCode since otherwise you will not be able to push changes to git without adding some sort of configuration file. Attaching a shell means you can still use Git and everything in VSCode but execute run commands in the terminal. To attach a shell, follow the below example image.

![Attach shell example]({% link attachments/attach_shell_example.png %})

If you would like to attach a shell manually in the console, you can also type the command `docker exec -it <CONTAINER>_<PROFILE> bash` which does the same thing as using the VSCode GUI.

#### EXTRA LINUX STEP YOU HAVE TO DO TO RUN CONTAINERS

Any time you restart your computer, you will HAVE to do `xhost +local:` to give the container access to X11 displays or any GUI ran from the container will crash. There are ways to automate this, but I have not implemented any of them personally.

{: .important}
Any time you restart your computer, run the command `xhost +local:` to give local applications the ability to run GUI apps.

#### EXTRA MACOS STEP YOU HAVE TO DO TO RUN CONTAINERS

When you want to use GUI apps, you will need an additional command at the start. Normally, if running a GUI app such as RViZ, you may do `rviz2` or `ros2 run rviz2`. Instead, use the following for any app that has a GUI:

```bash
vglrun rviz2
```

Or,

```bash
vglrun ros2 run rviz2
```

Here, `vglrun` tells graphics calls to pass over to macOS. If you do not use `vglrun`, the program will error out. This is required for MuJoCo, RViZ, or any other application that requires a GUI.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
> 
> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
>
> Last Reviewed: 2026-11-05
