---
title: macOS Docker Setup
parent: Technical
grand_parent: Development Tools
nav_order: 3
---

# macOS Docker Setup
{: .no_toc }

The easiest way to set up development tools for macOS is using our existing Docker setup.

{: .note}
Docker on macOS does **not** currently support USB passthrough. If USB passthrough is required, you must use the [Virtual Machine Setup]({% link docs/Development Tools/Technical/macOS-Virtual-Machine.md %}). See the [open docker issue](https://github.com/docker/roadmap/issues/511) to learn more.

## Table of Contents
{: .no_toc .text-delta}

- Table of Contents
{:toc}

## Overview

[Docker](https://www.docker.com/) is a containerization software that is used to get an isolated environment to program in. For more information about how Docker works, see our [documentation]({% link docs/Development Tools/Conceptual/Understanding-Docker.md %}).

While Docker works (mostly) out-of-the-box in Linux on our repositories, that is not the case with macOS. macOS cannot natively run graphical apps on Docker. For this, we will use two programs on macOS to bridge GUI apps from Docker to macOS, along with special configuration in Docker to link it all together.

## Downloading Docker Desktop

To install Docker Desktop, click [here](https://docs.docker.com/desktop/setup/install/mac-install/) to download. Then run the installer.

### Allowing Host Networking

By default, Docker Desktop disables containers to run web-apps on your local network. This is required for certain apps (e.g. Rerun, MuJoCo Scene Editor, etc.). So, host networking needs to be enabled.

Open Docker Desktop. It may prompt questions for initial setup. Note that you can skip account creation. Once on the main dashboard, click the Settings gear on the top-right, then select the "Resources" category, then the "Network" sub-category. Click the "Enable host networking" checkbox.

## Installing Required Apps

There are two applications required for macOS Docker. The first one is [XQuartz](https://www.xquartz.org/index.html). This will let Linux apps run on macOS. You can either install it from the link, or use the following if [Homebrew](https://brew.sh/) is installed:

```bash
brew install xquartz
```

The second program needed is [VirtualGL](https://virtualgl.org/). This will pass any graphics calls from Docker to macOS. You can either install it from the link, or use:

```bash
brew install virtualgl
```

## Cloning Repository

Once all the required programs are installed, you'll want to clone a repository that uses our docker setup. Type the following in to clone the repository:

```bash
git clone https://github.com/UNL-Lunabotics/<REPOSITORY_NAME>
```

Change `<REPOSITORY_NAME>` with the name of the repository you want to work with.

## Adding Overrides

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
> ```
>
> Replace `<CONTAINER_NAME>` and `<PROFILE_NAME>` with their respective names.

You can now use our Docker setup. Before moving on, however, it is important to note how to run GUI apps.

## Running GUI Apps

When you want to use GUI apps, you will need an additional command at the start. Normally, if running a GUI app such as RViZ, you may do `rviz2` or `ros2 run rviz2`. Instead, use the following for any app that has a GUI:

```bash
vglrun rviz2
```

Or,

```bash
vglrun ros2 run rviz2
```

Here, `vglrun` tells graphics calls to pass over to macOS. If you do not use `vglrun`, the program will error out. This is required for MuJoCo, RViZ, or any other application that requires a GUI.

## Moving On

Now that the Docker setup is finished, move on to [Using the Docker Setup]({% link docs/Development Tools/Technical/Using-Docker-Setup.md %})

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
