---
title: Docker Setup
parent: macOS Setup
nav_order: 1
---

## macOS Docker Setup

The easiest way to set up development tools for macOS is using our existing Docker setup.

{: .note}
Docker on macOS does **not** currently support USB passthrough. If USB passthrough is required, you must use the [Virtual Machine Setup]({% link docs/Technical/Setup Dev Tools/macOS/Virtual Machine/index.md %}). See the [open docker issue](https://github.com/docker/roadmap/issues/511) to learn more.

### Overview

[Docker](https://www.docker.com/) is a containerization software that is used to get an isolated environment to program in. For more information about how Docker works, see our [documentation]({% link docs/Curriculum/Understanding-Docker.md %}).

While Docker works out-of-the-box in Linux on our repositories, that is not the case with macOS. macOS cannot natively run graphical apps on Docker. For this, we will use two programs on macOS to bridge GUI apps from Docker to macOS, along with special configuration in Docker to link it all together.

### Downloading Docker Desktop

To install Docker Desktop, click [here](https://docs.docker.com/desktop/setup/install/mac-install/) to download. Then run the installer.

### Installing Required Apps

There are two applications required for macOS Docker. The first one is [XQuartz](https://www.xquartz.org/index.html). This will let Linux apps run on macOS. You can either install it from the link, or use `brew install xquartz` if [Homebrew](https://brew.sh/) is installed.

The second program needed is [VirtualGL](https://virtualgl.org/). This will pass any graphics calls from Docker to macOS. You can either install it from the link, or use `brew install virtualgl`.

### Cloning Repository

Once all the required programs are installed, you'll want to clone a repository that uses our docker setup. Type the following in to clone the repository:

```bash
git clone https://github.com/UNL-Lunabotics/<repo-name-here>
```

Change `<repo-name-here>` with the name of the repository you want to work with.

### Adding Overrides

Before starting the Docker Container, we will need to add a file in the repository, which will let Docker pass information to XQuartz and VirtualGL. To do this, create a file with the following name and content `compose.override.yaml`:

```yaml
# compose.override.yaml
services:
  # Change this name
  <docker-container-name>:
    environment:
      - DISPLAY=host.docker.internal:0
      - VGL_DISPLAY=egl
```

Change `<docker-container-name>` with the "name" from the main `compose.yaml` file.

{: .important}
The file **MUST** be named `compose.override.yaml`, with the correct service name. It will not work without.

### Runing the Setup Script

The last thing to do before starting the container itself is to run a script. The repository should have a `scripts/docker_setup.sh`. If at the root of the repository, run the following command to trigger the script:

```bash
./scripts/docker_setup.sh -m
```

This script will initailize and start XQuartz and VirtualGL.

### Starting the Docker Container

Now that everything is set up, we can finally start the docker container. If at the root of the repository, use the following command:

```bash
cd docker && docker compose up -d
```

The first time running this command may take a while, as it is downloading and installing everything needed in the container. Subsequent runs will be faster.

When it successfully starts up, you can enter the docker container shell by:

```bash
docker exec -it <docker-container-name> bash
```

{: .note}
> If on a repository that uses compose profiles, you will want to use the following instead:
>
> ```bash
> cd docker && docker compose --profile <profile-name> up -d
> ```
>
> Make sure to change the `<profile-name>` to the correct profile.
> Then similar for entering the docker container shell:
>
> ```bash
> docker exec -it <docker-container-name>_<profile-name> bash
> ```
>
> Again, changing the names to the correct one.

### Running GUI Apps

Now that the container is running and in a shell, you can start GUI apps. Normally, if running a launch file, you may do `ros2 launch sim sim.launch.py`. Instead, use the following for any app that has a GUI:

```bash
vglrun ros2 launch sim sim.launch.py
```

Here, `vglrun` tells graphics calls to pass over to macOS. If you do not use `vglrun`, the program will error out. This is required for Gazebo, MuJoCo, RViZ, or any other application that requires a GUI.

### Stopping the Container

To stop the docker container, open a new terminal and type the following, if at the root of the repository:

```bash
cd docker && docker compose down
```

You can now safely quit XQuartz from running as well.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
