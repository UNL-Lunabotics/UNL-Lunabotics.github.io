---
title: Linux Setup
parent: Technical
grand_parent: Development Tools
nav_order: 2
---

<!-- TODO: Clarify compose.override.yaml is in a repo -- Tell user to clone a repo -->

# Linux Setup
{: .no_toc }

Here you will find how to get set up with development tools for Linux.

## Table of Contents
{: .no_toc .text-delta }

- Table of Contents
{:toc}

## Installing Docker

Follow the [Docker Guide](https://docs.docker.com/engine/install/) to install Docker, then type `docker ps -a` to confirm it runs.

If you get an error saying `Got permission denied while trying to connect to the Docker daemon socket` then view the [Post-Installation Guide](https://docs.docker.com/engine/install/linux-postinstall) provided by Docker.

If not using an NVIDIA GPU, then you are done with the installation. Move on to [Using the Docker Setup]({% link docs/Development Tools/Technical/Using-Docker-Setup.md %}).

## NVIDIA GPUs

By default, NVIDIA GPUs do not get passed through into the Docker container. So, we will need to manually tell it to bridge the GPU.

### Installing the Container Toolkit

The NVIDIA Container Toolkit allows Docker (and other containerization software) to use the GPU.

First, [Install the Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installation), selecting your Linux distro and package manager.

Then, [Configure Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker). Ensure you restart the Docker daemon, otherwise changes will not be applied.

### Cloning Repository

Once all the required programs are installed, you'll want to clone a repository that uses our Docker setup. Type the following in to clone the repository:

```bash
git clone https://github.com/UNL-Lunabotics/<REPOSITORY_NAME>
```

Change `<REPOSITORY_NAME>` with the name of the repository you want to work with.

### Docker Compose Override

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

Move on to [Using the Docker Setup]({% link docs/Development Tools/Technical/Using-Docker-Setup.md %}).

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)