---
title: Using the Docker Setup
parent: Technical
grand_parent: Development Tools
nav_order: 5
---

# Using the Docker Setup
{: .no_toc }

Now that Docker is installed on your system, we can use it in our repositories.

## Table of Contents
{: .no_toc .text-delta }

- Table of Contents
{:toc}

## Cloning a Repository

If you haven't already, clone a repository that uses our docker setup. Type the following in to clone the repository:

```bash
git clone https://github.com/UNL-Lunabotics/<REPOSITORY_NAME>
```

Change `<REPOSITORY_NAME>` with the name of the repository you want to work with.

## Starting Docker

There are a couple of steps to start the Docker container.

### Setup Script

First, run the `docker_setup.sh` script using the following:

```bash
./scripts/docker_setup.sh <OS>
```

Replace `<OS>` with the device you use. `-l` for Linux, `-w` for Windows, and `-m` for macOS.

### Starting the Container

Now that Docker is ready to go, we can start the Docker container. There are two ways of doing this. It is recommended to use VSCode Extensions.

{: .important}
If your system requires a `compose.override.yaml`, add that in the `docker` folder before starting Docker. If unsure, refer back to your respective device's setup guide.

#### VSCode Extensions

This is the easiest way to get started, as it doesn't require any terminal commands.

First, install both the [Docker DX](https://marketplace.visualstudio.com/items?itemName=docker.docker) and [Container Tools](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-containers) extensions for VSCode.

Then, navigate back to the VSCode Explorer (where the repository files are), then expand the `docker` folder. Right-click on the `compose.yaml` file and select the "Compose Up - Select Services" option. In the pop-up window, select "Profiles" and choose the profile(s) you would like to use. Sometimes there is one default profile, while other repositories may have multiple. This will depend on the repository you are working with.

Now the Docker container should be starting up. This may take a while for the first time, as it is setting everything up from scratch. Subsequent runs should be much faster.

Once the Docker container builds, click on the container icon in the VSCode navigation bar. Then, under "Containers", you should see an entry for the container that was built. Right-click and press "Attach Shell".

Now, you should be brought into a new VSCode terminal. You should be able to run commands into this terminal (e.g. `ros2`, etc.)

{: .important}
This terminal is **NOT** the same as the normal VSCode terminal. The container terminal is running in a different environment. You should only run commands that are specific to your repository in this terminal (e.g. `ros2`, etc.)

When you want to stop the Docker container, navigate back to the VSCode Explorer, and expand the `docker` folder. Right-click on the `compose.yaml` file, and select the "Compose Down - Select Services". In the pop-up window, select "Profiles" and choose the profile(s) you would like to stop.

This will terminate the shell and stop the Docker container.

#### Terminal

If you prefer using traditional docker commands, you can do that as well.

First, open a new terminal in VSCode by clicking on the "Terminal" option in the top navigation bar, then "New Terminal".

Then, type the following command to start the docker container:

```bash
cd docker && docker compose up -d
```

{: .note}
> If using profiles, use the following instead:
>
> ```bash
> cd docker && docker compose --profile <PROFILE_NAME> up -d
> ```
>
> Replace `<PROFILE_NAME>` with the name of the profile you would like to start.

The first time running this command may take a while, as it is downloading and installing everything needed in the container. Subsequent runs will be faster.

When it successfully starts up, you can enter the docker container shell by:

```bash
docker exec -it <docker-container-name> bash
```

{: .note}
> If using profiles, use the following instead:
>
> ```bash
> docker exec -it <DOCKER_CONTAINER_NAME>_<PROFILE_NAME> bash
> ```
>
> Replace `<DOCKER_CONTAINER_NAME>` and `<PROFILE_NAME>` to their respective names.

To stop the container, type the following:

{: .important}
Ensure you are **NOT** in the container terminal. This should be the default VSCode terminal. If unsure, open a new terminal.

```bash
cd docker && docker compose down
```

{: .note}
> If using profiles, use the following instead:
>
> ```bash
> cd docker && docker compose --profile <PROFILE_NAME> down
> ```
>
> Replace `<PROFILE_NAME>` with the profile you would like to stop.

## Conclusion

You are now set up with our development tools! Happy coding!

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
