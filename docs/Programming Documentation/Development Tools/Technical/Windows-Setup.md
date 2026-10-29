---
title: Windows Setup
parent: Technical
grand_parent: Development Tools
nav_order: 1
---

# Windows Setup
{: .no_toc }

Here you will find how to get set up with development tools for Windows.

First, [Install VSCode]({% link docs/Administrative/How-to-Use-VSCode.md %}), then go through the Table of Contents.

## Table of Contents
{: .no_toc .text-delta }

- Table of Contents
{:toc}

## Installing Docker

Docker will let us create a virtual environment to use when developing for ROS2. This is because ROS2 **requires** base Ubuntu to run.

Follow the [Docker Guide](https://docs.docker.com/desktop/setup/install/windows-install/) to install Docker, then open Docker Desktop.

If you get a prompt saying: "Your WSL version needs updating", then:

1. Open the terminal
2. Type in `wsl --update`
3. Restart Docker

### Allowing Host Networking

By default, Docker Desktop disables containers to run web-apps on your local network. This is required for certain apps (e.g. Rerun, MuJoCo Scene Editor, etc.). So, host networking needs to be enabled.

Open Docker Desktop. It may prompt questions for initial setup. Note that you can skip account creation. Once on the main dashboard, click the Settings gear on the top-right, then select the "Resources" category, then the "Network" sub-category. Click the "Enable host networking" checkbox.

## Installing Git For Windows

Git is needed to clone the repository needed. You can download it at [Git Windows Install](https://git-scm.com/install/windows), and run the installer. Note that there are a lot of prompts, but just keep selecting "Next" and use the recommended settings. You do not need to open Git Bash once installed.

## Cloning the Repository

Once all the required programs are installed, you'll want to clone a repository that uses our Docker setup. Type the following in to clone the repository:

```bash
git clone https://github.com/UNL-Lunabotics/<REPOSITORY_NAME>
```

Change `<REPOSITORY_NAME>` with the name of the repository you want to work with.

Then, open the repository in VSCode.

## NVIDIA GPUs

{: .note}
Devices with AMD GPUs can skip this step.

By default, NVIDIA GPUs do not get passed through into the Docker container. So, we will need to manually tell it to bridge the GPU.

To do this, inside the repository, create a `compose.override.yaml` inside the `docker` folder. Add the following content:

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

## Enabling Device Passthrough for Windows

Enabling Device Passthrough will allow you to pass a USB device, such as a controller, to the Docker container. There are two ways to do this. It is **HIGHLY** recommended to use the script, as it will automate a lot of steps. However, for completeness, the manual guide is here too.

{: .important}
This process, whether you do it manually or through the script, requires a custom WSL kernel. This means that we are no longer using the official Microsoft WSL, which may break compatibility for certain apps. Everything done here is reversible.

### Script Setup

The script setup will do everything automatically, requiring minimal manual installation.

Open the repository in VSCode, then open a new terminal. Type the following in:

```bash
./scripts/docker_setup.md -w
```

This will download and install the custom kernel, and then load it in.

This will set everything up for you. Now, move on to [Using the Docker Setup]({% link docs/Programming Documentation/Development Tools/Technical/Using-Docker-Setup.md %}).

### Manual Setup

Note that this requires modifying the WSL kernel, which is fairly technical.

#### Install Ubuntu

Install Ubuntu on WSL using this command:
`wsl --install Ubuntu`

{: .note}
This command will require you to enter a username and password. You can enter anything into this, but I recommend a password you'll remember.

Then, set it as the default:
`wsl --set-default Ubuntu`

Type `wsl` to verify Ubuntu installed correctly, then type `exit` to exit from WSL.

#### Set up the Custom Kernel

Download [this](https://github.com/TheKing349/WSL2-Linux-Kernel/releases/latest/download/vmlinux+modules.zip) file, which is a modified kernel of WSL. It is open-sourced as a [fork](https://github.com/TheKing349/WSL2-Linux-Kernel) of the WSL2-kernel source code to include controller support.

Next, unzip the downloaded `vmlinux+modules.zip` file.
Then, create a new folder at `C:\Users\[YOUR_USER]\wsl`, where `[YOUR_USER]` is replaced with your username.

Then, put the `vmlinux` and `modules.vhdx` files to this folder: `C:\Users\[YOUR_USER]\wsl`

Then, make a new configuration file located here: `C:\Users\[YOUR_USER]\.wsl.config`. Put the following inside the file:

```conf
[wsl2]
kernel=C:\\Users\\[YOUR_USER]\\wsl\\vmlinux
kernelModules=C:\\Users\\[YOUR_USER]\\wsl\\modules.vhdx
```

To load the kernel, you'll need to restart WSL: `wsl --shutdown` and wait 30 seconds. Then type `wsl` to load WSL again.

Then, verify the kernel is loaded: type `uname -r`.  If the result has `theking349-joystick`, it is loaded successfully. You can now `exit` from WSL.

## Attach Controller to WSL

Now that initial setup is done --- either automated with the script or manually --- download [usbipd-win](https://github.com/dorssel/usbipd-win/releases/latest). This will act as the bridge to allow device activity onto WSL.

Plug your device into your computer, and in a terminal and type `usbipd list`.

This command will show the names of different USB devices connected, as well as the bus-id. Find the bus-id of the device you want to pass through.

Once you have the bus-id, type `usbipd bind -b <BUS_ID> --force` in an **admin** terminal. Replace `<BUS_ID>` with the bus-id found in the previous command.

This will force your device to 'bind' to WSL.

{: .note}
This will 'unregister' your device in Windows. To have Windows see your device again, type `usbipd unbind -b [BUS_ID_HERE]`.

Then restart your computer, open VSCode and the Docker container with your controller still plugged in.

In a terminal (does **not** have to be admin), type `usbipd list` again to make sure the bus-id is the same, and then execute `usbipd attach -w Ubuntu -b [BUS_ID_HERE]`, again replacing this command with your bus-id.

This will 'connect' your controller to WSL, which should allow input to work inside the Docker container.

## Using the Docker Setup

From here, it may be helpful to know how to use the Docker setup before continuing. View the [Using the Docker Setup]({% link docs/Programming Documentation/Development Tools/Technical/Using-Docker-Setup.md %}) documentation, then return here to test controller input.

## Testing Controller

Finally, we can test controller input.

First, restart the docker container, then attach a shell.

Then, type the following command. This should print the name of the USB device:

```bash
lsusb
```

Next, verify that the device passed in its input by typing the following command:

```bash
ls /dev/input
```

Here you should see something like the following:

```bash
event0  js0
```

Finally, test out the controller. First install `jstest-gtk`:

```bash
sudo apt update && sudo apt install jstest-gtk
```

Then, run it:

```bash
jstest-gtk
```

In the `jstest` window, you should see an entry for your controller. Double-click it, then test out the controller by moving the joystick or pressing buttons. You should see inputs reflected in `jstest`.

## Troubleshooting

Sometimes a step in the device passthrough may error out (REWORD!). Here are some common troubleshooting tips:

### Permission Errors

If you get a permission error at any point, restart the Docker container again. If that doesn't work, then type the following command:

```bash
sudo chmod -R 666 /dev/input && sudo chmod +x /dev/input
```

### No Such File or Directory

If you get the following error:

```bash
ls: cannot access '/dev/input': No such file or directory
```

Verify `lsusb` shows a device, and make sure the controller is successfully passed through with usbipd.

### No `jstest` Entry

If `jstest-gtk` does not show an entry for your controller, ensure the controller is connected and passed through usbipd, and that `ls /dev/input` shows an input device.

## References

This was only possible due to an issue on [Microsoft's WSL GitHub Page](https://github.com/microsoft/WSL/issues/7747), and the original [fork of WSL2-kernel source code](https://github.com/atticusrussell/WSL2-Linux-Kernel).

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
