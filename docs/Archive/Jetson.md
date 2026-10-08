---
title: Jetson
parent: Archive
nav_order: 4
---

## Table of contents

{:toc}

## Jetson-Specific

If for some reason you like suffering, you can browse these explanations on how to make Jetson Orin Nano's work.

## Installing Ubuntu 24.04 (Noble Numbat)

{: .warning}
If your Jetson does not support Ubuntu 24.04 (Like how the Jetson Orin Nano doesn't at the time of writing), then following this tutorial will **somehow** result in you installing Ubuntu 22.04 (Jammy Jellyfish) instead, which makes absolutely zero sense but hey thats Nvidia for you.
{: .mt-4 }

### Prerequisites

Materials:

1. USB-A to USB-C cable
2. Jetson Orin Nano Developer Kit with power supply
3. A singular female-to-female jump wire
4. The NVMe already installed on the Jetson
5. Any computer with a Debian-based Linux distribution (theoretically any computer will work but you might have to adapt these commands to whatever your OS accepts)

`sudo apt install -y tar gzip bzip2 xz-utils sudo wget qemu-user-static`

### Prepping the Jetson

Make sure the Jetson is not powered.

Allegedly you shouldn't have to do this every time, but in reality you have to do this every time. **The Jetson will not be discoverable unless it is in forced recovery mode.**

To put the Jetson in recovery mode, get a female-to-female jump wire and connect the FC REC pin to any GND pin (shown below)

![Jetson Pins]({% link attachments/Jetson_FC_REC_GND_Pins.png %})

![Wire in Jetson]({% link attachments/Wires_In_Jetson.jpg %})

After this connect the Jetson to a power source and connect it via USB-A to USB-C to your computer.

### Downloading Necessary Files

NVIDIA hates you and doesn't officially support Ubuntu24.04, so we have to use the Ubuntu22.04 drivers with the Ubuntu24.04 file system and frankenstein them together.

Go to <https://developer.nvidia.com/embedded/jetson-linux-r3644> and download the Driver Package (BSP) and the cuda_driver_36.4.4.tbz2. After you are done you should have the file `Jetson_Linux_R36.4.4_aarch64.tbz2` and `cuda_driver_26.4.4.tbz2`

Go to <https://developer.nvidia.com/embedded/jetpack/downloads> and download the Sample Root Filesystem. You should have file `Tegra_Linux_Sample-Root-Filesystem_R38.2.1_aarch64.tbz2`

**Make sure all these files are saved to your Downloads/ directory.**

### Flashing the Jetson

Ensure all the steps from Prepping the Jetson are done and it is connected to your Debian-based computer via USB-A to USB-C. Use `lsusb | grep NVIDIA` to verify the connection (there should be one device attached).

Start in your user home directory. **Execute each of the below steps one by one.** It is presented in block format to save space

```bash
sudo ufw allow nfs 

cd Downloads/

tar xf Jetson_Linux_R36.4.4_aarch64.tbz2

sudo tar -xpf Tegra_Linux_Sample-Root-Filesystem_R38.2.1_aarch64.tbz2 -C Linux_for_Tegra/rootfs/

cd Linux_for_Tegra
sudo ./tools/l4t_flash_prerequisites.sh

sudo ./apply_binaries.sh

sudo tar -xpf ../cuda_driver_36.4.4.tbz2 -C .

sudo chroot rootfs /bin/bash
apt --fix-broken install
systemctl enable ssh

exit

sudo tar -xpf kernel/kernel_supplements.tbz2 -C rootfs/

sudo ./apply_binaries.sh

# You may run into an error during this step saying no board connected or such, just unplug the Jetson and plug it back in and run the command again
sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1 \
     -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
     -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
     --showlogs --network usb0 jetson-orin-nano-devkit internal
```

This will run for a minute and open a ton of downloads folders. Just wait until it's finished. If you receive dpkg errors ignore them :)

### Post Flash Setup

**FIRST YOU NEED TO POWER CYCLE THE DEVICE.** Unplug the USB-C cable, the jumper wire, and the power supply. It should have no connections of any type in any port or pin.

Connect the Jetson to a monitor, keyboard, mouse, and power supply.

The system should boot up as normal for any Ubuntu installation. To connect the Jetson to unl-iot follow the separate guide for that. Follow the setup wizard, name the robot, and set the password to unlrobot.

After basic Ubuntu setup is done, make sure to install Jetpack. `sudo apt update && sudo apt upgrade -y && sudo apt install nvidia-jetpack`

## Installing Intel RealSense D435i Camera Software

### Setup

To install librealsense2, follow the instructions on this [website](https://jetsonhacks.com/2025/03/20/jetson-orin-realsense-in-5-minutes/)

### Setup Steps

All information taken from the [JetsonHacks librealsense repository](https://github.com/jetsonhacks/jetson-orin-librealsense), authored by the creators of the guide linked above in the **Setup** section.

1. First, clone the JetsonHacks repository for librealsense2.

   ```shell
   git clone https://github.com/jetsonhacks/jetson-orin-librealsense
   ```

2. Then, cd into `jetson-orin-realsense`
3. Once in the `jetson-orin-realsense` directory, follow the instructions in the README in the [JetsonHacks librealsense repository](https://github.com/jetsonhacks/jetson-orin-librealsense)

Once finished with the instructions in the JetsonHacks librealsense repository, make sure your realsense-viewer application is working and on. Then, on the left, turn on Stereo Module, RGB Camera, and Motion Module. Output should look like this:
![image]({% link attachments/Realsense-Output.png %})

## RPLidar Initial Setup (no Docker)

### Model: `RPLidar A3M1-R3`

Baud Rate: 256,000

### Model: `RPLidar S2M1-R2L`

Baud Rate: 1,000,000

### Docker Setup Steps

Ensure Jetson Orin is on Ubuntu 22.04, or that a Linux machine is being used

1. Plug in the Lidar to the Orin via USB

2. Check which port it is using and its permissions by running the command:

    ```bash
    ls -l /dev | grep ttyUSB
    ```

3. If the user doesn't have read and write permissions, run this command:

    ```bash
    sudo chmod 666 /dev/<ttyUSB0>
    ```

    *Make sure to replace `<ttyUSB0>` with the port your lidar is on!*

    **You must reboot OR logout and log back in for these permission changes to take effect**

4. Clone the rplidar_sdk git repository to your machine

   ```bash
   git clone https://github.com/Slamtec/rplidar_sdk
   ```

5. cd into `/rplidar_sdk`
6. Compile/make by running this command in the `/rplidar_sdk` directory

   ```bash
   make
   ```

7. Next, to test that the Lidar is working and sending data properly, cd into `/rplidar_sdk/output/Linux/Release` and run

   ```bash
   ./ultra_simple --channel --serial /dev/<ttyUSB0> <baud_rate>
   ```

   * Ensure the port is correct (mine is connected on `ttyUSB0`)
   * Ensure the baud rate matches that of your lidar model (For example, A3 Lidars have `256000` baud rate, and S2 Lidars have `1000000` baud rate)
   * **TROUBLESHOOTING:** *If you get a "no such file or directory" error when running `./ultra_simple`, ensure it is in the `/Release` directory by running `ls`. If it is not, rerun `make` in the `~/rplidar_sdk` directory*

After running these steps, you should get an output like:

```yaml
Ultra simple LIDAR data grabber for SLAMTEC LIDAR.
Version: x.x.x
SLAMTEC LIDAR S/N: xxxxxxxxxxx...
Firmware Ver: x.x
Hardware Rev: x
SLAMTEC LIDAR health status: x
grab scan data...
theta:  10.53 Dist: 0213.25 Q:47
theta:  10.72 Dist: 0214.75 Q:47
...
```

## RPLidar Setup With ROS2 and RViz2 (with Docker)

An `RPLidar S2M1-R2L` was used for this portion of the setup.

### Initial Check

***DISCLAIMER:** This setup assumes you are using a Docker container to run the Lidar in RViz. If you are not using Docker, the setup will be similar, but you will need to run most of the commands in the dockerfile manually*

Before doing anything, ensure that you are able to see the LiDAR on one of your serial ports by running

```bash
lsusb
```

OR

```bash
ls -l /dev | grep ttyUSB
```

If your LiDAR appears on one of the ports, you may proceed

***WRITE THIS PORT DOWN!** it will be important when giving privileges when running the docker container*

### Setup Steps

1. Plug in the LiDAR to the Orin via USB. If you are using the `RPLidar S2M1-R2`, there will be a UART to Serial bridge (likely a CP2102) that you will need to connect the LiDAR to first before plugging the USB end of the bridge into the Orin. Drivers for the bridge should be automatically installed on recent Linux Kernels. This setup does not account for plugging the LiDAR directly into the Orin's GPIO pins.

2. Create Dockerfile
   Here is the Dockerfile used in the LiDAR Setup:

   **It is important that the osrf/ros:humble-desktop-full Docker image is used so that tools like Rviz2 come pre-installed**

   ```Dockerfile
   # Official base image with RViz, Gazebo, etc.
   FROM osrf/ros:humble-desktop-full

   # ------------------------------------------------------------
   # 1. Base tools and dependencies
   # ------------------------------------------------------------
   RUN apt-get update && apt-get install -y \
       sudo nano git usbutils udev iputils-ping net-tools build-essential \
    && rm -rf /var/lib/apt/lists/*

   # ------------------------------------------------------------
   # 2. Create non-root 'YOUR_DESIRED_USERNAME' user
   # ------------------------------------------------------------
   ARG USERNAME=<YOUR_DESIRED_USERNAME>
   ARG USER_UID=1000
   ARG USER_GID=$USER_UID

   # Give <YOUR_DESIRED_USERNAME> privileges needed to access serial ports
   RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
    && mkdir -p /home/${USERNAME}/.config \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

   # ------------------------------------------------------------
   # 3. Enable password-less sudo
   # ------------------------------------------------------------
   RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

   # ------------------------------------------------------------
   # 4. Install and build the official ROS2 RPLIDAR driver
   # ------------------------------------------------------------
   USER ${USERNAME}
   WORKDIR /home/${USERNAME}

   # Create a colcon workspace
   RUN mkdir -p ~/ros2_ws/src
   WORKDIR /home/${USERNAME}/ros2_ws/src

   # Clone the official ROS2 rplidar repository
   RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

   # Build the workspace
   WORKDIR /home/${USERNAME}/ros2_ws
   RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

   # Make ROS2 workspace auto-sourced for new shells
   RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc && \
       echo 'source ~/ros2_ws/install/setup.bash' >> /home/${USERNAME}/.bashrc

   # ------------------------------------------------------------
   # 5. Copy optional config files (if you have them)
   # ------------------------------------------------------------
   COPY entrypoint.sh /entrypoint.sh
   COPY --chown=${USERNAME}:${USERNAME} .bashrc /home/${USERNAME}/.bashrc


   # ------------------------------------------------------------
   # 6. Default entrypoint
   # ------------------------------------------------------------
   ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
   CMD ["bash"]
   ```

   The corresponding `.bashrc` file in the same directory as the Dockerfile:

   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
   source ~/ros2_ws/install/setup.bash
   ```

   And the `entrypoint.sh` file in the same directory as the Dockerfile and `.bashrc`:

   ```bash
   #!/bin/bash

   set -e

   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash

   # Fixing permissions for USB serial devices
   if [ -e <YOUR_DEVICE_PORT> ]; then
       echo "Setting permissions for /dev/ttyUSB0..."
       sudo chmod 777 <YOUR_DEVICE_PORT> || echo "Failed to chmod /dev/ttyUSB0"
   fi

   echo "Provided arguments: $@"

   exec $@
   ```

   * replace <YOUR_DEVICE_PORT> with the USB port that your LiDAR is on (found in the **Initial Check** section of this page) - should be something like `/dev/ttyUSB0`

3. Ensure you are in the same directory as you Dockerfile, then build your Docker image:

   ```bash
   docker build -t <YOUR_IMAGE_NAME> .
   ```

   * replace `<YOUR_IMAGE_NAME>` with what you want to name you Docker image

4. Give docker permissions to your X11 Window Manager by running this command (necessary for displaying RViz)

   ```bash
   xhost +local:docker
   ```

   ***NOTE:** If you are using a Wayland-based compositor, you may need to install XWayland if it is not already installed. You can check this by seeing if the `/tmp/.X11-unix` directory exists*

5. Run a Docker container based on your image:

   ```bash
   docker run -it --user <YOUR_DESIRED_USERNAME> \
   -v $PWD/<YOUR_SOURCE_CODE_DIR>:/<YOUR_CONTAINER_SOURCE_CODE_DIR> \
   -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
   --env=DISPLAY \
   --privileged \
   --device <YOUR_DEVICE_PORT>:<YOUR_DEVICE_PORT> \
   --device-cgroup-rule='c *:* rmw' <YOUR_IMAGE_NAME>
   ```

   * replace `<YOUR_DESIRED_USERNAME>` with the username you chose in the Dockerfile
   * replace `<YOUR_SOURCE_CODE_DIR>` with the relative path of the directory your source code resides in
   * replace `<YOUR_CONTAINER_SOURCE_CODE_DIR>` with the absolute path of the directory you want your source code directory to be mounted in inside of the container
   * replace <YOUR_DEVICE_PORT> with the USB port that your LiDAR is on (found in the **Initial Check** section of this page) - should be something like `/dev/ttyUSB0`
   * replace `<YOUR_IMAGE_NAME>` with the name of your Docker image
   This command will open a shell in your docker container

6. Once inside the shell in your active container, run

   ```bash
   ls -l /dev/ttyUSB*
   ```

   to ensure your LiDAR shows up (should show something like `/dev/ttyUSB0`)

7. Start the LiDAR:

   ```bash
   ros2 launch rplidar_ros view_rplidar_s2_launch.py \
     serial_port:=/dev/ttyUSB0 \
     serial_baudrate:=1000000 \
     frame_id:=laser
   ```

8. If RViz does not immediately pop up, open another terminal (on the Orin, NOT in the container), and get into another shell inside the container with this command:

   ```bash
   docker exec -it <CONTAINER_NAME> /bin/bash
   ```

   To find `<CONTAINER_NAME>`, run the command

   ```bash
   docker ps
   ```

   in the terminal, and use the ContainerID field (should just be a series of numbers) associated with the container whose name you want to find.

9. In the separate shell, launch Rviz:

   ```bash
   rviz2
   ```

   The LiDAR data should be under the fixed frame titled "laser" and should appear automatically.

## Installing OV9281 Global Shutter UVC Camera Software

### Setup Steps

Additional Documentation for this camera can be found on the [Arducam Wiki](https://docs.arducam.com/UVC-Camera/Appilcation-Note/External-Trigger-Mode/OV9281-Global-Shutter/).

1. Plug the camera into the Jetson Orin via USB
2. Install v4l utility packages

   ```shell
   sudo apt-get install v4l-utils
   ```

3. List UVC devices connected to the USB ports (if you have multiple devices connected, multiple devices will show up! ensure that your Arducam device shows up)

   ```shell
   v4l2-ctl --list-devices
   ```

4. Install `guvcview` to test live video stream

   ```shell
   sudo apt install v4l-utils guvcview
   ```

5. Run `guvcview`, and select appropriate video input stream when prompted

   ```shell
   guvcview
   ```

Video stream from `guvcview` should look something like this:
![image]({% link attachments/Camera-Software-Video-Stream.png %})

### Working with Rviz2

Make sure that ROS2 is installed first!!

1. Install v4l2 camera node

   ```shell
   sudo apt install ros-humble-v4l2-camera
   ```

2. Launch

   ```shell
   ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
   ```

   Replace `video0` with the video stream corresponding to your camera (may be different if you have multiple cameras connected). For example, with the D435i connected, my video streams for the OV9281 were `video6` and `video7`.
3. In a separate terminal, view image:

   ```shell
   ros2 run image_tools showimage
   ```

4. Or, use Rviz to view (in a separate terminal as well)

   ```shell
   rviz2
   ```

   If using Rviz to visualize, select `Add` on the bottom left to add an image, go to the topics tab, find the `/image_raw topic`, and then select `Image`, and press `OK`. Rviz should look something like this, with the live video stream in the bottom left corner.
   ![image]({% link attachments/Camera-Software-RVIZ.png %})

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
> Author: Caleb Hans (<https://github.com/caleb-hansolo>)
