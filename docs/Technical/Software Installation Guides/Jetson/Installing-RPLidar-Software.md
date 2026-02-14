---
title: Installing RPLidar Software
parent: Jetson-Specific
nav_order: 3
---

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

***WRITE THIS PORT DOWN!** it will be important when giving priveleges when running the docker container*

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
   # 3. Enable passwordless sudo
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

> Author: Caleb Hans (<https://github.com/caleb-hansolo>)
