---
title: Installing Intel REalsense D435i Camera Software on Jetson
parent: Software Installation Guides
nav_order: 2
---

# Installing Intel REalsense D435i Camera Software on Jetson

## Setup

To install librealsense2, follow the instructions on this [website](https://jetsonhacks.com/2025/03/20/jetson-orin-realsense-in-5-minutes/?utm_source=chatgpt.com)

## Setup Steps

All information taken from the [JetsonHacks librealsense repository](https://github.com/jetsonhacks/jetson-orin-librealsense), authored by the creators of the guide linked above in the **Setup** section.

1. First, clone the JetsonHacks repository for librealsense2.

   ```shell
   git clone https://github.com/jetsonhacks/jetson-orin-librealsense
   ```

2. Then, cd into `jetson-orin-realsense`
3. Once in the `jetson-orin-realsense` directory, follow the instructions in the README in the [JetsonHacks librealsense repository](https://github.com/jetsonhacks/jetson-orin-librealsense)

Once finished with the instructions in the JetsonHacks librealsense repository, make sure your realsense-viewer application is working and on. Then, on the left, turn on Stero Module, RGB Camera, and Motion Module. Output should look like this:
![image]({% link attachments/Realsense-Output.png %})

> Author: Caleb Hans (<https://github.com/caleb-hansolo>)