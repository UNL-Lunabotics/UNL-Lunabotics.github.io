---
title: Generating the Ubuntu Virtual Machine
parent: macOS Setup
nav_order: 1
---

## Generating the Ubuntu Virtual Machine

On macOS, there is currently no way to get a fully working Docker with device pass-through. [Apple's Hypervisor Virtualization Framework](https://developer.apple.com/documentation/virtualization) only recently added support for device pass-through, but Docker has not implemented it yet. For more information, see the open [issue](https://github.com/docker/roadmap/issues/511).

Instead, we will be setting up a robust virtual machine running Ubuntu Linux by running a script to generate it.

{: .note}
This will consume more power and battery than a Docker setup, as the VM is a full operating system. Later, this guide will show you how to tune the VM's settings to be more power-efficient if needed.

### Install UTM

[UTM](https://mac.getutm.app/) is a fully open-source virtual machine hypervisor that we will be using. Download it and then open the `UTM.dmg`. Then, drag-and-drop the `UTM.app` to the `Applications` folder.

UTM does not need to be open yet, but does need to be downloaded for the script to run.

### Download the Script

Now that UTM is installed, you can download a script that creates a new virtual machine for you.

Download the [scripts_and_prebuilts](https://github.com/UNL-Lunabotics/scripts_and_prebuilts/archive/refs/heads/main.zip) repository, and extract and open the folder. Then, navigate to the `scripts` folder. Copy the `macOS Ubuntu VM` folder to your Desktop. You can delete the `scripts_and_prebuilts` folder if you want.

This will include five files. The first one is `build_vm.sh`, the main script you will be running. There are 2 helper scripts which tell UTM to generate and configure the vm. There are also 2 config files, which tells the virtual machine what programs to install. Feel free to poke around these. There are comments telling what each part does. You are free to modify any part of the script, though it is not recommended to.

### Prepare the Script

Now that the script is installed, you need to give it permission to run. To do this, go into your terminal and point it to wherever the script is located (e.g. `cd ~/Desktop/"macOS Ubuntu VM"`). Then type `sudo chmod +x build_vm.sh` and type in your Mac password to give it permission to run.

{: .important}
UTM **MUST** be downloaded for the script to run. The helper script communicates with UTM directly and will error out if UTM is not installed. UTM does not need to be running for the script to work.

### Run the Script

Ensure the script can run by typing `./build_vm.sh --help`. This should result in documentation on how to use this script. As you may see, this script is fairly customizable, letting you modify a lot of variables. If needed, add any configuration you would like, though the default should be good.

Most of these configurations can be edited after-the-fact, so you are not 'locked in' to the configuration made by this script, except for the Ubuntu/ROS version.

Run the script using `./build_vm.sh`, optionally extending it with configuration (e.g. `./build_vm.sh --username bob`). This will download the Ubuntu 24.04 Desktop Image for ARM64, generate the finalized config file, make the virtual machine, then start running it with UTM.

{: .important}
The script may prompt for permission to access UTM. Allow these permissions, otherwise the script will fail.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
