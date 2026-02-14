---
title: Generating the Ubuntu Virtual Machine
parent: macOS Setup
nav_order: 1
---

## Generating the Ubuntu Virtual Machine

On macOS, there is currently no way to get a fully working DevContainer with device passthrough. [Apple's Hypervisor Virtualization Framework](https://developer.apple.com/documentation/virtualizatio) only recently added support for device passthrough, but Docker has not implemented it yet. For more information, see the open [docker issue](https://github.com/docker/roadmap/issues/511).

Instead, we will be setting up a robust virtual machine running Ubuntu Linux.

{: .note}
This will consume more power and battery than a DevContainer as the VM is a full operating system. Later, this guide will show you how to tune the VM's settings to be more power-efficient if needed.

To set up the VM, we will be running a script, made by me, to generate a VM.

### Install Prereqs

To run these scripts, we need to install some command-line tools. First we will install [Homebrew](https://brew.sh/), a package manager for macOS. To do this, run `/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"`

Next, install `qemu` and `cdrtools`: `brew install qemu && brew install cdrtools`

{: .important}
These commands may require admin privileges. If it prompts for a password, use the same password you would to log onto your machine.

Lastly, we need to download [UTM](https://mac.getutm.app/). UTM is a fully open-source virtual machine hypervisor that we will be using. Download using the link above. Open the downloaded `UTM.dmg` and drag-and-drop the `UTM.app` to the `Applications` folder.

### Download the Script

{: .warning}
This documentation is not complete yet. The download for the script is not public, and therefore cannot be downloaded. Please update this documentation when this is finalized...

Now that everything is installed, you can download the script [here](LINK_HERE). This will include three files. The first one is `build-vm.sh`, the main script you will be running. There are also two config files. One is to tell UTM (a virtual machine hypervisor -- more on this later) how the VM should run. The second is a file that tells the virtual machine what tools to install. Feel free to poke around these. There are comments telling what each part does. You are free to modify any part of the script, though you shouldn't need to.

{: .note}
I recommend putting this script somewhere other than your Downloads folder for better organization. When we run the script, the virtual machine will be created inside the folder where the script lives. You are able to move the generated VM file wherever you'd like, however

### Running the Script

To run the script, go into your terminal, and point it to wherever the script is located: `cd ~/Downloads/FOLDER_HERE??`. Then `chmod +x build-vm.sh` to give it permission to run. Finally, run the script using `./build-vm.sh`. This will download the Ubuntu 24.04 Live Server Image for ARM64, generate the finalized config files, and export it to a `.utm` file.

### Customization

The script is very customizable. You can modify the name of the VM, the Display Resolution, amount of memory, and which Ubuntu version to use. For more information, look at the documentation header inside `build-vm.sh`.

{: .note}
Most of these customizations can be made after-the-fact, so you are not 'locked in' to how it is configured by default.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
