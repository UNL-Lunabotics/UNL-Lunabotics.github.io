---
title: Installing Ubuntu 24.04 + ROS2
parent: Mini PC
nav_order: 1
---

## Installing Ubuntu 24.04 + ROS2

This guide goes through setting up a new Mini PC with Ubuntu 24, ROS2 and various development tools.

### Explanation

The way this works is through [cloud-init](https://cloudinit.readthedocs.io/en/latest/index.html), a way to install Ubuntu with preconfigured items (accounts, packages, apps, etc.). Cloud-init is meant for servers or virtual machines so we have to also use Ventoy.

[Ventoy](https://www.ventoy.net) is an easy way to create a bootable USB. What's special about Ventoy is that it can boot from *multiple* ISO's when the computer boots and configure *how* it boots. This is needed for cloud-init because it will handle passing the cloud-init config for us.

### Prerequisites

Now that you know a little about how the process works, let's make sure you have everything needed for installation:

1. An 8GB+ USB Drive
2. A computer to set up the USB Drive
3. Download [Ubuntu 24.04 Desktop](https://releases.ubuntu.com/24.04.3/ubuntu-24.04.3-desktop-amd64.iso)
4. Download [Ventoy](https://www.ventoy.net/en/download.html)
5. Download the [Ventoy configuration]({% link attachments/ventoy-configuration.zip %})

### Preparing the USB Drive

If you're running Windows, open the `Ventoy2Disk.exe` program. If on Linux, run the `VentoyGUI.x86_64` program.

Once Ventoy is open, choose the USB Drive. Make sure it is the correct one and all data on the drive is backed up as it will erase anything off the drive. Then click 'Install' and confirm.

Next, go to your Files app and open the 'Ventoy' drive. Copy the `ubuntu-24.04.3-desktop-amd64.iso` onto this drive. Then unzip the `ventoy-configuration.zip` and copy the `ventoy` and `scripts` folder onto the drive. The structure should look as follows:

```bash
Ventoy Drive
├── isos
│   └── ubuntu-24.04.3-desktop-amd64.iso
├── scripts
│   └── ubuntu-autoinstall.yaml
└── ventoy
    └── ventoy.json
```

### Booting Mini PC with USB Drive

Now that the USB Drive is prepared, plug the drive in. Then turn on the Mini PC and press F7 to enter the boot options menu. Then select the USB Drive as the boot option.

{: .note}
The boot options button changes from computer to computer. On the Mini PC we currently use, F7 is used. If on a different computer, check its manual to find which button to use.

Once booted into Ventoy you should see an entry named `ubuntu-24.04.3-desktop-amd64.iso`. If you do not, verify that the USB Drive is configured [properly](#preparing-the-usb-drive).

Press Enter to select the Ubuntu iso. Press Enter again to select `Boot in normal mode`, and Enter again to select `Boot with scripts/ubuntu-autoinstall.yaml`.

Now Ubuntu will start booting, and you'll need to press Enter to select `Try or Install Ubuntu`.

### Installing Ubuntu

The installation is almost fully preconfigured, with only needing a valid network connection. Once Ubuntu boots up and finishes preparing, you will see a prompt that reads 'Connect to the Internet'.

#### Connecting to eduroam

Connect to NU-Guest, and a notification will prompt you to open the captive portal. Click the notification. Then click the button that reads "Connect wirelessly to eduroam".

Then, press the Windows Key and type 'Terminal', and Enter. In the terminal, type the following commands:

```bash
cd Downloads
chmod +x SecureW2_JoinNow.run
./SecureW2_JoinNow.run
```

Then sign in with your @huskers.unl.edu email and password.

Once the terminal says 'Connected', go back to the installer, click Next and then Install to install Ubuntu onto the Mini PC.

#### Finishing the Installation

Ubuntu and other packages (ROS2, VSCode, etc.) will now install which may take a while.

Once installed, click 'Reboot' and the PC will reboot into Ubuntu. There should be a user created named `workstation` and has the password of `unlrobot123`.

Log in and ensure ROS2 is installed correctly. Go to a terminal and type `ros2`. If installed, there should be text displayed.

That's it! Ubuntu is installed with everything needed to use it on the robot!

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
