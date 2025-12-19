---
title: Enabling Device Passthrough for Windows
parent: Windows Setup
nav_order: 4
---

# Enabling Device Passthrough for Windows

Enabling Device Passthrough will allow you to pass a USB device, such as a controller, to the DevContainer. Note that this requires modifying the WSL kernel, which is fairly technical.

## Install Ubuntu

Install Ubuntu on WSL using this command:
`wsl --install Ubuntu`

{: .note}
This command will require you to enter a username and password. You can enter anything into this, but I recommend a password you'll remember.

Then, set it as the default:
`wsl --set-default Ubuntu`

Type `wsl` to verify Ubuntu installed correctly, then type `exit` to exit from WSL.

## Set up the Custom Kernel

{: .warning}
The custom kernel is not finalized yet, and therefore cannot be downloaded (I need to make changes to it on GitHub). Please check back later to download it.

Download [this](LINK_HERE) file, which is a modified kernel of WSL. It is open-sourced as a [fork](https://github.com/atticusrussell/WSL2-Linux-Kernel) of the WSL2-kernel source code to include controller support.

Next, put the downloaded `vmlinux` to this folder: `C:\Users\[YOUR_USER]\`, where `[YOUR_USER]` is replaced with your username.

Then, make a new configuration file located here: `C:\Users\[YOUR_USER]\.wsl.config`. Put the following inside the file:

```conf
[wsl2]
kernel=C:\\Users\\[YOUR_USER]\\vmlinux
```

To load the kernel, you'll need to restart WSL: `wsl --shutdown` and wait 30 seconds. Then type `wsl` to load WSL again.

Then, verify the kernel is loaded: type `uname -r`. If the result starts with `5.15` and ends with `joystick+`, it is loaded successfully. You can now `exit` from WSL.

## Set up Device Passthrough

After that, download [usbipd-win](https://github.com/dorssel/usbipd-win/releases/latest). This will act as the bridge to allow device activity onto WSL.

Plug your device into your computer, and in a terminal and type `usbipd list`.

This command will show the names of different USB devices connected, as well as the bus-id. Find the bus-id of the device you want to pass through.

Once you have the bus-id, type `usbipd bind -b [BUS_ID_HERE] --force` in an **admin** terminal, where `[BUS_ID_HERE]` is you bus-id.
This will force your device to 'bind' to WSL.

{: .note}
This will 'unregister' your device in Windows. To have Windows see your device again, type `usbipd unbind -b [BUS_ID_HERE]`.

Then restart your computer, open VSCode and the DevContainer with your controller still plugged in.

In a terminal (does **not** have to be admin), type `usbipd list` again to make sure the bus-id is the same, and then execute `usbipd attach -w Ubuntu -b [BUS_ID_HERE]`, again replacing this command with your bus-id.

This will 'connect' your controller to WSL, which should allow input to work inside the DevContainer.

## Test Device Passthrough

Restart the DevContainer (`Ctrl+Shift+P` and `Reopen in DevContainer`).

In your DevContainer terminal, type `lsusb`. This should print the name of the passed through USB device.

Next, verify you have the correct permissions to use the device: `ls /dev/input`.

Here you should see something like the following:

```bash
event0  js0
```

If you instead get a permission error, restart the DevContainer again, or type this command: `sudo chmod -R 666 /dev/input && sudo chmod +x /dev/input`, and then `ls /dev/input` again.

If you get an error saying

```bash
ls: cannot access '/dev/input': No such file or directory
```

then verify `lsusb` shows a device, and make sure the controller is successfully passed through with `usbipd`.

Finally, type `evtest` (install if needed: `sudo apt update && sudo apt install evtest`). It should recognize the controller, and give you a number to type in. Then, move the joystick around and press the keys. `evtest` should output text displaying what you did.

If `evtest` failed to give you a number, then you do not have the correct permissions on `/dev/input`.

## References

This was possible due to an issue on [Microsoft's WSL GitHub Page](https://github.com/microsoft/WSL/issues/7747), and the [fork of WSL2-kernel source code](https://github.com/atticusrussell/WSL2-Linux-Kernel).

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)