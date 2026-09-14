---
title: WSL
parent: Technical
nav_order: 3
---

## WSL Install and Setup

What is WSL? It stands for Windows Subsystem for Linux. It basically allows you to install an entire Linux operating system on your computer that you can access as a subsystem. Your primary operating system will still be Windows, you can just access Linux through a command line and develop in it as if it were a real Linux machine. This has it's limitations, but it should work enough for ROS2 development kinda sorta.

You can follow the [official Windows WSL install instructions](https://learn.microsoft.com/en-us/windows/wsl/install) at the link, but I'll repeat it here for convenience.

1. Open PowerShell in administrator mode
    - Press the Windows button on your keyboard or by pressing the menu in the taskbar
    - Type in PowerShell
    - Click on the Run as administrator option
2. Type in the command `wsl --install` to install WSL itself
3. Restart your computer
4. Open PowerShell again (admin mode not required)
5. Type in the command `wsl --list --online` to view the available Linux distributions
6. Type in the command `wsl --install Ubuntu-24.04`
7. Type in your Unix username (doesn't really matter)
8. Type in your Unix password (remember this you will be typing it a lot, or write it down)
9. Close that PowerShell window and do not continue working in it or it'll break things
10. Type WSL in the windows start menu and open the app that looks like a penguin (this will be how you open WSL any time you want to)

Now that WSL is installed, you have to add an SSH key in order to be able to use GitHub.

1. Open WSL using the app NOT PowerShell (see above)
2. Set up a GitHub SSH key so you can pull/push in WSL
    - Type the command `ssh-keygen -t ed25519 -C "your_email@example.com"`
    - Press enter to accept default file location
    - Optionally enter a passphrase
    - Type the command `cat ~/.ssh/id_ed25519.pub`
    - Highlight the entire output, starting with ssh-ed25519 and ending with your email address, and copy it
    - Go to https://github.com/settings/keys
    - Click the New SSH key green button in the topish right
    - Put whatever title you want (ex. WSL_Ubuntu-24.04 or something)
    - Paste the contents of the cat command into the Key box
    - Click the Add SSH key at the bottom
3. Type the command `mkdir repos && cd repos`
4. Clone the repository via SSH
    - Go to the repository on github.com
    - Click the green button that says Code
    - Switch over to the SSH tab
    - Copy that
    - In the WSL terminal, type `git clone <whatyoucopied>`

## USB Passthrough

**WSL does not have access to any USB devices on your computer by default.** You have to manually link them. If you do not link them, nothing connected to your computer by USB will be accessible by WSL.

{: .note}
If USB passthrough is not important to you right now, it is recommended to skip this section and come back to it later, especially if you are new. Enabling device passthrough requires modifying the WSL kernel which is very technical.

### Setting Up the Custom Kernel

Assuming you have already installed wsl, you can begin setting up device passthrough by downloading [this](https://github.com/TheKing349/WSL2-Linux-Kernel/releases/latest/download/vmlinux+modules.zip) file, which is a modified kernel of WSL. It is open-sourced as a [fork](https://github.com/TheKing349/WSL2-Linux-Kernel) of the WSL2-kernel source code to include controller support.

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

### Testing USB Passthrough

Restart whatever container you are using, connect to it again, and then type `lsusb`, It should print the name of the passed through USB device.

Next, verify you have the correct permissions to use the device: `ls /dev/input`.

Here you should see something like the following:

```bash
event0  js0
```

If you instead get a permission error, restart the container again, or type this command: `sudo chmod -R 666 /dev/input && sudo chmod +x /dev/input`, and then `ls /dev/input` again.

If you get an error saying

```bash
ls: cannot access '/dev/input': No such file or directory
```

then verify `lsusb` shows a device, and make sure the controller is successfully passed through with `usbipd`.

Finally, type `evtest` (install if needed: `sudo apt update && sudo apt install evtest`). It should recognize the controller, and give you a number to type in. Then, move the joystick around and press the keys. `evtest` should output text displaying what you did.

If `evtest` failed to give you a number, then you do not have the correct permissions on `/dev/input`.

### Automated Script

Experimentally, you can run the automated Docker setup script that will set up USB passthrough on WSL for you. This has not been confirmed to work. To do so, ensure the repository you have cloned has a `/scripts/docker_setup.sh` file, and then run the command `./scripts/docker_setup.sh -w`. This is not necessary for base functionality.

### References

This was possible due to an issue on [Microsoft's WSL GitHub Page](https://github.com/microsoft/WSL/issues/7747), and the original [fork of WSL2-kernel source code](https://github.com/atticusrussell/WSL2-Linux-Kernel).

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
