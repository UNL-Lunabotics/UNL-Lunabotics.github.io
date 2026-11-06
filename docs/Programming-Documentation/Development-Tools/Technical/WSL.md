---
title: WSL
parent: Technical
grand_parent: Development Tools
nav_order: 1
---

# WSL

{: .important}
This is a WINDOWS EXCLUSIVE feature. If you are on Linux or MacOS, there is no need to follow this guide.

## Installation

Follow the [official Microsoft installation guide](https://learn.microsoft.com/en-us/windows/wsl/install) to install WSL. Generally, for Lunabotics purposes, you will just run the command `wsl.exe --install -d Ubuntu-24.04`.

Remember the password you pick! You will need it often.

You can verify it works by pressing the Windows button, typing `WSL` and seeing a penguin app available. Additionally, if you open a terminal or Powershell and type `wsl` it should enter your default distribution.

## Configuration

If you install multiple WSL distributions, you can set a default using `wsl.exe --set-default <Distro>`.

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

## Setting up the Custom Kernel

Enabling Device Passthrough will allow you to pass a USB device, such as a controller, to the Docker container. There are two ways to do this. It is **HIGHLY** recommended to use the script, as it will automate a lot of steps. You can find the script in the [Ultimate Docker Guide]({% link docs/Programming-Documentation/Development-Tools/Technical/Ultimate-Docker-Guide.md %}). However, for completeness, the manual guide is here too.

{: .important}
This process, whether you do it manually or through the script, requires a custom WSL kernel. This means that we are no longer using the official Microsoft WSL, which may break compatibility for certain apps. Everything done here is reversible.

### Installing the Custom Kernel Manually

{: .warning}
This kernel can be automatically installed for you when following the steps at [Ultimate Docker Guide]({% link docs/Programming-Documentation/Development-Tools/Technical/Ultimate-Docker-Guide.md %})

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

### Attach Controller to WSL

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

### Testing Controller

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

### Troubleshooting

Sometimes a step in the device passthrough may error out (REWORD!). Here are some common troubleshooting tips:

#### Permission Errors

If you get a permission error at any point, restart the Docker container again. If that doesn't work, then type the following command:

```bash
sudo chmod -R 666 /dev/input && sudo chmod +x /dev/input
```

#### No Such File or Directory

If you get the following error:

```bash
ls: cannot access '/dev/input': No such file or directory
```

Verify `lsusb` shows a device, and make sure the controller is successfully passed through with usbipd.

#### No `jstest` Entry

If `jstest-gtk` does not show an entry for your controller, ensure the controller is connected and passed through usbipd, and that `ls /dev/input` shows an input device.

### References

This was only possible due to an issue on [Microsoft's WSL GitHub Page](https://github.com/microsoft/WSL/issues/7747), and the original [fork of WSL2-kernel source code](https://github.com/atticusrussell/WSL2-Linux-Kernel).

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
>
> Last Reviewed: 2026-11-05
