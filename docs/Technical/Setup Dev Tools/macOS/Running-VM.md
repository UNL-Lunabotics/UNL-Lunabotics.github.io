---
title: Running the Ubuntu Virtual Machine
parent: macOS Setup
nav_order: 2
---

## Running the Ubuntu Virtual Machine
## Running the Ubuntu Virtual Machine

Once the script completes, UTM will open and the virtual machine will start to run. It will initially boot, then will open to a desktop environment. A window will appear and, after preparing, will prompt you for a password. You are welcome to change any fields you would like, but a password is required to continue.

Once a password is entered, "Next" and then "Install". This will download and install core Ubuntu and ROS2 packages, reboot, then do a post-install setup.

{: .important}
The initial setup will take a while. On a tested machine, it took ~15 minutes to fully configure, as it is downloading and installing a lot of packages.

Once rebooted, log in using the password you set.

The last thing to do is to go into `Settings > Display` and change the Resolution to `3456x2160` (default). I also like the scale set to `200%`. To change the resolution setting, see [Changing Display Resolution](#changing-display-resolution)

### Applications

Once you are logged in, you will see a few apps on the sidebar. The first one is [Zen](https://zen-browser.app/), which is a preferred browser. Of course, are able to install a different one if you so choose.

Open Zen at least once, and make sure you enable it as the default browser in the setup.

You will also see [VSCode](https://code.visualstudio.com/), which is the IDE of choice. For more information, see [How to Use VSCode](https://github.com/UNL-Lunabotics/lunabotics-documentation/wiki/How-to-Use-VSCode).

To clone any `UNL-Lunabotics` repository, I recommend using `gh auth login` and signing in with your GitHub account. Then, use `git clone [REPOSITORY_HERE]` where `[REPOSITORY_HERE]` is the URL of the `UNL-Lunabotics` GitHub page. You can also clone using VSCode if you want.

### Virtual Machine Quirks

Because you are on a "separate" operating system, some keybinds have changed.

Most notably, the `Command` key on your keyboard **ONLY** acts as the 'Windows' button (Meta Key), opening the Ubuntu App Launcher. It is **NOT** like macOS where you can do `Cmd+S` to save in Ubuntu.

Instead, the `Control` key is what you use, like you would on a Windows computer. So, use `Ctrl+S`, `Ctrl+A`, etc. You can change this in UTM Settings. If you would like, go into `Menu Bar > UTM > Settings > Input` and toggle the "Swap Control and Command keys" near the bottom.

However, `Cmd+Q` **will** still force-quit the application, forcibly shutting down the Virtual Machine.

### Closing the Virtual Machine

To safely quit the virtual machine, you'll do a soft-shutdown inside the VM. To do this, click the System Menu in the top-right corner, with the volume and power icons. Then, click the Power icon, and click "Power off".

Avoid using `Cmd+Q` or the Power button on UTM itself, unless the virtual machine is unresponsive. This is because the virtual machine may get corrupted if doing important tasks (i.e. running a command, etc.)

### Changing Username or Password

If you want to change the username or password, go into `Settings > System > Users` and press "Unlock" on the top-right. Type in your current password, then type a new password.

{: .note}
> Doing this may not change the `sudo` password used in the terminal. To change this, type `sudo passwd` in a terminal. Type in your current password, then type a new password.
>
> Similarly, you can type `passwd` to change your local password if you want.

### Configuring the Virtual Machine

There are various items we can configure in the virtual machine.

#### Changing Display Resolution

By default, the virtual machine is configured to display at `3456x2160`, the resolution of my 2024 16-inch MacBook Pro. Look up your specific Mac model and find your display resolution. If they don't match, you may want to change it in the virtual machine.

To do this, start the virtual machine and log in. Then type the following in a terminal:

```bash
sed -i /etc/default/grub -e 's/GRUB_CMDLINE_LINUX_DEFAULT=".*/GRUB_CMDLINE_LINUX_DEFAULT="console=tty0 video=[DISP_RES]@120"/'
```

Be sure to change `[DISP_RES]` at the end of this command to your desired resolution. You **MUST** keep the `@120` at the end. Example:
`... "console tty=0 video=3456x2160@120"/'`

Finally, update the changes by typing `sudo update-grub`, and then `sudo reboot`.

Log in again, go into `Settings > Displays` and locate your resolution in the `Resolution` dropdown. I also recommend doing `200%` scale. Then click "Apply" in the top-right corner.

#### Changing VM Resources

As discussed before, the VM will use more battery as it is a full operating system. To fix this, you can give the VM less of your computer resources, at the cost of VM performance.

To do this, power down the virtual machine, then go into the main UTM app. Right-click on the VM and click "Edit", then go into "System". Here you can modify how much RAM to give the VM, as well as how many CPU cores to allocate to it.

To reduce power consumption, lower these numbers. To increase performance on the VM, increase those numbers. Usually, CPU has the biggest impact on performance and battery.

#### Passing a USB device through

To pass a USB device, first plug in the device and turn on the VM. Then, in the top-right corner on UTM's Menu Bar, there should be a USB icon. Click on it, find your device, and click "Connect". Try running `lsusb` to confirm the device exists. If the USB device is a controller, you can type `sudo evtest` or `jstest-gtk` to see controller inputs.

#### Enabling a Shared Directory

If you find you need to share files/folders across your VM and macOS, follow the official UTM [guide](https://docs.getutm.app/guest-support/linux/#virtfs). Note that it is a little technical but should be doable. Also note that `spice-vdagent` is already installed, so you may skip this step.

That's it! Everything else works just like Ubuntu Linux. You are able to develop just as you would on native hardware.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
