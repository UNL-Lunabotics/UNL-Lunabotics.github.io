---
title: Running the Ubuntu Virtual Machine
parent: macOS Setup
nav_order: 2
---

# Runing the Ubuntu Virtual Machine

Now that the virtual machine is created, we need to actually run it.

To do this, open Finder and navigate to where the script is. You should be a new file titled `Ubuntu 24.04 ARM64.utm`(or another name if changed). Double-click on it. This will launch UTM and add a new VM entry. Run the virtual machine by pressing the Play button.

The initial setup will take a while. On my machine it took ~20 minutes to fully configure. This is because it is installing needed packages and applications on the fly when it's run for the first time. The VM will restart and configure some more things. Eventually, you shoud be met with a login screen.

The default password for the user `workstation` is `unlrobot123`. If you changed this in script, the password will be whatever you set.

{: .note}
If you ran this on Ubuntu 22.04, some items don't get configured right away. Instead, manually restart the virtual machine. This will trigger some items to be installed.

## Applications

Once you are logged in, you will see a few apps on the sidebar. The first one is [Zen](https://zen-browser.app/), which is my preferred browser. Of course, are able to install a different one if you so choose.

Open Zen at least once, and make sure you enable it as the default browser in the setup.

You will also see [VSCode](https://code.visualstudio.com/), which is the IDE of choice. For more information, see [How to Use VSCode](https://github.com/UNL-Lunabotics/lunabotics-documentation/wiki/How-to-Use-VSCode).

To clone any `UNL-Lunabotics` repository, I recommend using `gh auth login` and signing in with your GitHub account. Then, use `git clone [REPOSITORY_HERE]` where `[REPOSITORY_HERE]` is the URL of the `UNL-Lunabotics` GitHub page. You can also clone using VSCode if you want.

## Virtual Machine Quirks

Because you are on a "separate" operating system, some keybinds have changed.

Most notably, the `Command` key on your keyboard **ONLY** acts as the 'Windows' button (Meta Key), opening the Ubuntu App Launcher. It is **NOT** like macOS where you can do `Cmd+S` to save in Ubuntu.

Instead, the `Control` key is what you use, like you would on a Windows computer. So, use `Ctrl+S`, `Ctrl+A`, etc. You can change this in UTM Settings. To do this, go into `Menu Bar > UTM > Settings > Input` and toggle the "Swap Control and Command keys" near the bottom.

However, `Cmd+Q` **will** still force-quit the application, forcibly shutting down the Virtual Machine.

## Closing the Virtual Machine

To safely quit the virtual machine, you'll do a soft-shutdown inside the VM. To do this, click the System Menu on the very top-right corner, with the volume and power icons. Then, click the Power icon, and click "Power off".

Avoid using `Cmd+Q` or the Power button on UTM itself, unless the virtual machine is unresponsive. This is because the virtual machine may get corrupted if doing important tasks (i.e. running a command, etc.)

## Changing Username or Password

If you want to change the username or password, go into `Settings > System > Users` and press "Unlock" on the top-right. Enter the password of `unlrobot123`. Now, change the username or password.

## Configuring the Virtual Machine

There are various items we can configure in the virtual machine.

### Changing Display Resolution

By default, the virtual machine is configured to display at `3456x2160`, the resolution of my 2024 MacBook Pro 16-inch. Look up your specific Mac model and find your display resolution. If they don't match, you'll have to change it in the virtual machine.

To do this, start the virtual machine and log in. Then type the following in a terminal: `sudo nano /etc/default/grub`. Type in the password (`unlrobot123` by default), then find the line that says `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash video=3456x2160@120"`. Use your arrow keys to change the location of your cursor, and replace the `3456x2160` to your resolution. You **MUST** keep the `@120`.

To save the configuration file, you'll do `Ctrl+X`, and `Enter` to save. Then, to load your new changes, type `sudo update-grub`, and then `sudo reboot`.

Log in again, go into `Settings > Displays` and locate your resolution in the `Resolution` dropdown. I also recommend doing `200%` scale. Then click "Apply" in the top-right corner.

### Changing VM Resources

As discussed before, the VM will use more battery as it is a full operating system. To fix this, you can give the VM less of your computer resources, at the cost of performance.

To do this, go into the main UTM app, and right-click on the VM and click "Edit". Then go into "System". Here you can modify how much RAM to give the VM, as well as how many CPU cores to allocate to it.

To reduce power consumption, lower these numbers. To increase performance on the VM, increase those numbers.

### Passing a USB device through

To pass a USB device, first plug in the controller, and turn on the VM if you haven't already. Then, in the top-right corner on UTM's Menu Bar, there should be a USB icon. Click on it, find your device, and click "Connect". Try running `evtest` or launch `jstest-gtk` and verify it works.

### Enabling a Shared Directory

If you find you need to share files/folders across your VM and macOS, follow the official UTM [guide](https://docs.getutm.app/guest-support/linux/#virtfs). Note that it is a little technical but should be doable.

That's it! Everything else works just like Ubuntu Linux. You are able to develop just as you would on native hardware.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)