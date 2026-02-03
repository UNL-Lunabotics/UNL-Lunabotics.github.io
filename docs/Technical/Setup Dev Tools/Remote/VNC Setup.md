---
title: VNC Setup
parent: Remote Setup
nav_order: 2
---

# VNC Setup

Virtual Network Computing (VNC) is a protocol that allows one computer to attach a desktop to another through an IP Address. For our use, this means we can use the Mini PC as a desktop that is visible on your computer through the Tailscale IP Address.

## VNC Client Download

VNC requires a 'client' or a program that utilizes the VNC protocol. I would recommend downloading either [RealVNC](https://www.realvnc.com/en/connect/download/viewer/) or [TigerVNC](https://sourceforge.net/projects/tigervnc/).

## Connect to SSH

Before you can use VNC, there's a script you need to run on the Mini PC to get a VNC 'session'. To do this, we will use SSH, a protocol to allow a computer's terminal to connect to another.

To do this, open your terminal and type in the following command: `ssh workstation@TAILSCALE_IP_HERE`, where `TAILSCALE_IP_HERE` is the IP Address that Tailscale copied to your clipboard in the [previous step]({% link docs/Technical/Setup Dev Tools/Remote/Tailscale Setup.md %}#using-tailscale). Then type `unlrobot123` for the password when it asks.

## Run the Script

I made a script to be able to easily create an isolated VNC instance. If you want to learn more about how the script works or how I got there, follow [this]({% link docs/Technical/Processes/Mini PC Remote Connect.md %}) link.

The script should be run as follows: `sudo ./connect-vnc.sh USERNAME_HERE`, where `USERNAME_HERE` is whatever username you want to use. The username you put doesn't matter, it just has to be unique (the script will error out if you input a username that already exists). When it asks for a password, type in `unlrobot123`.

The script will then say "Your VNC instance is on port 5900." The number you see may differ, that's okay. Take note of it.

Keep this terminal open, as the moment you close it, the VNC instance will close.

## Connect to VNC

Now that we have a VNC instance running, you can connect to it.

In your VNC client of your choice, type in the following in the input box: `TAILSCALE_IP:PORT`, where again `TAILSCALE_IP` is the address of the Mini PC and `PORT` is the number the script gave you. Ensure you have the `:` in the middle.

You should now see a desktop and are able to run GUI applications on the Mini PC straight from your computer. You are free to develop, but take note on how to disconnect from VNC.

{: .note}
The created user is **temporary**. This means that any files or folders saved in your `home` directory will be deleted upon script exit. Please do not save any documents to your `home` folder, or move them before you disconnect.

## Disconnect from VNC

Apart from just closing VNC client, you'll also need to stop the script, which stops the VNC instance. To do this, go back to the terminal running the script and do the keybind `Ctrl+C`. Once the script exits, you can type `exit` to exit from SSH and are free to close the terminal.

That's it! You are able to connect and develop on the Mini PC remotely! Have fun developing!

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
