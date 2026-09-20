---
title: Remote Access
parent: Technical
nav_order: 4
---

## Table of contents

{:toc}

## Remote Setup

Because ROS2 requires Ubuntu to run properly, this guide shows how to remotely connect to one of our Mini PC's that is already set up on Ubuntu with ROS2. For more information about ROS2, see [Learning ROS2]({% link docs/Curriculum/Learning-ROS2.md %})<!-- #TODO: broken link -->.

## Tailscale Setup

Tailscale is a special VPN that allows one computer to gain access to another computer remotely, through a dedicated IP Address. This is used to connect your computer to the Mini PC.

### Download

Download Tailscale onto your computer using [this](https://tailscale.com/download) link.

### Log in

Once installed, a window should pop up saying "Join your network". Click "Sign in to your network". If on Linux, you'll instead type `sudo tailscale login` into a terminal and open the link.

Once at the login page, click "Sign in with Google." Then go to the Lunabotics Google Drive and find `Subteams > Programming > IMPORTANT INFO`. Note the "EMAIL INFO" within this document, and paste the address and password in when prompted.

Once signed in with the Google account, click the blue "Connect".

### Using Tailscale

On Windows or macOS there should now be a system tray entry for Tailscale. Open it (on Windows you have to right-click), and you should see a menu pop up. Hover over and navigate to `Network Devices > Tagged Devices > mini-pc-1`. Click on `mini-pc-1`. This should copy an IP Address to your clipboard. You will need this later on.

On Linux, type `sudo tailscale up` and then `tailscale status | grep tagged` and you should see information about the Mini PC. Copy and IP Address shown as you will need this later on.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)

## VNC Setup

Virtual Network Computing (VNC) is a protocol that allows one computer to attach a desktop to another through an IP Address. For our use, this means we can use the Mini PC as a desktop that is visible on your computer through the Tailscale IP Address.

### VNC Client Download

VNC requires a 'client' or a program that utilizes the VNC protocol. I would recommend downloading either [RealVNC](https://www.realvnc.com/en/connect/download/viewer/) or [TigerVNC](https://sourceforge.net/projects/tigervnc/).

### Connect to SSH

Before you can use VNC, there's a script you need to run on the Mini PC to get a VNC 'session'. To do this, we will use SSH, a protocol to allow a computer's terminal to connect to another.

To do this, open your terminal and type in the following command: `ssh workstation@TAILSCALE_IP_HERE`, where `TAILSCALE_IP_HERE` is the IP Address that Tailscale copied to your clipboard in the [previous step]({% link docs/Technical/Setup Dev Tools/Remote/Tailscale-Setup.md %}#using-tailscale). Then type `INSERTUNL'SDEFAULTPASSWORDASKIFYOUDONTKNOWIT` for the password when it asks.<!-- #TODO: mentions an old section name that no longer exists -->

### Run the Script

I made a script to be able to easily create an isolated VNC instance. If you want to learn more about how the script works or how I got there, follow [this]({% link docs/Technical/Processes/Mini-PC-Remote-Connect.md %})<!-- #TODO: broken link --> link.

The script should be run as follows: `sudo ./connect-vnc.sh USERNAME_HERE`, where `USERNAME_HERE` is whatever username you want to use. The username you put doesn't matter, it just has to be unique (the script will error out if you input a username that already exists). When it asks for a password, type in `INSERTUNL'SDEFAULTPASSWORDASKIFYOUDONTKNOWIT`.

The script will then say "Your VNC instance is on port 5900." The number you see may differ, that's okay. Take note of it.

Keep this terminal open, as the moment you close it, the VNC instance will close.

### Connect to VNC

Now that we have a VNC instance running, you can connect to it.

In your VNC client of your choice, type in the following in the input box: `TAILSCALE_IP:PORT`, where again `TAILSCALE_IP` is the address of the Mini PC and `PORT` is the number the script gave you. Ensure you have the `:` in the middle.

You should now see a desktop and are able to run GUI applications on the Mini PC straight from your computer. You are free to develop, but take note on how to disconnect from VNC.

{: .note}
The created user is **temporary**. This means that any files or folders saved in your `home` directory will be deleted upon script exit. Please do not save any documents to your `home` folder, or move them before you disconnect.

### Disconnect from VNC

Apart from just closing VNC client, you'll also need to stop the script, which stops the VNC instance. To do this, go back to the terminal running the script and do the keybind `Ctrl+C`. Once the script exits, you can type `exit` to exit from SSH and are free to close the terminal.

That's it! You are able to connect and develop on the Mini PC remotely! Have fun developing!

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)

## Mini PC Remote Connection

This goes over the process of being able to remote connect to the Mini PC using VNC. This idea sounds simple, but required a lot of workarounds and hack-y solutions, but we got there.

### Initial Idea

The initial idea was simple: Use [Tailscale](https://tailscale.com/) to SSH into the Mini PC and develop within a CLI. The issue with this is that any GUI application (RViz, Gazebo, etc.) wouldn't run properly.

### New Idea

So I had the idea to use [VNC](https://en.wikipedia.org/wiki/VNC) with Tailscale to be able to run GUI applications.

### Ubuntu Problems

The VNC idea didn't seem hard to implement as VNC is a simple protocol. But as I later found out, Ubuntu does not like VNC.

#### VNC Server Issues

I used [TigerVNCServer](https://tigervnc.org/), which seemed like the best VNC Server for Ubuntu. The server will then start a process that you specify, such as the terminal or a file browser, which will then be shown on the VNC client. It spawns a headless display. This worked fine but when I tried to start a new GNOME session, the PC crashed. No matter the config, the VNC Server didn't like it.

I ended up installing [xfce](https://xfce.org/) and used that for the VNC desktop environment.

#### User locking display

The next issue I faced is user permission issues on displays. I'm not entirely sure the issue here, but I kept running into `fuse` permission errors when running the VNC server. My best guess is a logged-in user connected to a display expects a certain number of monitors to be connected, and when VNC tried to start a headless display, it panicked.

When I tried running a VNC instance on a user that didn't have a physical monitor connected, it seemed to work just fine.

#### Ports

Even when I got a VNC instance to run properly on a different user, the network port used is global. This means that if `userA` and `userB` want a VNC instance, we need to find the next available port, otherwise TigerVNCServer will always start it on port `5901`.

### Revised Workflow

All of these issues combined made me rethink the entire VNC process for the Mini PC. The new idea was to dynamically create a new temporary user, get an available port, then spawn the VNC instance on that port with xfce. This would allow for multiple people to develop on the Mini PC at once, including multiple on VNC.

### Script

With this new workflow, I created a bash script to do all of these things. The bash script requires a command-line argument for the temporary username. The script then creates the user, scans the network ports and finds the next available one, and then starts the VNC server. On script exit (when you do `Ctrl+C`), the script stops the VNC instance and destroys the user.

### Conclusion

This process was super overcomplicated and there's probably a better way, but this is what I found that works.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
