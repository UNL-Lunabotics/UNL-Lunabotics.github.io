---
title: Mini PC Remote Connection
parent: Processes
nav_order: 1
---

# Mini PC Remote Connection

This goes over the process of being able to remote connect to the Mini PC using VNC. This idea sounds simple, but required a lot of workarounds and hack-y solutions, but we got there.

## Initial Idea

The initial idea was simple: Use [Tailscale](https://tailscale.com/) to SSH into the Mini PC and develop within a CLI. The issue with this is that any GUI application (RViz, Gazebo, etc.) wouldn't run properly.

## New Idea

So I had the idea to use [VNC](https://en.wikipedia.org/wiki/VNC) with Tailscale to be able to run GUI applications.

## Ubuntu Problems

The VNC idea didn't seem hard to implement as VNC is a simple protocol. But as I later found out, Ubuntu does not like VNC.

### VNC Server Issues

I used [TigerVNCServer](https://tigervnc.org/), which seemed like the best VNC Server for Ubuntu. The server will then start a process that you specify, such as the terminal or a file browser, which will then be shown on the VNC client. It spawns a headless display. This worked fine but when I tried to start a new GNOME session, the PC crashed. No matter the config, the VNC Server didn't like it.

I ended up installing [xfce](https://xfce.org/) and used that for the VNC desktop environment.

### User locking display

The next issue I faced is user permission issues on displays. I'm not entirely sure the issue here, but I kept running into `fuse` permission errors when running the VNC server. My best guess is a logged-in user connected to a display expects a certain number of monitors to be connected, and when VNC tried to start a headless display, it panicked.

When I tried running a VNC instance on a user that didn't have a physical monitor connected, it seemed to work just fine.

### Ports

Even when I got a VNC instance to run properly on a different user, the network port used is global. This means that if `userA` and `userB` want a VNC instance, we need to find the next available port, otherwise TigerVNCServer will always start it on port `5901`.

## Revised Workflow

All of these issues combined made me rethink the entire VNC process for the Mini PC. The new idea was to dynamically create a new temporary user, get an available port, then spawn the VNC instance on that port with xfce. This would allow for multiple people to develop on the Mini PC at once, including multiple on VNC.

## Script

With this new workflow, I created a bash script to do all of these things. The bash script requires a command-line argument for the temporary username. The script then creates the user, scans the network ports and finds the next available one, and then starts the VNC server. On script exit (when you do `Ctrl+C`), the script stops the VNC instance and destroys the user.

## Conclusion

This process was super overcomplicated and there's probably a better way, but this is what I found that works.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)