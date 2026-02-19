---
title: Tailscale Setup
parent: Remote Setup
nav_order: 1
---

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
