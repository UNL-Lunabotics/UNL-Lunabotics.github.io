---
title: Installing Docker
parent: Development Tools Setup
nav_order: 3
---

# Installing Docker

Docker will let us create a virtual environment to use when developing for ROS2. This is because ROS2 **requires** base Ubuntu to run. If you are on base Ubuntu, Docker is not needed, but would still work.

You will use the installation guide provided by Docker depending on your operating system:

- [Linux](https://docs.docker.com/engine/install/ubuntu/)
- [Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
- [macOS](https://docs.docker.com/desktop/setup/install/mac-install/)

If on Windows or macOS, open Docker.

If on Windows, you get a prompt saying: "Your WSL version needs updating". To fix this:

1. Open the terminal
2. Type in `wsl --update`, which should be the same commmand that Docker gives you
3. Restart Docker

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)