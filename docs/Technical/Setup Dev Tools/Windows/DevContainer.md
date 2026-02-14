---
title: Setting up the DevContainer
parent: Windows Setup
nav_order: 3
---

## Setting up the DevContainer

DevContainers will use Docker to create the virtual environment mentioned before.

### Cloning the Repository

Open VSCode and click on "Clone Git Repository". Sign in with GitHub, and clone the `UNL-Lunabotics/terrence_2.0` repo.

### Notes

Make sure Docker Desktop is running **before** launching the DevContainer, otherwise VSCode will error out.

Before starting the DevContainer, add the following lines of code to `.devContainer/devcontainer.json`, after line `29`.

```js
    "features": {
        "ghcr.io/devcontainers/features/desktop-lite:1": {}
    },
    "forwardPorts": [6080, 5901],
    "portsAttributes": {
        "6080": {
            "label": "desktop"
        }
    },
```

### Starting the DevContainer

Then do `Ctr+Shift+P` and type `Rebuild and Reopen in Container`. This will launch the DevContainer.

{: .note }
Rebuilding the DevContainer is only needed if any changes are made to the files in the `./devContainer/` folder. After this is run, you may use `Reopen in Container` instead.

This may take a while, but once completed, the DevContainer should open with all your files in the "Explorer" tab on the left.

When opening GUI applications inside the DevContainer (Gazebo, rviz2, etc.), you will need to go to point a browser to `localhost:6080`, or use a VNC client of your choice, using the same URL.

{: .important }
If running on an ARM-based system, you **must** use new Gazebo. Gazebo Classic does not have an executable for ARM as far as I know.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
