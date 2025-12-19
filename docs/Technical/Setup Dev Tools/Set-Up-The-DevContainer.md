---
title: Setting up the DevContainer
parent: Development Tools Setup
nav_order: 4
---

# Setting up the DevContainer

DevContainers will use Docker to create the virtual environment mentioned before.

## Cloning the Repository

If comfortable with `git` and the terminal(recommended), sign into GitHub, type `git clone  https://github.com/UNL-Lunabotics/terrence_2.0`

Otherwise, open VSCode, and click on "Clone Git Repository". Sign in with GitHub, and clone the `UNL-Lunabotics/terrence_2.0` repo.

## Starting the DevContainer

Once the repository is cloned, open VSCode and point it to the repo.

> Note: If you are on Windows or macOS, follow [this](#notes-for-windows-and-macos-users) before continuing.

Then, do `Ctr+Shift+P` (or `Cmd+Shift+P` on macOS), and type `Rebuild and reopen in Container`, with will launch the DevContainer.

{: .note }
Rebuilding the DevContainer is only needed if any changes are made to the files in the `./devContainer/` folder. After this is run, you may use `Reopen in Container` instead.

The DevContainer should open successfully, with all your files in the "Explorer" tab on the left.

Once the DevContainer launches, you are done setting up the dev tools!

{: .important }
If running on an ARM-based system, you **must** use new Gazebo. Gazebo Classic does not have an executable for ARM as far as I know.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)

---

### Notes for Windows and macOS users

1. Make sure Docker Desktop is running **before** launching the DevContainer.

2. Add the following lines of code to `.devContainer/devcontainer.json`, after line `29`.

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

When opening GUI applications (Gazebo, rviz2, etc.), you will need to go to point a browser to `localhost:6080`, or use a VNC client of your choice, using the same URL.

Continue to [Starting the DevContainer](#starting-the-devcontainer)