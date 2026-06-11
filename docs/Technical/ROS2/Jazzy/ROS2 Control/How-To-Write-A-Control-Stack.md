---
title: How to Write a Control Stack
parent: Jazzy
nav_order: 1
---

## How to Write a Control Stack

This page is meant to be your one stop for everything you need to code a custom ROS2 (Jazzy) control stack from scratch. It will include a full step by step process of what you should do, mostly by linking to other pages already written.

Things this documentation **WILL** cover:

- Setting up the ROS2 Control package
- Setting up the `ros2_control.xacro` URDF file
- Writing a controller (custom and built in)
- Writing a hardware component
- Connecting the hardware component to a microcontroller
- Writing the launch files for everything
- Setting up gamepad input
- Writing a proper `CMakeLists.txt` and `package.xml` for the control package
- Setting up CAN communication
- Setting up custom ROS2 services for gamepad input
- Setting up mock hardware components for testing
- Telemetry scaffolding
- Encoder best practices (software and hardware)
- C++ syntax that confuses new people so people can read the code better (not necessarily write it)

Things this documentation will **NOT** cover:

- Writing any part of the URDF that isn't for ROS2 Control
- Writing any part of the URDF that is meant for simulation
- Setting up a repository or any of the rest of the ROS2 packages besides Control
- Installing software
- Setting up Docker
- Controlling the robot in simulation
- Setting up any sort of robot visualization software like RViz2 or Foxglove
- Setting up any physical controller that isn't a gamepad (no keyboard or anything like that)
- Literally anything about autonomy
- How to write in C++
- How to program in general
- Basic ROS2 concepts

This is not to say that there does not exist ANY documentation on the above items, just that this guide will not cover it. Before starting this process, you should already have the rest of your ROS2 packages set up for development, including but not limited to description and bringup. You are expected to already have things like Docker or whatever development environment you are using. You are expected to have all required software already installed (it should all be in the Dockerfile). You are expected to have a basic knowledge of how ROS2 works and how to program in general. You are expected to already understand concepts like URDF.

{: .warning}
This is a highly technical guide, not a conceptual one. If you do not understand a concept written above, this documentation will in all likelihood will not explain it. You should consult the Curriculum section to learn more.

1. Create the package
2. Create the state interfaces in the URDF
3. Create the controller header file with boilerplate
4. Create the controller source file with boilerplate
5. Create the config/ directory
6. In config/ add the robot_controller.xml and the robot_controller.yaml and joystick.yaml
7. Update the CMakeLists.txt and package.xml
8. build the project, it should already compile with just the boilerplate in there. It might have some unused variable warnings but still should compile
9. If you haven't already, it's time to do ConOps. Design the state machine flow. What does the robot do when? This should be compatible with teleop and autonomy. Keep in mind a manual mode for testing or recovery mid match
10. If you haven't already, create the interfaces package and define a service for state changes
11. In the control cpp package, create a new source file for the state_manager_client or whatever you want to call it. Write it to sub to joy, create a client for the set_state or change_state service, and write the whole thing and such
12. colcon build again, it should all compile
13. Fill out the controller header file specifics
14. Fill in the easy controller functions in the controller source file (everything besides update). Build again, it should still compile
15. Fill out the update function. Build again, it should still compile
16. Create the launch file (wired assume and local network)
17. Do extensive testing of the controller with a gamepad plugged in and echoing cmd_vel, once satisfied, move on
18. Create the hwc header file with boilerplate
19. Create the hwc source file with boilerplate
20. In config/ add the hwc xml and link it in the URDF
21. Update the CMakeLists.txt and package.xml
22. Build the project, it should compile correctly and still run the controller fine
23. Add in the prebuilt header files for can_comms (if needed) and microcontroller comms (if needed)
24. Build the project and all that
25. Fill out the easy hwc functions (everything besides read/write)
26. Fill out read/write hwc functions
27. Build and test everything with a microcontroller and/or can
28. That's it :)

### 1. Creating the Control Package

Read the documentation on [ROS2 Packages]({% link docs/Technical/ROS2/Jazzy/Packages/Creating-Packages.md %}) and create an ament_cmake package named control.

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
