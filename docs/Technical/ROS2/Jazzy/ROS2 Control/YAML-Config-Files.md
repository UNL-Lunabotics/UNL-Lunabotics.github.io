---
title: YAML Config Files
parent: ROS2 Control
nav_order: 4
---

## YAML Config Files

There are generally two types of config files that live in the controls section of the code, and that's the controller YAML and the joystick YAML.

### Controller YAML

This is the config file you will give to the controller manager in ROS2 Control that tells it what controller code to run. Additionally, you can define parameters in this YAML for use elsewhere in the controller (for example, wheel radius).

Here is a sample file (replace the word robot with the name of your robot):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50 # This is in Hz and is the speed your entire program will run at

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster   # this can be left unchanged

    robot_controller:
      type: "robot_controller/RobotController"

robot_controller:
  ros__parameters:
    # Literally any configurations you want to set
    wheel_radius_m: 0.929
```

{: .important}
The type you specify under robot_controller is the same thing exported in the plugin descriptor file usually titled `robot_controller.xml`. More details are at the very end of [Custom-Controllers.md]({% link docs/Technical/ROS2/Jazzy/ROS2 Control/Custom-Controllers.md %}).

While ROS2 Control is technically unit-less (you can read more on the page [Units](({% link docs/Technical/ROS2/Jazzy/ROS2 Control/Units.md %}))), there are some conventions to follow. Any measurement of length should be in meters, any measurement of speed should be in meters/second, and any measurement of revolution should be in radians/second.

{: .important}
The ros parameters you set in this YAML will be read in the on_init() function in `robot_controller.cpp`.

### Joystick YAML

ROS2 comes with built-in capabilities to handle all gamepad input behind the scenes with a node called joy (short for joystick). This team uses a controller that is similar to an XBox gamepad. In order for all this behind the scenes stuff to work, you have to configure what axes you actually want to use. This is specifically for joystick input, which is why this YAML is frequently called `joystick.yaml`.

```yaml
teleop_twist_joy_node:
  ros__parameters:
    axis_linear:
      x: 1.0
    axis_angular:
      yaw: 3.0
    scale_linear:
      x: 1.0
    scale_angular:
      yaw: 1.0

    deadzone: 0.5

    enable_button: 4
    enable_turbo_button: 5  # Optional, don't really recommend using it
    
    require_enable_button: false    # Require a button to be pressed to enable any other button/axes input to be processed
```

{: .important}
All of the above setting names are very specific, do not change them. The only thing you can change is the numerical values.

All of the above numerical values are indexes in an array. Under the hood, the drivers for gamepad's send all the data as two arrays, one for buttons and one for axes. You select which axes you want to use for joystick control of the robot in this file, or what buttons you may want to use for enable or turbo (if you implement these optional features).

Unfortunately, every controller has a different array scheme. To find yours, plug in the controller and run the command `ros2 run joy joy_node` and it'll print the output to the console.

Later, when creating the launch files for the program, this is the file that you will give to the process that'll launch the joy features.

{: .note}
This file could easily be moved elsewhere in the program than with the rest of the ROS2 Control code. It is not a ROS2 Control component and the controller doesn't even read this information. However, since it has something to do with the idea of controlling the robot, it's put here for thematic cohesion more so than technical cohesion.

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
