---
title: Controllers
parent: ROS2 Control
nav_order: 2
---

## Introduction

You can find a list of already existing ROS2 Controllers on the [ros2_controllers](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html) page. The rest of this guide will concern making a custom controller.

All controller logic will be written in C++, including extensive use of header files. If you are not familiar with C++, I recommend either crying or going to [LEARN C++](https://www.learncpp.com/) and doing some tutorials.

Anywhere you see the word robot in these guides, you should probably replace it with the name of your root unless it is obvious that you shouldn't. Robot is just the placeholder we are using.

## Architecture of a Custom Controller

Firstly, you will need to make a CMake ROS2 package using the following command: `ros2 pkg create --built-type ament_cmake control`. Note that it HAS to be a CMake package; a Python package will not work. We write this in C++ and C++ is a compiled language, and CMake compiles while Python builds do not.

There is a lot of boilerplate that is needed for a custom controller. The guide on how to make one is vague and mostly verbal with no examples, but you can find it at [Writing a New Controller](https://control.ros.org/jazzy/doc/ros2_controllers/doc/writing_new_controller.html).

To explain the nonsense going on, I am going to write out a whole starter file and explain the boilerplate in sections underneath it.

### The Header File

```cpp
// File is named robot_controller.hpp
#ifndef ROBOT_CONTROLLER_HPP // 1.1
#define ROBOT_CONTROLLER_HPP

#include

namespace robot_controller
{
    class RobotController : public controller_interface::ControllerInterface // 1.2
    {
        public:
            RobotController() = default; // 1.3

            // 1.4
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;


            // 1.5
            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;
            
            controller_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            controller_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            controller_interface::return_type update(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
        

        // 1.6
        private:
            // Any necessary structs, helper functions, variables, etc
            // Realtime tools go here
            // ROS subs and pubs go here
            // Params from the URDF go here
            // Specifically, put std::string joint_names{"joint_name"} here for all joints
    };
} // namespace robot_controller

#endif // ROBOT_CONTROLLER_HPP
```

**1.1:** ROS2 Control requires the use of [header guards](https://www.learncpp.com/cpp-tutorial/header-guards/). Basically, C++ allows for only one definition of something, and if multiple files in the same project feature the same #includes then things are copied multiple times and errors get thrown from too many definitions of the same function/object. Header guards define a scope for includes to prevent copies. ROS2 Control's internal codebase uses many libraries that may or may not show up in controllers, so you have to use header guards. Header guards need a unique name, so convention is to just name it the file name (which should match the namespace and class name), including extension since hpp and cpp files will be named the same thing.

**1.2:** The specific name of all these things is very important. The namespace needs to match the name of the file, which should just be `<robot name>_controller`. The class name for the robot is pascal case but the same words. It NEEDS to inherit `controller_interface::ControllerInterface`. Notice how it's fairly common C++ to name namespace's in snake case and class names to be written in pascal case. A lot of this is just explained, so you follow conventions and make the code easier to understand.

**1.3:** This is a relatively new C++ feature that is basically just shorthand for creating a constructor that takes no parameters. [Default Constructors and Default Arguments](https://www.learncpp.com/cpp-tutorial/default-constructors-and-default-arguments/).

**1.4:** This is where you declare the functions that will configure the state and command interfaces of the system. This is just a function declaration, implementation will be in the cpp file. Note that controller_interface is a ROS2 import.

**1.5:** This is where the main functions of the file get declared. The init, configure, activate, deactive, and update functions will be where the bulk of the controller is written. Here, we just declare functions, override the default ones, and follow the template required for them as outlined in [Writing a New Controller](https://control.ros.org/jazzy/doc/ros2_controllers/doc/writing_new_controller.html).

**1.6:** This is where things can vary the most from the templates. Typically, the main thing that will go into the private section is state machine structs, helper functions, subs and pubs, parameters from the URDF, and any other local variables.

### Interface Configuration Functions

The source file for a controller is far too large to talk about in one section, so from now on **all of these subsections will be about one (or a few) functions inside robot_controller.cpp**.

```cpp
// File is named robot_controller.cpp

// Includes here

namespace robot_controller
{
    controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            left_wheel_joint_name_ + "/velocity",
            claw_joint_name_ + "/position"
        };

        return config;
    }

    controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names = {
            left_wheel_joint_name_ + "/position",
            left_wheel_joint_name_ + "/velocity",
            claw_joint_name_ + "/position"
        };

        return config;
    }
} // namespace robot_controller
```

Basically, this is just where you just claim the state and command interfaces declared in the [ROS2 Control URDF]({% link docs/Technical/ROS2/Jazzy/URDF/ROS2-Control-URDF.md %}). All of this is boiler plate except the names of the joints you are claiming, those will have to be changed.

**You should declare your joint names as variables in the header file**. This means if they change you can just change that one variable in the header file and not have to change it in the MANY places it will be used throughout the source file.

### The on_init() Function

This function runs when you first run your launch file and the robot needs to be initialized with the node parameters. It's main purpose is to parse variables set in the [ROS2 Control URDF]({% link docs/Technical/ROS2/Jazzy/URDF/ROS2-Control-URDF.md %}) and map them to local variables declared in the header file.

In the example below, we use auto_declare, which is part of the ros2 control framework and this function is allowed to use since it inherits controller_interface::ControllerInterface. It registers the parameters and passes them off to the nodes circling around in the ROS2 Control soup. Put the type in the `<>` followed by `("param", "default value")`.

Besides that, there is not much else going on. This all runs in a try/catch loop that'll catch errors and return an ERROR status, otherwise return a SUCCESS.

```cpp
// File is named robot_controller.cpp

namespace robot_controller
{
    controller_interface::CallbackReturn RobotController::on_init()
    {
        try
        {
            // This is where you would put other variables
            auto_declare<std::string>("left_wheel_joint_name", "left_wheel_Joint");
            auto_declare<std::string>("claw_joint_name", "claw_joint_name");
            auto_declare<double>("wheel_radius_m", 0.05);

            // These should ALWAYS be here for every bot
            auto_declare<std::string>("odom_topic", "/odom");
            auto_declare<std::string>("odom_frame_id", "odom");
            auto_declare<std::string>("base_frame_id", "base_link");
            auto_declare<bool>("publish_odom_tf", true);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }
} // namespace robot_controller
```

### The on_configure() Function

This function executes after initialization and is when the local variables, pub/subs, real-time nonsense, and everything is set up. It basically does all the configuration stuff that isn't setting node parameters.

This function looks really intimidating, and it's mostly because of how verbose C++ is. Basically all this is doing is what is said in the comments. The long types are just the boilerplate for ROS2 Control. This function can vary a lot depending on what you're trying to implement, so I added some comments with some optional stuff you could add.

```cpp
// File is named robot_controller.cpp

namespace robot_controller
{
    controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
    {
        // Set local variables to the node parameter values
        left_wheel_joint_name_ = get_node()->get_parameter("left_wheel_joint_name").as_string();
        claw_joint_name_ = get_node()->get_parameter("claw_joint_name").as_string();
        wheel_radius_m_ = get_node()->get_parameter("wheel_radius_m").as_double();

        // Remember these should always be here
        odom_topic_ = get_node()->get_parameter("odom_topic").as_string();
        odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
        base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
        publish_odom_tf_ = get_node()->get_parameter("publish_odom_tf").as_bool();



        // Initialize the realtime buffers if you have any



        // The pub/subs look really ugly but this is just how you declare them it's really messy
        // ROS subscriptions
        cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Twist & msg) { cmdVelCb(msg); });
        
        // ROS publishers
        auto odom_pub = get_node()->create_publisher<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SystemDefaultsQoS());

        odom_pub_rt_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
            odom_pub);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());



        // Do other things like set modes or fault latched state



        RCLCPP_INFO(get_node()->get_logger(), "Configured RobotController.");
        
        return controller_interface::CallbackReturn::SUCCESS;
    }
} // namespace robot_controller
```

### The on_activate() and on_deactivate() Function

This one is really easy. Just make sure that the interfaces have been claimed correctly. Check that all of them are accounted for and error out if they aren't. Sometimes interfaces don't get claimed and we want to error that so we notice.

```cpp
// File is named robot_controller.cpp

namespace robot_controller
{
    controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
    {
        // Make sure that the command interfaces are present

        // Make sure that the state interfaces are present

        // Safety outputs

        RCLCPP_INFO(get_node()->get_logger(), "Activated RobotController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }
} // namespace robot_controller
```

The on_deactivate() function is even easier, just make sure all outputs are set to 0. The robot is not moving.

```cpp
// File is named robot_controller.cpp

namespace robot_controller
{
    controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Stop any sort of command outputs
        // If you're using a state machine, set mode to IDLE

        RCLCPP_INFO(get_node()->get_logger(), "Deactivated RobotController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }
} // namespace robot_controller
```

### The update() Function

I am not actually going to put a filled in update function here. This is where ALL of the actual runtime logic will go, so it would be way too long to include here.

```cpp
// File is named robot_controller.cpp

namespace robot_controller
{
    controller_interface::return_type RobotController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Stuff goes here
        // Generally, first read states, second calculate kinematics, third write commands
    }
} // namespace robot_controller
```

The update function needs a lot of thought put into it. Keep in mind that this function can be run hundreds of times per second as it is the main control loop of the program. If it is too slow or causes any sort of hangup, the entire control for the robot can start falling apart. It needs to be real-time compliant, which means everything follows a strict schedule and has to take a determinable amount of time.

Here are some quick do's and don'ts for the update function:

- **DON'T allocate memory**. This is slow and takes an unpredictable amount of time. Use the header file to declare any needed variables. Never use the new keyword.
- **DON'T resize containers**. This shouldn't come up that much since it's C++, but if you are presented with the opportunity to resize any sort of container, do not do it.
- **DON'T create or modify strings**. Creating or modifying strings is just memory allocation under the hood, avoid it.
- **DON'T use standard logging**. Writing to the console or by using RCLCPP_INFO both block the thread until it completes, throttling the program. Logging is acceptable in error paths, but not the hot path. If you use logging, use RCLCPP_WARN_THROTTLE or RCLCPP_INFO_THROTTLE
- **DON'T use any sort of waits or pauses**. For obvious reasons.
- **DON'T use standard pub/subs/broadcasters/etc**. Standard ROS2 pub/subs use thread locks and allocate memory. When you publish odom or transforms or anything, use `realtime_tools::RealtimePublisher` or broadcaster
- **DO use realtime_tools**. This will help make sure everything runs on a strict control schedule.
- **DO check for NaNs and invalid data**. It happens so often that some mild misconfiguration somewhere else in the code will generate NaN values and crash the entire controller. Add safeguards.
- **DO keep the math as efficient as possible**. There is some complicated kinematics going on, keep it as optimal as possible (most of these have solved best equations available to find on the internet)
- **DO split the update() function into helper functions**. You are allowed to use helper functions to make the update() function less of a mess, and this is encouraged. Remember that any function update calls is still part of update, so any helper functions must comply with these constraints as well.
- **DO read states first, calculate second, and write commands third**. That should be the basic structure of your update function

### Class Registration Macro

You NEED to put this at the end of the controller file. If you do not, the ROS2 framework will not know this controller even exists. Make sure to replace robot with your bot name. You have to handle state machines, reading state interfaces, writing command interfaces, doing all of the behavior logic, etc. Just go look at some code from actual bots.

```cpp
// Note that somewhere in the includes you have to include the actual plugin export function
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(robot_controller::RobotController,
                       controller_interface::ControllerInterface)
```

This goes outside of the robot controller namespace.

### Final Controller Source File Format

Putting all of that together, you get the following starter code. You can also add whatever helper functions you need anywhere you think works, these are just the REQUIRED functions. Remember to alter the header file as needed.

```cpp
// File is named robot_controller.cpp

// Includes here

namespace robot_controller
{
    controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            // claim joints with a command interface
        };

        return config;
    }


    controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names = {
            // claim joints with a state interface
        };

        return config;
    }


    controller_interface::CallbackReturn RobotController::on_init()
    {
        try
        {
            // Pass params to nodes
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
    {
        // Set local variables to the node parameter values

        // Initialize the realtime buffers if you have any

        // ROS subscriptions
        
        // ROS publishers

        // Do other things like set modes or fault latched state

        RCLCPP_INFO(get_node()->get_logger(), "Configured RobotController.");
        
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
    {
        // Make sure that the command interfaces are present

        // Make sure that the state interfaces are present

        // Safety outputs

        RCLCPP_INFO(get_node()->get_logger(), "Activated RobotController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }
   
   
    controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Stop any sort of command outputs
        // If you're using a state machine, set mode to IDLE

        RCLCPP_INFO(get_node()->get_logger(), "Deactivated RobotController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RobotController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Stuff goes here
        // Generally, first read states, second calculate kinematics, third write commands
    }

} // namespace robot_controller


PLUGINLIB_EXPORT_CLASS(robot_controller::RobotController,
                       controller_interface::ControllerInterface)
```

## Exporting the Controller

There are three things you need in order to export a controller, so the controller manager will be able to find it: the plugin descriptor file, the CMake export function, and the class registration macro. If you don't have all three of these things done, the controller manager won't be able to launch your controller.

The **plugin descriptor file** is a very simple XML file that describes what part of the cpp code is the actual controller. It will probably live at `control/config/robot_controller.xml`, but you can put it wherever you want as long as the control package will be able to find it during build, so it is just easier to put it inside the control package. It is basically all boilerplate with very little you have to change. The below code block is the entire XML file.

```xml
<library path="robot_controller">
  <class
    name="robot_controller/RobotController"
    type="robot_controller::RobotController"
    base_class_type="controller_interface::ControllerInterface">
    <description>
      Robot supervisor controller with mode gating (IDLE/DRIVE/DIG/DUMP/FAULT).
    </description>
  </class>
</library>
```

The things you need to change include:

- The library path. This path is kind of confusing because it's not referencing the file path, it's referencing the path once colcon build is run and everything's in the install directory. Declaring the library path as robot_controller means it assumes that the controller file will be installed into the robot_controller directory using something similar to this code `add_library(robot_controller SHARED src/robot_controller.cpp)`
- The name and type should both just match namespace / or :: ClassName.
- The base class type should ALWAYS be controller interface.
- The description is whatever.

The **CMake export function** will go in the `CMakeLists.txt` file. Somewhere in the CMake file, you have to export the plugin descriptor file using this command: `pluginlib_export_plugin_description_file(controller_interface config/robot_controller.xml)`. This command does use the file path and assumes that you put the XML in control/config/.

The **class registration macro** is something we have already talked about. It goes at the very end of the `robot_controller.cpp` file and registers the class and allows ROS2 Control to do all the background work necessary to load it as a plugin.

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
