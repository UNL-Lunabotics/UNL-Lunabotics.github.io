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

## Interface Configuration Functions

The source file for a controller is far too large to talk about in one section, so from now on **all of these subsections will be about one (or a few) functions inside robot_controller.cpp**.

```cpp
// File is named robot_controller.cpp
#ifndef ROBOT_CONTROLLER_CPP
#define ROBOT_CONTROLLER_CPP

#include

namespace robot_controller
{
    controller_interface::InterfaceConfiguration TerrenceController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            left_joint_name_ + "/velocity",
            right_joint_name_ + "/velocity",
            loader_joint_name_ + "/position",
            hopper_joint_name_ + "/position"
        };

        return config;
    }

    controller_interface::InterfaceConfiguration TerrenceController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names = {
            left_joint_name_ + "/position",
            left_joint_name_ + "/velocity",
            right_joint_name_ + "/position",
            right_joint_name_ + "/velocity",
            loader_joint_name_ + "/position",
            hopper_joint_name_ + "/position"
        };

        return config;
    }
} // namespace robot_controller

#endif // ROBOT_CONTROLLER_CPP
```

### The on_init() Function

```cpp
// File is named robot_controller.cpp
#ifndef ROBOT_CONTROLLER_CPP
#define ROBOT_CONTROLLER_CPP

#include

namespace robot_controller
{
    controller_interface::CallbackReturn TerrenceController::on_init()
    {
        try
        {
            auto_declare<std::string>("left_joint_name", "DS_Joint");
            auto_declare<double>("wheel_radius_m", 0.05);

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

#endif // ROBOT_CONTROLLER_CPP
```

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
