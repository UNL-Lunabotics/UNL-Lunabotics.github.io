---
title: Gazebo in URDF
parent: URDF
nav_order: 1
---

## Introducing the Gazebo Headache

Basically, Gazebo has really bad documentation and the community documentation is worse. **We use Gazebo Harmonic, not Gazebo Classic** and almost every guide you find on the internet will be for Gazebo Classic. Furthermore, Gazebo (version Harmonic) has a lot of documentation for migrations but not a whole lot for writing from scratch. The versioning is very confusing because people say Gazebo to refer to modern Gazebo and also say Gazebo to refer to Gazebo Classic.

Additionally, Gazebo is weirdly determined to separate itself from ROS2, but the main reason to use the program is because it has good ROS2 integration. Because of this, they have very little documentation on integrating with ROS2. This headache is compounded by the fact that most of Gazebo's docs are written for SDF, which is just their slightly different version of URDF, so all of it needs to be translated to URDF. What they want you to do is write everything in SDF with a little ROS2 integration, but we mostly care about ROS2 features, so we want to write everything in URDF with a little Gazebo integration and that is upsetting for Gazebo.

What's worse is the usual go-to method of just look at Nav2's documentation isn't going to work, because Nav2 only has Gazebo Classic documentation. This is basically going to be one of your only sources for reliable information. I highly don't recommend asking generative AI for help as because it is only trained on the resources available on the web, it also doesn't know anything about Gazebo Harmonic.

It is even difficult to figure out what GitHub repositories you can even look at for this specific version of Gazebo because everything's split between 10 different repositories with different versioning schemes. You can view the full feature list at <https://gazebosim.org/docs/harmonic/release-features/#harmonic>.

## Gazebo Link/Joint References

This is the easy part. Let's look at the example given in the URDF with Xacro page.

```XML
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Add Gazebo references to links/joints including color and friction values -->

    <!-- Define Gazebo info for any sensors -->

</robot>
```

The first part, adding Gazebo references, is really easy. There is basically only two things you can do there, give a specific link a color and various physics properties.

## Gazebo Sensors

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
