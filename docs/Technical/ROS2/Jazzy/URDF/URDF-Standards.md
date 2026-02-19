---
title: URDF Standards
parent: URDF
nav_order: 1
---

## Conventions

1. Declare joints before links
2. When declaring a joint/link pair, name them the same thing but append a `_joint` or `_link` suffix
   1. `example_joint` and `example_link`
3. Put one new line between a joint/link pair
4. Put two new lines between two different joint/link pairs
5. Put three new lines between any two different sections

## Standards

[Official Robotics Standards](https://reps.openrobotics.org/)

Most important to note:

1. The parent link to all of the actual robot, rigidly fixed to the mobile platform base, is `base_link`
2. The world fixed coordinate frame for odometry is `odom`
3. ROS uses a right hand coordinate system where X forward, Y left, and Z up.

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
