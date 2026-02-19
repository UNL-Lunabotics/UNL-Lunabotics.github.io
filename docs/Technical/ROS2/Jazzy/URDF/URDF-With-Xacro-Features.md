---
title: URDF with Xacro Features
parent: URDF
nav_order: 3
---

## Tags

XML works kind of like HTML so it has a variety of different tags that you can open with `<tag>` and close with `</tag>` or do it all in one line with `<tag/>`. The specific tags you need are highly context dependent, so please refer to the above examples to see some. In general, unless it's an int, and even sometimes when it is an int, everything should be in quotes when giving something a value. That is basically it, there are elements and values and that is 90% of a URDF.

## Property, Property Blocks, and Args

Xacro properties are equivalent to regular variables in any other programming language. A property block is something that has more than one value, so more similar to an object in an object orientated programming language. After you create a property or property block, you use `"${property_name}"` to evaluate it.

An example of using properties is shown here (taken from the xacro wiki):

```XML
<xacro:property name="the_radius" value="2.1" />
<xacro:property name="the_length" value="4.5" />

<geometry type="cylinder" radius="${the_radius}" length="${the_length}" />
```

An example of using a property block is shown here (taken from the xacro wiki):

```XML
<xacro:property name="front_left_origin">
  <origin xyz="0.3 0 0" rpy="0 0 0" />
</xacro:property>

<wheel name="front_left_wheel">
  <xacro:insert_block name="front_left_origin" />
</wheel>
```

The above example actually utilizes a macro, which is expanded upon later in [Macros](#macros)

## Math Expressions

When using xacro and inside a tag, you can do `"${math equation}"` in order to do evaluate that value. This could be something like `"${pi/2}"`. This is extremely helpful when calculating a LOT of things, especially rotations and inertia.

Xacro (in recent veresions) specifically uses the [Python math module](https://docs.python.org/3/library/math.html) in order to compute this, so anything available there is available inside a `"${math equation}"` expression. This includes a lot of functions and constants like pi, e, inf, and nan.

Math operators like *, /, +, and - are available.

## Conditional Blocks

Xacro allows two conditional blocks: if and unless. If is a standard if conditional and unless executes if the given value is NOT true. These expresions accept values of 1, 0, true, or false.

```XML
<xacro:if value="expression or var">
  <... some xml code here ...>
</xacro:if>
<xacro:unless value="expression or var">
  <... some xml code here ...>
</xacro:unless>
```

This is particularly useful when dealing with Gazebo. Typically, as shown in some previous xacro examples, the Gazebo file is only included and loaded if a value like use_sim is true.

## Rospack commands

Xacro supports all of the rospack commands that you use in [roslaunch](https://wiki.ros.org/roslaunch/XML#substitution_args). Particularly, substitution arguments are incredibly important. Substitution arguments are variables that can be set by the launch files. This is where we get values like use_sim or use_control.

You can declare them like this `<xacro:arg name="myvar" default="false"/>` and evaluate them at any time with `"$(arg thingsname)"`.

Typically, they are declared in the main `urdf.xacro` file and can be used in any other file that the main file includes.

Additionally, one of the main uses for these rospack commands is to find meshes. You can use the find keyword in this context and then put a ros2 package file path. For example, if I have the ros2 package description and it has a folder named meshes/ with meshes inside of it, I could use the below argument to give the chassis its mesh. You have to preface this all with file:// which is the modern solution in Jazzy (older versions are different).

```XML
<mesh filename="file://$(find rover_description)/meshes/chassis.STL"/>
```

A full practical example of this is shown below:

```XML
<collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
        <mesh filename="file://$(find rover_description)/meshes/chassis.STL" />
    </geometry>
</collision>
```

There are other commands which you can view at the hyperlink at the start of this section, but these are the most common and most useful ones.

## Macros

One of the most powerful things you can do in xacro is use macros, but it is also highly unlikely you will actually end up using them. This is because most URDFs will be exported without them, and you would have to go in and manually edit things a LOT in order to utilize them to the fullest.

It is easiest to conceptualize macros as just a way to declare objects in xacro. They can do other things, but one of the main uses is to be able to reuse a lot of code. You can declare a macro named 'wheel' with a lot of different parameters, and later in the URDF you can use this macro, give it the parameters it needs, and not have to rewrite everything but slightly different for every wheel.

You can write out the code and only accept value params, or you can accept whole blocks of XML as a param.

Here is an example using just value params:

```XML
<!-- Create a simple box xacro object with params width height depth -->
<xacro:macro name="box" params="width height depth">
  <link name="generated_link">
    <visual>
      <geometry>
        <box size="${width} ${height} ${depth}"/>
      </geometry>
    </visual>
  </link>
</xacro:macro>

<!-- Create a box using this macro -->
<xacro:box width="2" height="1" depth="0.5"/>
```

An example of using whole XML block params is given below:

```XML
<!-- Create the macro (aka object) with params origin, content, and anothercontent -->
<xacro:macro name="pr2_caster" params="suffix *origin **content **anothercontent">
  <joint name="caster_${suffix}_joint">
    <axis xyz="0 0 1" />
  </joint>
  <link name="caster_${suffix}">
    <xacro:insert_block name="origin" />
    <xacro:insert_block name="content" />
    <xacro:insert_block name="anothercontent" />
  </link>
</xacro:macro>

<xacro:pr2_caster suffix="front_left">
  <pose xyz="0 1 0" rpy="0 0 0" />
  <container>
    <color name="yellow"/>
    <mass>0.1</mass>
  </container>
  <another>
    <inertial>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
    </inertial>
  </another>
</xacro:pr2_caster>
```

There is more to macros but it is unlikely you will ever have to use them heavily, so don't worry about it. If you do need to do more research, you can start at the [ROS Xacro Deep Wiki](https://deepwiki.com/ros/xacro/5.1-macros-and-parameters).

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
