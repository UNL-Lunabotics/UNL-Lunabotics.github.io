---
title: MuJoCo Scene File
parent: Files
nav_order: 5
---

## MuJoCo Scene File

MuJoCo scene files, known as an MJCF, defines which objects are placed in the world when the simulation is running. It is very configurable and customizable, and similar to a URDF.

### Basics

Each MJCF will be a little different, depending on how it is set up, but every MJCF will start with the following:

```xml
<mujoco model="model_name">
  <!-- Content here -->
</mujoco>
```

Then, you can optionally define any `<compiler>`, `<visual>`, `<default>`, or `<option>` tags if needed. These would specify global values such as gravity, global friction, etc.:

```xml
<mujoco model="example">
  <compiler angle="radian" />
  <option timestep="0.002" gravity="0 0 -9.81" />
  <default>
    <geom friction="0.01 0.01 0.01" condim="4" />
    <joint damping="0.1" frictionloss="0.01" />
  </default>
</mujoco>
```

Here, we specify to use radians as our angles, then define friction, gravity, and the timestep.

Then, to add objects into the world, add a `<worldbody>` tag, then `<body>` tags, like the following:

```xml
<mujoco model="example">
  <worldbody>
    <body name="ground_plane" pos="0 0 0">
      <inertial mass="1" pos="0 0 0" diaginertia="1 1 1" />
      <geom type="plane" size="100 100 0.1" friction="0.01 0.01 0.01" />
    </body>
  </worldbody>
</mujoco>
```

Here a `<body>` tag is set with the name of "ground_plane", positioned at the world origin. Each `<body>` can have a `<geom>` and `<intertial>` tag, defining the type of object it is, along with its visual and collision. In the example, we set the object type to a plane, with a mass of 1, size of 100 on the x and y, and set some friction. Note that multiple `<body>` tags can be specified.

A complete simple example is as follows:

```xml
<mujoco model="example">
  <worldbody>
    <body name="ground_plane" pos="0 0 0">
      <inertial mass="1" pos="0 0 0" diaginertia="1 1 1" />
      <geom type="plane" size="100 100 0.1" friction="1.0 0.8 0.01" />
    </body>
    <body name="box" pos="2.8417219933466216 2.3179857352986479 0.49999999990199806">
      <inertial mass="1" pos="0 0 0" diaginertia="0.16666 0.16666 0.16666" />
      <geom type="box" size="0.5 0.5 0.5" friction="1.0 0.8 0.01" />
    </body>
    <body name="sphere" pos="1.7558753273133232 -0.14177392862852622 0.499999999902001">
      <inertial mass="1" pos="0 0 0" diaginertia="0.1 0.1 0.1" />
      <geom type="sphere" size="0.5" friction="1.0 0.8 0.01" />
    </body>
    <body name="cone" pos="-0.45892397368634047 -1.1120748308789254 0.49999999993296756">
      <inertial mass="1" pos="0 0 0" diaginertia="0.075 0.075 0.075" />
      <geom type="cylinder" size="0.5 0.5" friction="1.0 0.8 0.01" />
    </body>
    <body name="cylinder" pos="-3.4181480821354935 0.11779292196676421 0.49999942638036005">
      <inertial mass="1" pos="0 0 0" diaginertia="0.14580 0.14580 0.125" />
      <geom type="cylinder" size="0.5 0.5" friction="1.0 0.8 0.01" />
    </body>
  </worldbody>
</mujoco>
```

Here, a plane, box, sphere, cone, and cylinder are defined with mass, size, friction and position values.

### Including other MJCF

For more complicated worlds, it may be preferable to split up the MJCF into different files. To do this, define another MJCF, then in the main MJCF, add the following:

```xml
<mujoco model="example">
  <include file="/path/to/other.mjcf" />
</mujoco>
```

Then, MuJoCo will reference the `other.mjcf`, found in the path, when launching the simulation. The file path can be relative or absolute.

### Defining Meshes and Materials

Instead of predefined simple objects or colors, you can include meshes and materials to get custom shapes and colors. MuJoCo works best with `.obj` files for meshes, and `.mtl` files for materials, though others may work. Note that `.stl` files are not supported.

{: .note}
While MuJoCo does STL files, there seems to be some compatibility issues with `mujoco_ros2_control`, and it may error out when launching. It is preferable to use `.obj` files instead.

To add a mesh to an MJCF, first make the mesh available to be used:

```xml
<mujoco model="example">
  <asset>
    <material name="MyCustomMaterial" specular="0.5" shininess="0.5" rgba="1 1 1 1.0" />

    <mesh file="my_custom_mesh.obj" />
    <mesh file="/path/to/another_custom_mesh.obj" />
  </asset>
</mujoco>
```

Here we set an `<asset>` tag to define any custom meshes or materials. For materials, specify the name, then the material properties, such as color. For meshes, set the file path, relative or absolute.

Then, a `<body>` tag can reference these later:

```xml
<mujoco>
  <worldbody>
    <body name="my_body" pos="3 1 1.5">
      <geom material="MyCustomMaterial" mesh="my_custom_mesh" class="visual" />
      <geom mesh="another_custom_mesh" rgba="0.3 0.0 0.3 1" class="collision" />
    </body>
  </worldbody>
</mujoco>
```

Here, we set a material and mesh geometry. The material references a mesh to use and will be used by the entire `<body>` tag. The mesh defines an individual mesh to use as part of the `<body>`. Note that multiple mesh geometry can be defined.

There is a lot of other configuration that can be specified. To learn more, visit the [MuJoCo XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html) documentation.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
