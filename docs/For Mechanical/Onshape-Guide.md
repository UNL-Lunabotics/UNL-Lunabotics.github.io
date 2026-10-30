---
title: Onshape Guide
parent: For Mechanical
nav_order: 1
---

# Onshape Guide

The goal is to turn Onshape part + mate connections into the URDF link/joint structure. If you need a reminder on URDF links/joints, it is below:

![URDF Link and Joint Explanation]({% link attachments/URDF-Link-Joint-Explanation.png %})

Below is a non-exhaustive list about requirements for programming for Onshape:

- The exporter works only on top-level assemblies.
- Animate mate connectors to ensure correct functionality
- Have reasonable and descriptive part names. The exporter can ignore certain parts based on the name. This is helpful for parts that don’t need to be modeled in the exporter (e.g. screws, inner e-box, etc.)
- Planar mates will be skipped. If a mate needs movement, give it another mate connection in addition to the planar.

  {: .note}
  Planar mates do not break the exporter. This is just a consideration for important mate connectors for movement.
- Mate connectors that have movement will need a special prefix to the name.

  {: .note}
  Mates who do not have the prefix will be skipped, which may be okay for certain mates (e.g. a Fasten mate just to align axes, etc.)
  - Fasten mates must be named `fix_NAME`. The exporter creates “fixed” joint.
  - Slider mates must be named `dof_NAME` for the exporter. The exporter creates “prismatic” joint.
  - Revolute and Cylindrical mates must be named `dof_NAME` for the exporter. The exporter creates “revolute” joint.
- Gear relations are able to be exported. Be sure to click the source mate first, then the target mate.
- The exporter is sometimes finicky, especially when it comes to mate connectors. Small changes to the mate connector may need to happen.

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
