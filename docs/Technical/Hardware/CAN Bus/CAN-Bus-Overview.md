---
title: CAN Bus Overview
parent: CAN Bus
nav_order: 1
---

## CAN Bus Overview ##

For our 2025-2026 rover, Tibble, we use CAN bus to communicate with our two drivetrain motors (Kraken X60s). The Kraken X60s have a built in motor controller, and can be sent commands over CAN bus using the Phoenix 6 API to send those signals to a CAN adapter, which then sends the CAN signal over the bus to the motors. We place two motors on our CAN bus, and then terminate it (with a 120 Ohm resistor) back inside our E-Box. The starting termination resistor is built into the USB-to-CAN adapter boards we use. 

So far, we only use our single CAN bus to communicate with our two drivetrain motors. However, it is possible to place more devices on this bus to communicate with each of them simultaneously. 

The other guides in this folder will outline the more technical setup of how to wire, install software for, and set up our CAN communications.
