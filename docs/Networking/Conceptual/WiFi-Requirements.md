---
title: WiFi Requirements
parent: Systems Engineering
nav_order: 1
---

## WiFi Requirements

These are specifically the requirements to be in compliance with NASA rule specifications (as of 2025-2026 year. **Edit this document as years go.**).

This document was written when the team was using the [tp-link Archer AX80](https://www.tp-link.com/us/home-networking/wifi-router/archer-ax80/) router.

Official requirements:

1. The router must support both 2.4G and 5G bands.
2. The router must have the ability to turn off the 2.4 band (will be proven in Proof of Life).
3. Bluetooth transmission of equipment in the 2.4G band is allowed for sensors and other robot communications.
4. Robots are REQUIRED to have external WiFi antenna's.
5. The use of 2.4G Zigbee/IEEE 802.15.4 technology is prohibited.
6. The SSID name of the bands should be `Team_##` for the 2.4G bands and `Team_##_5G` for the 5G bands.
7. The SSID has to be broadcasting. Ensure any 'Hide SSID' options are UNCHECKED.
8. The WiFi bands have to be encrypted (means unspecified).
9. On the 2.4G band, the Channel Width has to be in the 20MHz range due to the no channel bonding rule.
10. The 2.4G band has to be on Channel 1 during competition runs and inspection.
11. The robot CAN NOT be on the 2.4G band in the RoboPit (unless special permission from the Pit Boss is gained. Just use the 5G band for testing).
12. Any wireless back channels are completely banned. No bridging connections from the robot to the router via secondary wireless devices.

Unofficial requirements just because of the router we use:

1. TURN OFF SMART CONNECT. It swaps between 2.4G and 5G to find the best connection which will kill ROS2 and also probably NASA wouldn't like it

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
