---
title: CAN Tuner X Kraken X60 Motor Setup
parent: Technical
grand_parent: Embedded Systems
nav_order: 3
---

## Tuner X Kraken X60 Motor Setup ##

We use the Kraken X60 brushless motors for our drive train. In order to test these motors and install new firmware updates, the [Tuner X](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html) software is required.

### Initial Thoughts and Shortcomings ###

Since Tuner X only has a GUI on Windows, this process will assume you have a computer capable of running Windows 11. It is possible to run Tuner X on Linux, but that is something our team has not yet figured out how to effectively do. Tuner X also uses SSH to run commands from the computer that Tuner X is installed on to the robot controller PC. Unfortunately, it is kind of a pain in the ass to get this all set up, especially if you are on a corporate or university WiFi network with restrictions placed on SSH (hint hint: use a phone hotspot for this). Because of this, the method described below requires both a Windows computer and your separate robot controller computer.

### Prerequisites ###

For this, you need a PC or laptop with Windows 11 (I'm so sorry), and your robot controller PC.

You must have already set up the [CAN Adapter Board]({% link docs/Technical/Hardware/CAN Bus/CAN-Adapter-Board-Setup.md %}) and must have already run the CAN adapter bringup commands described in that documentation page. Your CAN Adapter Board must be plugged into your robot controller PC and that must be plugged into your CAN network with your motors on it. And please remember to power your motors.

You must install **Phoenix 6 Tuner X** by following the instructions from [this link](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html) or by downloading if from the [Github page](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases).

## Setup Process ##

This process requires SSH. If you are on a public network or university/corporate network, it is advisable to use a private network either not connected to internet, or a phone hotspot to prevent SSH permission issues over the network (and also to prevent security issues with opening a port on you robot controller PC).

### Tuner X Setup ###

Open Tuner X. Navigate to Settings on the mid-bottom left. Scroll down and open up the `SSH Configuration` setting. Change the `Username` and `Password` fields to the username and password of the robot controller PC that you are using. You will be SSH'ing into this PC from the computer with Tuner X on it.

![Tuner X SSH Configuration]({% link attachments/tuner-x/tuner-x-ssh-config.png %}){: style="width: 75%; display: block; margin: 0 auto;" }

{: style="text-align: center;" }

### Robot Controller PC Setup ###

Open a terminal window on your robot controller PC. Run `ip a` and note your computer's IP address on the network you would like to SSH over. Typically, this will be your wireless network.

Open another terminal window, and make a directory for a python script. You need a python script to run.

Create a python virtual environment with `pythnon -m venv venv` and activate it using the `source venv/bin/activate`. Assuming your robot controller PC is on Linux or Mac. This tutorial will cover robot controller PC's specifically using Linux. If you have Windows on you robot controller PC, good luck, as this guide will probably not apply to you.

Create a file named `diag_server.py` in root directory of your virtual environment. Paste the following code in it.

```python
import time
import os
os.environ["CTR_TARGET"] = "Hardware"

if __name__ == "__main__":
    import phoenix6
    from phoenix6 import hardware, controls; unmanaged

    bus = phoenix6.CANBus("can0")

    # CHANGE THIS TO YOUR MOTOR ID(s)
    # if you have multiple motors on your CAN bus you will need to find each motor ID and make a separate TalonFX object for each motor.
    motor_id_1 = 0
    # motor_id_2 = 1
    # Yes, you need Tuner X to see the motor ID
    kraken_1 = hardware.TalonFX(motor_id_1, bus)
    # kraken_2 = hardware.TalonFX(motor_id_2, bus)

    # Keep feeding motor(s) so it doesn't disable itself
    while True:
        unmanaged.feed_enable(0.1)
        time.sleep(0.1)
```

With the virtual environment active, install the phoenix6 API with this command
`python3 -m pip install phoenix6`
For more information, please visit the official [Phoenix 6 documentation page](https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation-nonfrc.html).

Run the python script with `python diag_server.py`. Once you have this running, if you are using the same CAN adapter boards as described in [CAN Adapter Board]({% link docs/Technical/Hardware/CAN Bus/CAN-Adapter-Board-Setup.md %}), you should see all lights (red, blue, and green) light up on the board, and the motor indication LEDs should be glowing (or blinking) solid orange.

Refer to [this link](https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/index.html) to understand the status LED codes on your own motors.

### SSH Setup ###

Once your `diag_server.py` script is running on the robot controller PC, navigate to the `Devices` tab on Tuner X on you other computer, and input the IP of you robot controller PC into the bottom left prompt window with the text `Enter Team # or IP` above it. Both your Tuner X PC and robot controller PC must be on the same network for this to work.

Once you have connected Tuner X to your robot controller PC, you should see your motor(s) on the `Devices` page.

![Tuner X Devices]{% link attachments/tuner-x/tuner-x-devices.png %}{: style="width: 75%; display: block; margin: 0 auto;" }

{: style="text-align: center;" }

Click into your motor(s) on Tuner X. You will be greeted with a variety of options. On the `Info` tab, you have the ability to view (and change) your motor ID, motor name, and you can update your motor's firmware as well. Tuner X will prompt you to update the firmware if it detects that your motor is using an outdated firmware. I recommend updating unless you have a good reason not to.

![Tuner X Motor info tab]{% link attachments/tuner-x/tuner-x-info.png %}{: style="width: 75%; display: block; margin: 0 auto;" }

{: style="text-align: center;" }

![Tuner X Motor info tab, updating firmware]{% link attachments/tuner-x/tuner-x-update-firmware.png %}{: style="width: 75%; display: block; margin: 0 auto;" }

{: style="text-align: center;" }

If you navigate to the `Control` tab, you can test the motors by controlling their speed and direction directly. **ENSURE THAT YOU HAVE CLICKED ENABLE AT THE TOP OR ELSE THIS WON'T WORK!!** In the picture below, I have selected the `Differential Duty Cycle` Control Mode and set the `AverageOutput` to 1. This turns our motor forward at full speed. you can set `AverageOutput` to 0 to stop it, and to -1 for full speed in the opposite direction. Vary the speed by choosing a decimal value in between -1 and 0 ot 0 and 1. This is a surefire method for testing the speed performance of your motors.

![Tuner X Control Tab]{% link attachments/tuner-x/tuner-x-control.png %}{: style="width: 75%; display: block; margin: 0 auto;" }

{: style="text-align: center;" }

When you are finished testing, stop the python script on your robot controller PC, and the SSH connection should disconnect automatically since the Phoenix 6 diagnostics server is no longer running.

> Author: Caleb Hans (https://github.com/caleb-hansolo)
>
> Last Reviewed: 2026-11-02
