---
title: CAN Adapter Board Setup
parent: CAN Bus
nav_order: 2
---

## CAN Adapter Board Setup ##

We use a [CANable 2.1 clone board](https://a.co/d/079qc0wA) ($15 on Amazon) for the CAN communications with our rover. This adapter takes input from a USB-C type connector connected to our robot's Mini PC, and converts those into CAN signals that our motors can understand. 

### CAN Adapter Board Firmware Reset ###

When ordered, the CAN adapters will come with a firmware called `slcand`. This needs to be reset, as we have only been able to get the Phoenix 6 API working with another firmware, called `candlelight`. 

To reset the firmware on the board, you must first put the board into boot mode. To do this, short the boot pins, and while they are shorted, plug the CAN adapter into your computer of choice. The light on the board should be solid red, with no other colors. You may un-short the boot pins once the CAN adapter is plugged in.

Once that is done, navigate to the [official CANable firmware updater](https://canable.io/updater/canable2.html). Ensure you are on a **Chromium-based browser** (e.g. Google Chrome, Chromium, Vivaldi), or this will not work. Click 

Once you navigate to the [official CANable firmware updater](https://canable.io/updater/canable2.html), select the `candlelight` firmware from the dropdown, and hit the `Connect and Update` button. Select your board from the pop-up window (should show up as something like `DFU in FS mode`), and then hit `Connect`. The lights should blink really fast and then return to being solid. Once this has happened, and once you get confirmation from the updater website, you will know that the firmware has been updated. You can now unplug the board and remove the short from the boot pins if you haven't done so already.

### CAN Adapter Board Network Interface Setup ###

Before continuing to this step, please make sure that the firmware is updated to `candlelight` as per the previous instructions on this page! These instructions will not work otherwise. 

Making sure the boot pins are **NOT** shorted, plug the CAN adapter into your computer again. You should see the lights on the board glow solid red and green. This confirms that the board is not in boot mode, and is ready to configure. 

Open a terminal window. Run `lsusb`, and make sure your CAN adapter shows up. It should look something like this:
```
Bus 003 Device 005: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter
```

Once you have confirmed that, run `ip a` or `ip link` in the same terminal window. You should see a `can0` network interface (likely near the bottom). It should look like this:
```
5: can0: <NOARP,ECHO> mtu 16 qdisc noop state DOWN group default qlen 10
    link/can 
```

Once you have successfully confirmed that your CAN adapter shows up as a USB and network interface, you should run this command
```bash
sudo ip link set can0 up type can bitrate 1000000
```
replacing the number after bitrate with whatever bitrate you want to communicate at over you CAN bus. The Kraken x60 motors use the CAN FD protocol, which takes a bitrate of 1,000,000 bits per second, which is why we use the number `1000000`. 

Once you have ran this command, run `ip a` or `ip link` again. You should see that the can0 interface is now up and running:
```
5: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP group default qlen 10
    link/can
```
You should also see that a solid blue light has lit up on the board itself. This indicates that the board is up and ready to communicate! If this blue light is ever off, you will know that the can0 network interface is not up, and the adapter will not be able to communicate. A blinking blue light means the CAN adapter is transmitting data. Right now, all three lights (red, blue, and green) should be glowing solid.

You are now ready to use the adapter to communicate with the devices on your CAN bus!
