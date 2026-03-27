---
title: Installing the Phoenix6 C++ API
parent: Mini PC
nav_order: 3
---

## Installing the Phoenix6 API ##

To control the Kraken X60 motors over CAN bus, the Phoenix6 API (C++, C#, Java, or Python) is required. We primarily use the C++ API.

To install the API, follow this [CTR Electronics Phoenix6 installation guide](https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation-nonfrc.html#apt-repository-installation), specifically the instructions under the `Apt Repository Installation` section.

**IMPORTANT DISCLAIMER:** You may see that you need to pay for a Phoenix6 Pro license in order to use their API. This is **NOT TRUE!!** The Phoenix6 API is free to use, whereas Phoenix6 *Pro* is not. All the Pro version provides is extra data from the motors, which we do not need for our use case.

**BEFORE** going through these instructions, you should be aware that `sudo apt update` may not work. Once you get past these instructions (`<year>` will be whatever year you have chosen - we use 2026):

```bash
sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr<year>.list "https://deb.ctr-electronics.com/ctr$<year>.list"
```

And once you open the .list file:

```bash
sudo nano /etc/apt/sources.list.d/ctr<year>.list
```

You will see this

```
deb [signed-by=/usr/share/keyrings/ctr-pubkey.gpg] https://deb.ctr-electronics.com/tools stable main
```

Inside the square brackets you must include the statement `arch=amd64`, since that will resolve an architecture mismatch when running `sudo apt update`. You do **NOT** need to change anything else. The correct .list file should look like this:

```
deb [arch=amd64 signed-by=/usr/share/keyrings/ctr-pubkey.gpg] https://deb.ctr-electronics.com/tools stable main
```

Once you have made this change, you should be good to run

```bash
sudo apt update
sudo apt install phoenix6
```
