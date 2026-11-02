---
title: Setting Up VSCode
parent: Technical
grand_parent: Development Tools
nav_order: 1
---

# Setting Up VSCode

## What is VSCode?

VSCode is something called an Integrated Development Environment (IDE). Basically, this is the programming equivalent of Microsoft Word or Google Docs. In order to write code, programmers need an application they can actually type in. Theoretically, you can write code in Google Docs if you really wanted to, but the nice thing about IDEs is that they are INTEGRATED. Not only can you type code in them, they also provide tools to help you format code, run the code, detect errors, and more.

There are many IDEs out there, but the nice thing about VSCode is that:

- It's available on every operating system
- It is very simple and not cluttered with a billion different tools and menus that no one knows how to use (think of the difference between how many menus Word has vs Google Docs)
- It supports every language you can program in
- Most importantly, it's free!

## Downloading VSCode

Go to [this](https://code.visualstudio.com/Download) link and download the correct installer for your computer. Please note that you can ignore the small blue buttons underneath each operating system, and instead you can just click the big blue button directly under the logos. After that, open the installer and follow the steps.

## Getting Started with VSCode

The important thing to know about VSCode is that it is technically just a text editor. However, it has this feature called extensions. If you open the extensions menu and search for whatever programming language you want to use, you can find the corresponding extension that will enable you to run the code. **You HAVE to install the language's extension to be able to use it (most of the time)**.

For example, if I wanted to install the Python extension, I would navigate to the extensions menu on the left-hand sidebar and enter 'Python' into the search bar. After it finishes loading, I would select the extension called Python with the publisher Microsoft and click the blue install button.

**All the extension in VSCode does is enable you to use a programming language you have already downloaded the software for. You still have to download the language's SDK (most of the time)**. Usually, you can find this software by googling the language's name plus 'SDK', so for example, 'Python SDK'. SDK stands for Software Development Kit, and it is what allows you to actually run the software. Some languages, such as C, are compiled, and you do not need to download their SDK and instead need to download the compiler that they use.

After you have the language extension installed and the SDK (or compiler) downloaded, you can begin coding!

## Setting up Profiles

VSCode profiles let you separate what extensions you want to use in each repository. This is really useful so you don't have to have every extension enabled in every single repository even when it's not relevant.

You can create a profile by doing `CTRL/CMD + SHIFT + P` and then `Profiles: New Profile...`. I recommend at least creating a profile for Lunabotics work and Documentation since they use dramatically different extensions.

## Installing Recommended Extensions

Repositories from our team always come with a `extensions.json` that gives you extensions it recommends you install. Shockingly, we do actually recommend you install these. There should be a pop-up when you open the repository and all you should have to do is click yes.

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
>
> Last Reviewed: 2026-11-02
