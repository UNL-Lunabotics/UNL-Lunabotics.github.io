---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults
title: Home
nav_order: 0
layout: home
---

# Home

## What and Why?

Welcome to the lunabotics documentation wiki! This was started in 2025 over the summer as our subteams' first big push towards proper documentation.

In the 2024-2025 competition season, programming ran into a LOT of issues. In the past, the programming subteam had basically just been one guy doing everything and communicating nothing to anyone. Because of this, that year we were essentially a brand-new programming team and all of our members were starting from scratch. This caused massive issues with getting things up and running and contributed to our robot not moving at UCF.

## How to Navigate

<u>You will navigate via the sidebar on the left side of the screen</u>

This repository is split into subsections with a specific purpose and specific audience it is written for, the details of which are specified below. The first page in every section is a more in depth introduction as well as table of contents for the section, and it is recommended that if you are new to this documentation, you should start there whenever looking through a subsection.

<u>If you are familiar with programming</u>, basically all of the subsections will be accessible to you

<u>If you are NOT familiar with programming</u>, you should mainly stay in the Non-Programmer Friendly Zone. This is specifically written for that purpose, though you should also be able to navigate the Systems Engineering subsection as well.

## How to Edit

It is HIGHLY recommended doing any writing more complicated than grammar or spelling edits within VSCode rather than the browser editor. Firstly, it is just easier to navigate and edit a lot of files in VSCode rather than the browser. You can work on multiple pages at once before having to do one commit and the GitHub repository includes folder organization for convenience (the folders do not display on wiki). Additionally, VSCode will also include the recommended extensions for this repository, <u>including the linter that we use and some convenience features for editing in Markdown</u>

If you would like to learn how to set up VSCode to edit this repository, please see [VSCode Setup](https://github.com/UNL-Lunabotics/lunabotics-documentation/wiki/VSCode-Setup)

## Subsection Breakdown

- **Curriculum**: This is where you can find all our pages that will explain all the important concepts for you to know. These documents assume that you have some programming experience but may not know the specific languages we will be using.
  - What it will teach you: How to program a robot and the important concepts behind that
  - What it will not teach you: How to program in C++ or Python or understand basic programming concepts. Some of these resources are available in the Non-Programmer Friendly Zone

- **Non-Programmer Friendly Zone**: This is exactly what it sounds like. If you have never programmed before and either want to learn or just understand what is going on, these resources are meant to help you get a basic understanding of what we're doing. This was primarily created because a lot of people on the mechanical subteams wanted to better communicate with programming and understand how to work with us. It will include a simplified and sped up ROS2 curriculum designed to help accomplish this communication

- **Technical Documentation**: This is where you will probably spend most of your time if you are on the programming subteam. Here, we will outline in way more detail how to install things, configure things, launch things, upgrade things, the pros and cons of different options, etc. These will be written assuming you have already gone through the ROS2 Curriculum (not the simplified one)

- **Systems Engineering**: This will include more year specific technical documentation that will be necessary for systems engineering. This includes things like architecture design per year and diagrams. This section will be written for everyone. It is incredibly important that a balance is maintained between making technologically informing documents AND keeping them accessible to all audiences of any programming experience

- **Administrative**: This section contains information needed to maintain this repository as well as the team as a whole. It will include subteam lead responsibilities, criteria for selecting an apprentice, and how to maintain things like our virtual machines, Google Docs, and this documentation. Likely only the lead and the lead apprentice will frequent this section

> Author: Ella Moody (https://github.com/TheThingKnownAsKit)
