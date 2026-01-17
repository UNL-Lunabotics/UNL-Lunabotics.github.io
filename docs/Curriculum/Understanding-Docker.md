---
title: Understanding Docker
parent: Curriculum
nav_order: 5
---

# Understanding Docker

In robotics, Docker is a very useful tool that allows developers to run the same code on many different machines, regardless of the operating system (OS). This page will walk you through a basic overview of what Docker is and does, and will provide some helpful resources to deepen your understanding of Docker.

## What is Docker? 

Docker is a tool that allows you to run loosely isolated environments called "containers." You can have multiple containers running on one OS, each not interfering with one another. Containers contain all the packages, dependencies, and necessary software to run whatever is in the container, meaning you don't need to install packages and dependencies directly onto the host OS. 

### What is a Container?

A container is an isolated process that contains all the packages, dependencies, and files that it needs to run. Unlike a virtual machine, it doesn't run an entire new operating system, but rather uses the kernel of the host OS. 

### What is an Image?

According to the official Docker documentation, "a container image is a standardized package that includes all of the files, binaries, libraries, and configurations to run a container." So, a Docker image is like a blueprint for a container. Images are immutable, which means that once they have been "built," any containers based off of that image cannot be changed if the image is changed. If you would like to change an image and the containers based off of that image, you must edit the image, rebuild it, and then restart the desired containers to be based off of the new version of the image. 

More information about images can be found [here](https://docs.docker.com/get-started/docker-concepts/the-basics/what-is-an-image/).

### What is a Dockerfile?

A Dockerfile is a text file that is used to describe how to build a container image. It describes all the relevant commands to run, files to edit, and packages to install when the container image is built.

More information about Dockerfiles can be found [here](https://docs.docker.com/get-started/docker-concepts/building-images/writing-a-dockerfile/).

### How do I Install Docker?

For windows, see [this page]().

For Linux, see [this page]().

### Further Research

If you have any more questions or would like to learn more about Docker, containers, images, and more, please refer to the [Docker Docs](https://docs.docker.com/get-started/docker-overview/).

## Why do We Use Docker?

In robotics, Docker containers are often used when the host operating system isn't fully equipped to run ROS2 (ROS2 only supports development on Ubuntu). So, if someone is running Windows 11 and wants to develop with ROS2, they can create a Docker image with the necessary packages and dependencies from Ubuntu, as well as whatever other software they may be using, in order to have a blueprint for a container where they can actually create and develop in a real ROS2 workspace. If you are interfacing with hardware in a Docker container, there can be issues with permissions, which you may need to fix by edting the priveleges of Docker or the container itself. 

For integration with your robotics systems, ROS2 has official documentation regarding how to set up your Docker images and containers [here](https://docs.ros.org/en/rolling/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html).

As described in the documentation linked above, another useful tool that is also used often with ROS2 Docker development is the VSCode extension called [Dev Containers](https://code.visualstudio.com/docs/devcontainers/create-dev-container). This extension allows you to develop directly in your Docker container in the VSCode IDE. Dev Containers also allows you to install VSCode extensions, mount workspaces/directories, edit priveleges, and run commands inside your container environment. It is a very powerful tool greatly simplifies ROS2 Docker development.

> Author: [Caleb Hans](https://github.com/caleb-hansolo)