---
title: Glossary
parent: Non-Programmer Friendly Zone
nav_order: 4
---

## Glossary

### Table of Contents

[A](#a) &bull; [B](#b) &bull; [C](#c) &bull; [D](#d) &bull; [E](#e) &bull; [F](#f) &bull; [G](#g) &bull; [H](#h) &bull; [I](#i) &bull; [J](#j) &bull; [K](#k) &bull; [L](#l) &bull; [M](#m) &bull; [N](#n) &bull; [O](#o) &bull; [P](#p) &bull; [Q](#q) &bull; [R](#r) &bull; [S](#s) &bull; [T](#t) &bull; [U](#u) &bull; [V](#v) &bull; [W](#w) &bull; [X](#x) &bull; [Y](#y) &bull; [Z](#z)

### A

- Abstraction: The process of simplifying a complex system by hiding unnecessary details, so the developer can focus on the most important details.
- Action [[ROS2](#r)]: One of the three forms of communication between [nodes](#n) in ROS2., intended for long running tasks. They consist of three parts: a goal, feedback, and a result. Actions are similar to services, only they can be canceled during execution and provide constant feedback rather than a single response.
- Agile: A software development philosophy that focuses on an iterative process and places strong emphasis on teams performing regular demonstrations of the in-development project.
- Application-Programmer Interface (API): An API,is a connection between two or more computers or computer programs. APIs are not intended to be used by humans directly, instead being used almost exclusively by computers to send or receive data from other computers.
- Asynchronous: Asynchronous programming is a technique that allows events to occur independently of the program's main [control flow](#c). This allows multiple tasks to run concurrently without blocking the execution of other tasks.

### B

- Bit: A bit is the most basic unit of information in computing. It represents a logical state with one of two possible values, commonly represented by `1` and `0`.  
- Build: A software build is the process of converting written code into something that can be run on a computer.
- Byte: A unit of digital information consisting of eight bits. The byte is the smallest addressable unit of memory in many computer architectures. Units of information larger than a byte follow standard Metric system prefixes (Ex: kilobyte, megabyte)

### C

- Call: A Call is an instruction that tells a program to execute a particular function or other subroutine.
- Callback: A callback function is a function that can be passed as a parameter into another function. That function can then call upon the callback function inside its own definition.
- Camel Case: A writing format where each word in a phrase is capitalized and no spaces or punctuation are used. Often stylized as "CamelCase" or "camelCase". Camel Case is a common convention for writing multi-word [identifiers](#i) in computer programs.
- Class: A class is an entity structure in many programming languages used to create [objects](#o). The features of a class can vary in different programming languages, but they generally consist of states and behaviors associated with the object.
- Client-Side: Refers to the Client end of the [client-server model](https://en.wikipedia.org/wiki/Client%E2%80%93server_model). The client side of the model usually refers to software running on an end user's system, which connects to the [server-side](#s) as necessary to perform certain operations.
- Command-Line Interface (CLI): The CLI is a means of interacting with software by executing written commands in a terminal. It is also sometimes called a command-line shell, or simply, shell.
- Compiler: A compiler is software that acts as a sort of translator that translates (or compiles) code from one programming language to another. It is most often used to convert code written in a [high-level language](#h) to a [low-level language](#l). Languages like C, C++, and Rust require a compiler to run code.
- Constant: A constant, in many programming languages, is a type of [identifier](#i) that represents a known value that will never change. This is in contrast to [variables](#v), which can be changed.
- Control Flow: The process of how the execution of code progresses from one command to the next.

### D

- Declare: Declaration is the process of specifying the properties of an [identifier](#i) for the first time. The contents of a declaration will vary from language to language, but they generally specify the type of data being declared and the actual data itself.  

### E

- Encapsulation: The practice of bundling data with the specific code or structures that operate on that data. This prevents external operations from affecting data it's not supposed to.

### F

- Framework: A software framework provides code with reusable, generic functionality that developers can utilize to simplify their code and focus on the actual task at hand rather than building up common infrastructure from scratch
- Function: A function is a kind of [identifier](#i) in computer programming. They work very similarly to functions in mathematics, effectively grouping up code meant to perform a particular task into a callable unit of logic, allowing it to be invoked more easily. You might also hear terms like method, routine, and procedure used. While these are all technically different from functions, they share a lot of the same behavior.
  
### G

- Garbage collection: A feature in some programming languages that attempts to automatically reclaim memory that is no longer being used by the program. This relieves the developer from having to manually manage the memory used by their program.
- Gazebo: An open-source robotics simulator developed by Open Robotics, the same parent company that owns [[ROS2](#r)]. There are two versions of Gazebo. The original Gazebo, now known as Gazebo Classic, was discontinued in January 2025. The modern Gazebo, formerly known as Gazebo Ignition, sometimes called New Gazebo, is the currently supported version.
- Git: Git is an open-source [version control](#v) system that allows developers to keep track of changes made to a project and seamlessly collaborate on development from multiple systems. For more information, see [Git for Non-Developers: A 10-Minute Guide](https://medium.com/@avnishyadav25/git-for-non-developers-a-10-minute-guide-614690c87126), by Avnish Yadav.
- GitHub: GitHub is a Microsoft-owned proprietary developer platform that allows developers to create, store, manage, and share their code using Git. (Source: [Wikipedia](https://en.wikipedia.org/wiki/GitHub))
- GitLab: GitLab is a platform similar to GitHub that allows developers to create, store, manage, and share their code using Git. Unlike GitHub, GitLab provides the option for developers to self-host their repositories, rather than relying on the cloud.
- GNU Compiler Collection (GCC): A collection of [compilers](#c) that support multiple programming languages, CPU architectures, and operating systems.  
- Graphical User Interface (GUI): A form of [user interface](#u) that uses graphical icons and visual indicators to make interacting with software easier than the classic [command-line interface](#c).

### H

- High level language: Programming languages designed with the intention of being human-readable. These languages often hide a lot of the complex details of a program's functionality behind an easier to understand [abstraction](#a) layer.

### I

- Identifier: Any character, symbol, or other lexical token that denotes entities within a programming language. These entities can include variables, functions, objects, and other data types.
- Inheritance:The mechanism of basing an [object](#o) or [class](#c), known as a subclass upon another object or class, known as a superclass. The subclass will retain all of the attributes of the superclass, in addition to any new attributes specified in the subclass.
- Instance:
- Integrated Development Environment (IDE): An IDE is a category of software that provides a comprehensive set of tools for software development. These platforms bundle various individual tools (i.e. [compilers](#c), [linters](#l), etc.) together to ensure compatibility between them and improve the productivity of software developers.
- IntelliSense: Intellisense is Microsoft's implementation of intelligent code-completion in VSCode and Visual Studio. It (attempts to) speed up the development process by predicting what code needs to be written and providing a shortcut for autocompleting that line or block of code.
- Interpreter: An interpreter is a kind of software that executes code without first [compiling](#c) it into [machine code](#m). Python and JavaScript are two popular languages that directly interpret code without relying on a compiler.

### J

- JavaScript Object Notation (JSON), is a data-serialization language similar to [XML](#x), but utilizing syntax similar to that used to construct objects in JavaScript. This makes the language both human-readable and very easy for computers to parse, even faster than XML.

### K

- Kebab case: Also sometimes called dash case, this is a grammar convention used in many programming languages charactarized by the use of lowercase letters and hyphen characters in place of whitespace for multi-word phrases. Often stylized as `kebab-case`.

### L

- Lambda function: A block of code that is not bound to any [identifier](#i). Also sometimes called an anonymous function.
- Launch File [[ROS2](#r)]: A launch file in ros2 allows you to configure and execute multiple ROS2 nodes simultaneously with a single command (Source: [Foxglove](https://foxglove.dev/blog/how-to-use-ros2-launch-files)).
- Linter: A series of programming tools that check for code style consistency and formatting errors.
- Low level language: Programming languages are considered low-level if they provide little to no [abstraction](#a) from the computer's raw instruction set. These languages allow the programmer to exert full control over the programs they write, with the tradeoff of being far more difficult to work with and the risk of error being significantly higher. Examples of low-level languages include Assembly and raw machine code.

### M

- Machine Code: Machine code is the raw instruction fed to a computers central processing unit (CPU) in order to execute tasks. It is generally not human readable. Different CPU architectures use different machine code.
- Message [[ROS2](#r)]: Messages are a special data structure used by ROS2 to exchange information between nodes. They are denoted by the `.msg` file extension.
- Multithreading: Multithreading is a programming model that allows for multiple process to be executed at once utilizing a multithreaded central-processing unit (CPU). This is especially useful for complex computations that would take a very long time if each step had to be executed one after the other.

### N

- Nav2: Nav2 is a robotics navigation framework that provides various autonomous functions to a robot. It allows robots to navigate through complex environmebnts and complete user-defined application tasks (Source: [Nav2 Docs](https://docs.nav2.org)).
- Node [[ROS2](#r)]: An element in a ROS2 system that serves a single, modular purpose, such as controlling wheels or publishing sensor data (Source: [ROS2 Docs](https://docs.ros.org/en/rolling/ROS-Framework/nodes/Working-with-nodes/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)).

### O

- Object: An entity in many programming languages that has state, behavior, and identity. Objects represent an individual, identifiable item, unit, or entity, real or abstract, with a defined role in the system (Source: [Wikipedia](https://en.wikipedia.org/wiki/Object_(computer_science))).

### P

- Package [[ROS2](#r)]: A directory that contains files and folders pertaining to a ROS2 project. One bot might have multiple packages with each one responsible for a particular function of the bot (Source: [Automatic Addison](https://automaticaddison.com/organizing-files-and-folders-inside-a-ros-2-package)).
- Paradigm: A programming paradigm refers to the approach or style that dictates how a computer program is designed and implemented. Some programming languages are designed with a particular programming paradigm in mind, but many of the most popular programming languages are designed in such a way that the developer can decide for themselves what paradigm they want to design their software around.
- Parameter: Also known as a formal argument, a parameter is an [identifier](#i) that is passed into a function. The function's definition defines what kind of parameters it can take.
- Parse: The process of breaking up an input, such as a string of text, and breaking it into structured parts as defined by the specified grammar rules.
- Pipeline: A data pipeline is a set of data processing stages where the output of one stage is the input of the next stage. Think of an assembly line in a factory, one step of building the product is completed before the product is sent to the next station for the next stage of construction.
- Pointer: A pointer is a type of object or variable (depending on the programming language) that stores a memory address. The memory address can "point" to other data types stored by the program, effectively allowing you to refer to that value without directly interacting with its [identifier](#i). See the article [Pointer in programming](https://www.geeksforgeeks.org/dsa/pointer-in-programming/) by GeeksforGeeks if you want to learn more, as pointers can be a pretty complex topic to grasp.
- Pseudocode: A way of writing non-functional code with the goal of being translatable into any programming language. This is most often used to describe the steps in an algorithm. The goal of pseudocode is to be as human readable as possible while maintaining a structure that makes it easy to implement.
- Publisher [[ROS2](#r)]: The publisher node sends messages on a specific [topic](#t), allowing [subscriber](#s) nodes on that topic to receive the messages. A publisher node can also publish messages from multiple topics.

### Q

### R

- Race condition: A type of error that occurs when multiple processes attempt to modify the same shared data at the same time, resulting in unpredictable behavior such as bugs, corruption, or security vulnerabilities.
- Robot Operating System 2 (ROS2): An open-source software suite that provides various tools for deploying, running, and maintaining robotic applications. Despite its name, ROS2 is not an operating system, but rather a series of frameworks, libraries, and other tools.
- ROS2 Control: A framework that enables the real-time control of your robot using a standardized interface built for [ROS2](#r).
- Runtime: The period of time in which a program is being executed.

### S

- Script: A relatively short and simple set of instructions that typically automate an otherwise manual process (Source: Wikipedia). Whereas a computer program might be capable of performing a wide variety of tasks, a script is typically intended to perform few tasks.
- Secure Shell Protocol (SSH): A secure communication protocol most often used to allow a user to access and control a computer remotely over a network.
- Server-Side: Refers to the Server end of the [client-server model](https://en.wikipedia.org/wiki/Client%E2%80%93server_model). The server is the "central" system that sends information to the [client-side](#c) as it is requested.
- Service [[ROS2](#r)]: One of the three forms of communication between [nodes](#n) in ROS2. Services follow a call-and-response model, where information is only provided to the caller (or [client](#c)) upon request from the [server](#s) (Source: [ROS2 Docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)). This distinguishes it from [topics](#t), wherein information is constantly being provided and received via publishers and subscribers.
- Simultaneous Localization and Mapping (SLAM): A process where a computer constructs or updates a map of an unknown environment while *simultaneously* keeping track of an entity's location within it (Source: [Wikipedia](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)).
- Snake Case: A naming convention common in programming characterized by each space in a multi-word phrase being replaced by an underscore. Often stylized as `snake_case`.
- Software Development Kit (SDK): A collection of software development tools bundled into one installable package. These kids are generally platform/operating system specific, as different platforms will have different tools for developing software.
- Subscriber [[ROS2](#r)]: A node that listens for messages published on a specific topic and processes the received data. A subscriber node can "subscribe" to multiple topics in a system to receive information from multiple sources simultaneously.
- Syntax: Just like natural language, each computer programming language has its own set of grammar and rules the programmer has to follow when writing code. These rules are known as the syntax of the language. If a developer attempts to run a program with incorrect syntax, the program will most likely return a syntax error.

### T

- Terminal User Interface (TUI): Also known as a text-based user interface, a TUI is a sort of middle-ground between a [graphical user interface](#g) and a [command-line interface](#c). A TUI allows the user to interact with the software in more ways than just typing in specific commands, but they are far more limited than graphical interfaces, as they are only capable of displaying text. Two TUIs commonly used today are Vim and Nano.
- Topic [[ROS2](#r)]: One of the three forms of communication between [nodes](#n) in ROS2., they allow information to be passed from one or more [publisher](#p) nodes to one or more [subscriber](#s) nodes.

### U

- Unified Robot Description Format (URDF): An [XML](#x) based file format that represents the physical model of a robot. It is the standard ROS2 uses for representing robots and is also used by other tools like Rviz and Gazebo sim. For further reading, see [Understanding URDF Files]({% link docs/Curriculum/Understanding-URDF-Files.md %})
- User Interface (UI): The user interface refers to the space in a program where the interaction between humans (users) and the machine occur. Most often, a program will either use a [graphical user interface](#g) (GUI), a [Terminal user interface](#t) (TUI), or a [command-line interface](#c) (CLI).

### V

- Variable: A variable, in many programming languages, is a type of [identifier](#i) that represents a known value that might change. This is in contrast to [constants](#c), which never change.
- Version Control: The practice of controlling, organizing, and tracking different versions in history of computer files (Source: [Wikipedia](https://en.wikipedia.org/wiki/Version_control)) This process is most often automated using version control systems like [Git](#g).

### W

- Workspace [[ROS2](#r)]: A workspace is a directory on your computer containing ROS 2 packages (Source: [ROS2 Docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)).

### X

- Extensible Markup Language (XML): A text-based file format for storing, transmitting, and reconstructing data (Source: [Wikipedia](https://en.wikipedia.org/wiki/XML)). XML is designed to be simple and widely applicable, as well as being easy for a computer to parse.
- Xacro: Xacro, which stands for XML macros, is a special version of XML files that allows you to break up your data into multiple smaller files. These smaller files can then be combined into one large file by your computer so that they can be read as any other XML file.

### Y

- YAML: A text-based data serialization language similar to [XML](#x), but with a stronger focus on being human readable and having less strict structure specifications than XML.

### Z

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)  
> Author: Jesse Mills (<https://github.com/JesseMills0>)
