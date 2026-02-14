---
title: Introduction to Programming
parent: Non-Programmer Friendly Zone
nav_order: 1
---

## Introduction to Programming

This guide will give a basic introduction to the major concepts of programming, go over some simple examples, and then redirect you to more comprehensive sources for learning more if you would like to continue exploring program. The main purpose of this guide is to equip you with enough information to be able to communicate with a programmer, not to teach you how to program

If you would like to learn more about the terms used here, please reference the [glossary]({% link docs/Non-Programmer Friendly Zone/Glossary.md %})

### Programming Concepts

Programming is essentially just various ways to tell a computer what to do. You write instructions in a programming language of choice and something called a compiler takes those instructions and translates it into machine code that the computer then executes

Firstly, there are a LOT of different programming languages and each one is meant to be good at a certain kind of thing. For example, some programming languages are really good at data processing (SQL), while others are really good at programming websites (JavaScript)

There's another way to classify programming languages: low level and high level. In Lunabotics, we use BOTH high and low level programming languages for different purposes, so it is important to be roughly familiar with both

   1. Low level programming languages work really closely with the hardware. They allow you to have precise control over memory and can send direct commands to motors. The compiler, aka the translator, has to do a lot less work because you're speaking in a language that is very similar to machine code. This usually means that low level programming languages are more efficient but more difficult to write in
   2. High level programming languages do NOT work closely with hardware. It is usually extremely difficult to directly tell hardware what to do and more of the computer processes are abstracted, or hidden, away from you. The compiler has to do a lot more to translate your instructions because what you're writing is extremely disconnected from machine code. This usually means that high level programming languages are easier to write in but slower to execute

Lastly, there are yet more ways to categorize programming languages based on their paradigms. This basically means how does the language choose to write its instructions. This can get a little confusing, and we are only using one paradigm, so I am only going to explain that one but know that there are many more you can look into if you want. The programming subteam on Lunabotics uses an object-orientated paradigm. This means that the way we represent information in our code is through objects. For example, we can make an object named RobotArm and give it a property like length and say it can do the action move_up. More specifics on how this works will be explained during the examples section

### Programming Examples

It can be really hard to learn programming without examples, so here are two code blocks of equivalent Python and C++ code (the two languages that Lunabotics uses). They do the exact same thing and output the same stuff, and both of them are object-orientated, but one of them is low level (C++) and the other is high level (Python)

Note that anything that comes after a `#` in Python or `\\` in C++ is called a comment. This is text that will not get translated into machine code, it's only there to help explain what's going on and make things more organized

Consider the following Python code

```python
import math # 1. Sometimes you want to use code that other people already wrote in other files
            # You can import these already existing files to use yourself

# 2. A class is the object we are making. Here, we are making a Rover object
class Rover:

   # 3. This is a function. It defines actions that the object can do
   # 4. This is a special kind of function that initializes the object
   def __init__(self, x: float = 0.0, y: float = 0.0, heading: float = 0.0):
      self.x = x
      self.y = y
      self.heading = heading  # degrees; 0° = +X (east)

   # 5. These next three functions define how the rover can move
   def forward(self, distance: float) -> None:
      rad = math.radians(self.heading)
      self.x += distance * math.cos(rad)
      self.y += distance * math.sin(rad)

      # 6. This print statement sends whatever is written after it to the console so you can see it
      print(f"Drive {distance} m  ➜  pose=({self.x:.2f}, {self.y:.2f}), θ={self.heading:.0f}°")

   def turn_left(self, angle: float) -> None:
      self.heading = (self.heading + angle) % 360
      print(f"Turn left {angle}°  ➜  θ={self.heading:.0f}°")

   def turn_right(self, angle: float) -> None:
      self.heading = (self.heading - angle) % 360
      print(f"Turn right {angle}° ➜  θ={self.heading:.0f}°")


# 7. This is a special kind of function that kind of acts like the main control center for the rover
def main() -> None:
   rover = Rover() # 8. Here we create the Rover object

   # 9. Here we create a list of instructions for the Rover to execute
   script = [
      ("forward", 5),
      ("left", 90),
      ("forward", 3),
      ("right", 90),
      ("forward", 2),
   ]

   # 10. We loop through all the instructions and call the corresponding
   # function depending on which command it has received
   for cmd, val in script:
      if cmd == "forward":
         rover.forward(val)
      elif cmd == "left":
         rover.turn_left(val)
      elif cmd == "right":
         rover.turn_right(val)

   print(f"Final pose = ({rover.x:.2f}, {rover.y:.2f}), θ={rover.heading:.0f}°")


# By default, object orientated programs do not know where to start in a program.
# This is a special command that goes at the end of Python files that tells the program
# to start at the main() function if we run the file
if __name__ == "__main__":
   main()
```

Now, compare that to the same steps but in C++

```c++
#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <cmath>

class Rover {
    double x, y;           // metres
    double heading;        // degrees; 0° = +X (east)
    static constexpr double pi() { return std::acos(-1); }

public:
    Rover(double x0 = 0, double y0 = 0, double h0 = 0) : x{x0}, y{y0}, heading{h0} {}

    void forward(double distance) {
        double rad = heading * pi() / 180.0;
        x += distance * std::cos(rad);
        y += distance * std::sin(rad);
        std::cout << "Drive " << distance << " m  ➜  pose=("
                  << x << ", " << y << "), θ=" << heading << "°\n";
    }
    void turn_left(double angle) {
        heading = std::fmod(heading + angle, 360.0);
        std::cout << "Turn left " << angle << "°  ➜  θ=" << heading << "°\n";
    }
    void turn_right(double angle) {
        heading = std::fmod(heading - angle + 360.0, 360.0);
        std::cout << "Turn right " << angle << "° ➜  θ=" << heading << "°\n";
    }
    void report() const {
        std::cout << "Final pose = (" << x << ", " << y << "), θ="
                  << heading << "°\n";
    }
};

int main() {
    Rover rover;
    std::vector<std::pair<std::string, double>> script = {
        {"forward", 5},
        {"left",    90},
        {"forward", 3},
        {"right",   90},
        {"forward", 2},
    };

    for (const auto& [cmd, val] : script) {
        if (cmd == "forward")      rover.forward(val);
        else if (cmd == "left")    rover.turn_left(val);
        else if (cmd == "right")   rover.turn_right(val);
    }
    rover.report();
    return 0;
}
```

Final output of both programs if you were to run them:

```console
Drive 5 m  ➜  pose=(5.00, 0.00), θ=0°
Turn left 90°  ➜  θ=90°
Drive 3 m  ➜  pose=(5.00, 3.00), θ=90°
Turn right 90° ➜  θ=0°
Drive 2 m  ➜  pose=(7.00, 3.00), θ=0°
Final pose = (7.00, 3.00), θ=0°
```

If all of that was a little confusing, that's okay! This section isn't really about teaching you how to program, but just giving an example of what some code could look like and some basic differences between C++ and Python. Notice how Python reads much more closely to regular English instructions while C++ has a lot of weird things going on that don't really make sense if you don't know C++. This is the difference between high and low level programming languages

### Learning How to Program

A full programming lesson is outside the scope of this guide, but there are some great resources out there for you. There is nothing about programming that you can't learn from free resources out there on the internet. I have a few big recommendations:

If you would like to learn Python, W3Schools has a pretty solid Python programming course. This one is really nice because you can program in the browser for the exercises and not have to download anything. If you just want to dip your toes into the world of programming, I recommend starting here at W3Schools: <https://www.w3schools.com/python/python_intro.asp>

If you would like to learn C++, I would actually recommend that you learn C first and then learn C++ after that. C++ is a bit of a nightmare while C is both incredibly useful to know and largely transferrable to C++. C is an extremely foundational programming language that a lot of people recommend you learn as your first language. It is on the lower level side, so if you want to start getting into robotics programming as a hobby or seriously, I recommend you start here, otherwise just learn Python. The computer science professor who teaches the introductory C course at UNL actually records all of his lectures and uploads them for free on YouTube, and he is a really good professor. CSCE-155E is the course, and here is the link to the Fall 2024 playlist: <https://www.youtube.com/watch?v=mD5EQXzdn98&list=PL4IH6CVPpTZWhIUrPho27zNE3-Ut4D0Jk>

After you learn C, here is the W3Schools link for C++, though I personally find that this particular set of tutorials does not go in depth enough: <https://www.w3schools.com/cpp/default.asp>

**The best way to learn how to program is to do projects.** I can almost guarantee you that none of this will really set in unless you actually put it into practice. If you really want to learn programming very well and don't just want a vague idea of how it works, you should decide on a simple project that sounds fun to you and just do it. Pick something like programming pong in Python or something simple like that. Google is your friend and there is nothing you can't learn from forum posts and guides people have already made. Don't be afraid of not doing things the best way, just do it and learn from it

### Where to Go Next

Now that you have a basic understanding of programming, you can really go anywhere in the Non Programmer Friendly Zone. Specific questions that people have asked programming to explain will be listed in here, though for next steps I recommend going to [Introduction to ROS2]({% link docs/Non-Programmer Friendly Zone/Introduction-to-ROS2.md %})

If you are on a mission to become a programming nerd, you will also need to know what VSCode is and how to set it up with everything you'll want, which you can find at [How to Use VSCode]({% link docs/Administrative/VSCode-Setup.md %})

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
