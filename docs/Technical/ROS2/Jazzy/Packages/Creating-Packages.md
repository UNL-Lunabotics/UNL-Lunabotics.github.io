---
title: Creating Packages
parent: Packages
nav_order: 1
---

## Creating Packages

Packages are how you structure your ROS2 code. Each package should be for it's own specific purpose, and additional libraries that package will need are imported here.

There are two kinds of packages: ament_cmake and ament_python. The CMake package is compiled, while the Python one is not. In general, the only thing that can go in ament_Python packages are files that do not need to be compiled, such as Python scripts, xml files, YAML files, and SDF files. Anything that needs to be compiled, like C or C++ files or any custom ROS2 components (services, messages, interfaces, etc).

{: .important}
Everything will work in a CMake package, but not everything will work in a Python package. It is a good rule of thumb that when in doubt, make it an ament_cmake package.

Whether or not you just make everything a CMake package or utilize Python packages when applicable is up to you. It is perfectly acceptable to mix the different package types in the same repository. Currently, the Lunabotics team at UNL makes our bringup, URDF, and sim packages Python and everything else CMake.

To create the packages, cd into wherever in your repository you want your ROS2 code to live, and do ONE of the following terminal commands:

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
```

You can look at the official documentation for [ROS2 Packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) at the link.

### CMake Packages

When you create your CMake package, you'll end up with a file structure like this:

```
package_name\
   \include\package_name
   \src
   CMakeLists.txt
   LICENSE
   package.xml
```

You are allowed to delete the include folder if you wish (do not do that if you plan on writing any C or C++ code) and you can also delete/rename the src folder. The LICENSE file is not technically required by highly encouraged to keep. You CANNOT delete the `CMakeLists.txt` or `package.xml` files.

{: .important}
The `CMakeLists.txt` and `package.xml` files are what tells colcon how to build and compile this package.

The `package.xml` file lists all the library imports this package needs, as well as telling colcon what its build type even is and defining the package name under the hood. The `CMakeLists.txt` defines how to compile the package, where to find the source files, and where to put the compiled files.

#### package.xml for CMake

When you first create a package, the boilerplate that is generated is as follows:

```xml
<?xml version="1.0"?>
<?xml-model
   href="http://download.ros.org/schema/package_format3.xsd"
   schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
 <name>package_name</name>
 <version>0.0.0</version>
 <description>TODO: Package description</description>
 <maintainer email="user@todo.todo">user</maintainer>
 <license>Apache-2.0</license>

 <buildtool_depend>ament_cmake</buildtool_depend>

 <test_depend>ament_lint_auto</test_depend>
 <test_depend>ament_lint_common</test_depend>

 <export>
   <build_type>ament_cmake</build_type>
 </export>
</package>
```

The `<name>package_name</name>` is what ROS2 will know the package by under the hood, NOT the folder name. It is good practice to make sure the folder name and the name declared in this xml match.

When you are editing the `package.xml`, you will likely be changing what the depends are and little else. There are a few types of dependencies you can add in. The below information was sourced from the [ROS2 DeepWiki](https://deepwiki.com/ros2/ros2_documentation/6.4-dependency-management).

- `<buildtool_depend>` are tools needed to build the package (you will always have ament_cmake here)
- `<build_depend>` are packages needed at build time only
- `<build_export_depend>` are packages needed to build against this package (you will likely never use this)
- `<exec_depend>` are packages needed at runtime only
- `<test_depend>` are packages needed for testing only
- `<depend>` is the all-encompassing tag that combines build, build_export, and exec

{: .note}
The dependencies you add in the `package.xml` will specifically be internal ROS2 dependencies. This includes other user-made packages. Any external to ROS2 dependencies will be declared in the `CMakeLists.txt` file. Some ROS2 dependencies need to be declared in both locations.

The export section just states that the build type for this package is ament_cmake.

{: .warning}
If you are using generative AI while coding in ROS2, it really likes to tell you that more things need to be exported in the `package.xml`. This is false, unless you're programming a really weird edge case, you should not need to add anything to the `package.xml` export.

#### CMakeLists.txt

It is recommended that you go read the official documentation for [Writing CMakeLists Files](https://cmake.org/cmake/help/book/mastering-cmake/chapter/Writing%20CMakeLists%20Files.html) first.

Then, read the official [ament_cmake ROS2 documentation](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html) to learn more of the specifics for ROS2.

### Python Packages

When you create your Python package, you'll end up with a file structure like this:

```
package_name\
   resource\
      package_name
   package_name\
      __init__.py
   test\
      test_copyright.py
      test_flake8.py
      test_pep257.py
   LICENSE
   package.xml
   setup.cfg
   setup.py
```

Basically everything just created is boilerplate that you will not change and cannot delete. The only thing you can safely delete if you so wish is the test\ folder, which contains all the scripts to test the code. It is recommended to make a new folder to put your code in, or several new folders, though you can store code in the package_name\ directory.

{: .warning}
Do not delete resource\ or package_name\. The resource folder is required for the ROS2 environment to detect the package. The package_name\ folder is required for Python to work properly as it contains the init file that mark it as a Python package, and if you don't have that, you can't import its modules.

ROS2's Python packages follow the standards for creating Python packages in general, which you can read at the official documentation for [Packaging Python Projects](https://packaging.python.org/en/latest/tutorials/packaging-projects/).

{: .note}
The only thing you will need to commonly edit is the `setup.py` file (which is sort of equivalent to a `CMakeLists.txt`).

#### package.xml for Python

When you first create a package, the boilerplate that is generated is as follows:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>package_name</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">ubuntu</maintainer>
  <license>Apache-2.0</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

The purpose of package.xml is to define the ROS2 dependencies that you need, as well as some other basic information about the package. If you would like to read more in depth, visit the [package.xml for CMake](#packagexml-for-cmake) section above as all the same information applies.

Really, you shouldn't need to edit the `package.xml` for a Python project. Not entirely sure why, but you just don't really need to. All of the packages that use Python in the Lunabotics team's competition code is still the basic boilerplate.

#### setup.py

This is the main file you will be editing while developing a Python package. It is all of the metadata for the package, as well as instructions for where to place the scripts/other files during the build process.

When you first generate a package, this is the boilerplate:

```python
from setuptools import find_packages, setup

package_name = 'package_name'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
```

For the purposes of ROS2 use, the only thing you need to change here (usually) is the data_files list. This is where you need to put all of the new directories or files you add into the project. If you do not add them here, they will not be findable by the rest of the ROS2 environment.

{: .warning}
By default, just adding in new packages like how the default ones are does NOT recurse through every file in that directory.

To add in recursive search, the easiest method is just to use the glob library and import it. Additionally, import os to have better directory code that won't break the second anything changes.

```python
import os   # NEW IMPORT
from glob import glob # NEW IMPORT
from setuptools import find_packages, setup

package_name = 'package_name'   # CHANGE ME

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "new_folder_1"), glob("new_folder_1/*")),
        (os.path.join("share", package_name, "new_folder_2"), glob("new_folder_1/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
```

You can repeat the above pattern forever for any new folders you add to the directory. This will not only add the folder to the ROS2 build, but also anything contained in it.

If you want to explore what you can do in `setup.py` in more detail, feel free to check out the [Packaging Python Projects](https://packaging.python.org/en/latest/tutorials/packaging-projects/) documentation.

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)
