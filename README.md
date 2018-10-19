# Search-Based Motion Planning Library

## Overview

SMPL is a set of packages for implementing robotic motion planners using
heuristic search algorithms. Includes the following packages:

* `smpl` - A library of generic graph representations, heuristics, search algorithms, and many utilities for robotic motion planning.
* `smpl_test` - Example applications and test cases using SMPL
* `smpl_ros` - A ROS Interface to the planning algorithms available in SMPL
* `sbpl_kdl_robot_model` - An implementation of SMPL planning interfaces using the KDL library for kinematics.
* `sbpl_pr2_robot_model` - An implementation of SMPL planning interfaces for the PR2 and UBR1 robots, using their custom kinematics libraries.
* `sbpl_collision_checking` - A collision detection library using approximate sphere- and grid-based models for a robot and its environment. Implements the collision checking interface defined in the smpl package.
* `sbpl_collision_checking_test` - Example application and benchmarking tools for `sbpl_collision_checking`
* `smpl_moveit_interface` - Plugins to the MoveIt! motion planning framework for planning with SMPL
* `smpl_ompl_interface` - An implementation of ompl planning interfaces using SMPL algorithms
* `smpl_urdf_robot_model` - An implementation of SMPL planning interfaces using a URDF model and custom forward kinematics

## Building

The core `smpl` package is built with standard cmake conventions. It can be
built simultaneously within a catkin workspace using the `catkin_make_isolated`
or `catkin build` commands.

The other packages are standard catkin packages and can be built with catkin
build tools. These instructions assume you are building all of the SMPL
packages simultaneously within a catkin workspace *catkin_ws*.

Required system dependencies for each package can be installed using
[rosdep](wiki.ros.org/rosdep). System dependencies can be installed via:

```sh
rosdep install --from-paths smpl -i -y
```

where `smpl` is the path to the cloned repository, not the `smpl` package.

SMPL requires the latest SBPL to be installed. Because catkin prefers to find
packages in your current and parent workspaces before searching system
directories, you may need to remove any binary packages that provide SBPL, e.g.
ros-_distro_-sbpl, to prevent catkin from giving it higher priority.

```sh
git clone https://github.com/sbpl/sbpl
cd sbpl && mkdir build && cd build && cmake .. && make && sudo make install
```

Clone additional source dependencies:

```sh
git clone https://github.com/aurone/leatherman
```

Once all system and source dependencies are built and installed, (re)build your
catkin workspace:

```sh
cd catkin_ws
catkin_make [-j#]
```

if using standard catkin commands, or

```sh
catkin build
```

using the `catkin` tool from the `python-catkin-tools` package.

## Running

Coming "soon"
