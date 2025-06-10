# AgileX Scout Navigation with Nav2 and Simulation with Ignition Gazebo v6 (Fortress)

This repository provides a configuration for the navigation of the AgileX Scout robot with NAV2 and a 
simulation environment for the robot in Ignition Gazebo v6 (Fortress). The robot is a skid-steering mobile robot
with a 3D lidar sensor mounted on top. The simulation environment is based on the AWS RoboMaker Small Warehouse World,
which is a Gazebo world well suited for testing robot navigation in both cluttered and open spaces.

## Description

This repository contains the configuration files and the launch files for performing autonomous navigation with
the AgileX Scout robot using Nav2. It also contains the configuration files for the simulation of the robot in Ignition Gazebo v6 (Fortress).

All parameters used for the Navigation stack are collected in several YAML configuration files, inside the `rm_navigation/params` folder.
They have been carefully tuned to work as well as possible with the AgileX Scout robot and the simulation environment.
Despite this, it is still recommended to consider the specific characteristics of the environment in which the robot will operate
and to keep an eye on the robot during operation for safety reasons.

## Installation

### Dependencies

The following dependencies are required to run the simulation and the navigation stack:
- `navigation2`
- `pointcloud-to-laserscan`
- `slam-toolbox`
- `ros-gzfortress`
- `ignition-gazebo6`
- `spatio-temporal-voxel-layer`

The dependencies can be installed using the following command:

```bash
sudo apt install ros-humble-<dependency-name>
```

### Building

The compilation of the packages is required to run the simulation and the navigation stack. The packages can be compiled 
using the following command:

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-clean-cache
```

## Usage in Simulation Environment with Gazebo

To launch the simulation of the robot in Gazebo, along with NAV2, and the waypoint publisher node run the following commands in separate terminals:

```bash
ros2 launch robot_description simulation.launch.py
```

```bash
chmod 0700  /run/user/1000/
ros2 launch rm_navigation rm_navigation.launch.py
```

```bash
ros2 run waypoint_navigation_pkg waypoint_navigation_node
```

```bash
ros2 service call /start_navigation std_srvs/srv/Trigger
```

The first command will launch the simulation of the robot in Gazebo, with the 2D lidar sensor mounted on top.
The second command will launch the navigation stack, performing SLAM using AMCL, and the RViz GUI.
The third and fourth commands spawn the waypoint publisher.

If Gazebo crashes due to OGRE exception in WSL, try launching it using software acceleration instead of hardware acceleration:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe
ign gazebo empty.sdf # test that it works
```
