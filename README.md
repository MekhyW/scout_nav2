# AgileX Scout Navigation with Nav2 and Simulation with Ignition Gazebo v6 (Fortress)

This repository provides a configuration for the navigation of the AgileX Scout robot with NAV2 and a 
simulation environment for the robot in Ignition Gazebo v6 (Fortress). The robot is a skid-steering mobile robot
with a 3D lidar sensor mounted on top. The navigation stack launcher is based on the package `nav2_bringup` from the
[Navigation2](https://navigation.ros.org/) stack. The simulation environment is based on the AWS RoboMaker Small Warehouse World,
which is a Gazebo world well suited for testing robot navigation in both cluttered and open spaces.

## Description

This repository contains the configuration files and the launch files for performing autonomous navigation with
the AgileX Scout robot using Nav2. It also contains the configuration files for the simulation of the robot in Ignition Gazebo v6 (Fortress).

All parameters used for the Navigation stack are collected in several YAML configuration files, inside the `scout_nav2/params` folder.
They have been carefully tuned to work as well as possible with the AgileX Scout robot and the simulation environment.
Despite this, it is still recommended to consider the specific characteristics of the environment in which the robot will operate
and to keep an eye on the robot during operation for safety reasons.

## Installation

### Dependencies

The dependencies can be installed using the following command:

```bash
$ sudo apt install ros-humble-navigation2 ros-humble-pointcloud-to-laserscan ros-humble-slam-toolbox ros-humble-ros-gz libignition-gazebo6-dev ros-humble-spatio-temporal-voxel-layer python3-colcon-common-extensions -y
```

### Building

The compilation of the packages is required to run the simulation and the navigation stack. The packages can be compiled 
using the following command:

```bash
$ source /opt/ros/humble/setup.bash
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
$ source install/setup.bash
```

## Usage

The robot can be tested in both simulated and real environments. The following steps are required to launch the robot in both environments.
Several parameters are provided in the launch files to customize the robot's behavior and the navigation stack.

The parameters for the navigation stack are collected in several YAML configuration files, inside `scout_nav2/params` folder.
The available configuration files are:
- `scout_amcl.yaml`: navigation parameters using AMCL for localization and SLAM toolbox for map creation, with real Scout robot
- `scout_slam_localization.yaml`: navigation parameters using SLAM toolbox for localization only, with real Scout robot
- `sim_lidar2d_amcl.yaml`: navigation parameters using AMCL for localization and SLAM toolbox for map creation, with simulated Scout robot and 2D lidar
- `sim_lidar3d_amcl.yaml`: navigation parameters using AMCL for localization and SLAM toolbox for map creation, with simulated Scout robot and 3D lidar
- `sim_slam_localization.yaml`: navigation parameters using SLAM toolbox for localization only, with simulated Scout robot and 3D lidar

The parameters used for the navigation are automatically selected based on the parameters provided in the launch file.

The maps created by the SLAM toolbox are saved in the `scout_nav2/maps` folder. Maps can have different formats:
- `pgm` and `yaml` files for 2D maps created using the SLAM toolbox and deserialized using the `map_server` node, so that the map can be
  used by the AMCL localization algorithm. The `pgm` file is a grayscale image of the map, while the `yaml` file contains the metadata of the map.
- `data` and `posegraph` files for 2D maps created using the SLAM toolbox and serialized using the `slam_toolbox` node, so that the map can be
  used by the SLAM toolbox algorithm for efficient localization. These files are binary files and can be visualized using the `slam_toolbox` node.

### Usage in Simulation Environment with Gazebo

To launch the simulation of the robot in Gazebo, along with NAV2, run the following command in separate terminals:

```bash
$ ros2 launch agilex_scout simulate_control_gazebo.launch.py lidar_type:=2d rviz:=true

$ ros2 launch scout_nav2 nav2.launch.py simulation:=true slam:=true localization:=amcl
```

The first command will launch the simulation of the robot in Gazebo, with the 3D lidar sensor mounted on top, and the RViz GUI (optional).
The second command will launch the navigation stack with the specified parameters.

Parameters for simulation launch file:
- `lidar_type`: 3d for a 3D lidar (pointcloud2), 2d for a 2D lidar (laserscan)
- `rviz`: true if launching rviz, false if launching only the gazebo simulation

Parameters:

- `slam`: `True` if you want to use SLAM toolbox for map creation, `False` if you want to do localization + navigation with the map already created
- `simulation`: `true` if running in simulation with gazebo, `false` if launching the real AgileX Scout robot with real sensors
- `localization`: choose the localization algorithm, among `amcl` and `slam_toolbox`

If the graphics rendering on Gazebo is not working/crashing, use software rendering:
```bash
$ export LIBGL_ALWAYS_SOFTWARE=1
$ export MESA_GL_VERSION_OVERRIDE=3.3
$ export GALLIUM_DRIVER=llvmpipe
$ ign gazebo empty.sdf # test if gazebo works
```

## Repository structure

The repository is organized in different packages:
1. `agilex_scout`: contains the URDF description of the AgileX Scout robot, the Gazebo simulation configuration description files, 
   and the launch files for the simulation and the real robot control.
2. `aws-robomaker-small-warehouse-world`: contains the Gazebo world files for the AWS RoboMaker Small Warehouse World.
3. `scout_nav2`: contains the configuration files and launch files for the navigation of the AgileX Scout robot with Nav2.
   - `launch` folder: contains the main launch file for the navigation stack.
   - `maps` folder: contains the maps created by the SLAM toolbox.
   - `params` folder: contains the YAML configuration files for the navigation stack.
   - `rviz` folder: contains the RViz configuration files for the visualization of the robot and the navigation stack operation.
4. `spatio-temporal-voxel-layer`: contains the spatio-temporal voxel layer plugin for the costmap.
5. `nav2_bringup_custom`: contains the custom version of the `nav2_bringup` package, which is a fork of the original package.
   The custom version is needed to allow the use of the `nav2_collision_monitor` node, which is not available in the original package, 
   and to allow the use of other custom plugins for the navigation stack.