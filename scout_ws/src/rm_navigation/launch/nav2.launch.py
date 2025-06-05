import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    slam_arg = DeclareLaunchArgument(
        name="slam",
        default_value="False",
        description="Launch SLAM or launch localization and navigation",
        choices=["True", "False", "true", "false"],
    )

    localization_arg = DeclareLaunchArgument(
        name="localization",
        default_value="slam_toolbox",
        description="Launch localization with AMCL algorithm or SLAM toolbox for localization only",
        choices=["amcl", "slam_toolbox"],
    )

    simulation_arg = DeclareLaunchArgument(
        name="simulation",
        default_value="true",
        description="Launch simulation with gazebo or launch real robot navigation",
        choices=["True", "False", "true", "false"],
    )

    return LaunchDescription(
        [
            slam_arg,
            simulation_arg,
            localization_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):
    rm_localization_dir = get_package_share_directory("rm_localization_custom")
    nav2_launch_file = os.path.join(rm_localization_dir, "launch", "bringup_launch.py")
    rm_navigation_dir = get_package_share_directory("rm_navigation")

    map_file = "warehouse/map_slam_v2.yaml"
    use_sim_time = "true"
    params_file_name = "sim_lidar2d_amcl.yaml"

    # NAV2 parameters file path
    nav2_params_file = os.path.join(rm_navigation_dir, "params", params_file_name)
    # map file path for localization with AMCL
    map_yaml_file = os.path.join(rm_navigation_dir, "maps", map_file)

    # if slam is enabled --> slam + localization + navigation
    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,  # full configuration parameters
            "slam": "True",  # slam activated
            "map": "",  # no map,
            'use_collision_monitor': 'false',
        }.items(),
        condition=IfCondition(PythonExpression(["'" , LaunchConfiguration("slam"), "' == 'true'"]))
    )

    # if slam is disabled --> localization + navigation
    nav2_localization_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,  # full configuration parameters
            # localization + navigation
            "slam": "False",
            "map": map_yaml_file,  # map file
            "localization": LaunchConfiguration("localization")  # amcl or slam_toolbox
        }.items(),
        #condition=IfCondition(PythonExpression([LaunchConfiguration("slam"), " == False"])),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("slam") , "' == 'false'"]))
    )

    # static transform from world to map
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--yaw",
            "0.0",
            "--pitch",
            "0.0",
            "--roll",
            "0.0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "map",
        ],
    )

    # launch rviz
    rviz_default_config_file = os.path.join(rm_navigation_dir, "rviz", "nav2.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_default_config_file],
    )

    return [nav2_slam_launch, nav2_localization_navigation_launch, rviz_node, static_tf]
