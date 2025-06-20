from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import HasNodeParams, RewrittenYaml
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    lifecycle_nodes = ['map_saver']
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'localization_launch.py')
    configured_params = ParameterFile(RewrittenYaml(source_file=params_file, root_key=namespace, param_rewrites={}, convert_types=True), allow_substs=True)
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=os.path.join(slam_toolbox_dir, 'config', 'mapper_params_localization.yaml'), description='Full path to the ROS2 parameters file to use for all launched nodes')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation (Gazebo) clock if true')
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='True', description='Automatically startup the nav2 stack')
    declare_use_respawn_cmd = DeclareLaunchArgument('use_respawn', default_value='False', description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='info', description='log level')
    start_map_server = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[configured_params]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ])
    has_slam_toolbox_params = HasNodeParams(source_file=params_file, node_name='slam_toolbox')
    start_slam_toolbox_localization_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch_file), launch_arguments={'use_sim_time': use_sim_time}.items(), condition=UnlessCondition(has_slam_toolbox_params))
    start_slam_toolbox_localization_cmd_with_params = IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch_file), launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': params_file,}.items(), condition=IfCondition(has_slam_toolbox_params))
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(start_map_server)
    ld.add_action(start_slam_toolbox_localization_cmd)
    ld.add_action(start_slam_toolbox_localization_cmd_with_params)
    return ld
