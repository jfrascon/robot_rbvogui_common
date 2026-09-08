from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """Declare the robot-scoped inputs used by the VOG fork control processes."""
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='', description='Project namespace.'),
            DeclareLaunchArgument(
                'robot_name', default_value='vog', description='Unique robot name.'
            ),
            DeclareLaunchArgument(
                'robot_params_file', description='Complete robot parameter YAML file.'
            ),
            DeclareLaunchArgument(
                'robot_params_file_allow_substs',
                choices=['True', 'true', 'False', 'false'],
                description='Allow ROS launch substitutions in robot_params_file.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                choices=['True', 'true', 'False', 'false'],
                description='Use ROS simulation time when true.',
            ),
            DeclareLaunchArgument(
                'robot_fork_controller_node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            DeclareLaunchArgument(
                'robot_fork_serial_node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            rlh.SetRobotNamespace(
                namespace=LaunchConfiguration('namespace'),
                robot_name=LaunchConfiguration('robot_name'),
                output_context_key='robot_namespace',
            ),
            rlh.SetRobotPrefix(
                robot_name=LaunchConfiguration('robot_name'), output_context_key='robot_prefix'
            ),
            rlh.RequireFile(path=LaunchConfiguration('robot_params_file')),
            _include_controller_server(),
            _include_serial_driver(),
        ]
    )


def _include_controller_server() -> IncludeLaunchDescription:
    """Start the fork action server with every child launch key set explicitly."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('joint_position_controller_server'),
                    'launch',
                    'joint_position_controller_server.launch.py',
                ]
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('robot_namespace'),
            'params_file': LaunchConfiguration('robot_params_file'),
            'params_file_allow_substs': LaunchConfiguration('robot_params_file_allow_substs'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'node_args': LaunchConfiguration('robot_fork_controller_node_args'),
        }.items(),
    )


def _include_serial_driver() -> IncludeLaunchDescription:
    """Start the physical fork driver only when the launch uses real time."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('joint_position_controller_server'),
                    'launch',
                    'prismatic_joint_position_serial_driver.launch.py',
                ]
            )
        ),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={
            'namespace': LaunchConfiguration('robot_namespace'),
            'params_file': LaunchConfiguration('robot_params_file'),
            'params_file_allow_substs': LaunchConfiguration('robot_params_file_allow_substs'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'node_args': LaunchConfiguration('robot_fork_serial_node_args'),
        }.items(),
    )
