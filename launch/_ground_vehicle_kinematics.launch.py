from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """Declare the robot-scoped inputs used to start the VOG kinematics node."""
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
                'node_args',
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
            _include_kinematics(),
        ]
    )


def _include_kinematics() -> IncludeLaunchDescription:
    """Include the four-swerve kinematics launch with every input mapped explicitly."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ground_vehicle_kinematics'),
                    'launch',
                    'four_swerve_kinematics.launch.py',
                ]
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('robot_namespace'),
            'robot_prefix': LaunchConfiguration('robot_prefix'),
            'params_file': LaunchConfiguration('robot_params_file'),
            'params_file_allow_substs': LaunchConfiguration('robot_params_file_allow_substs'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'node_args': LaunchConfiguration('node_args'),
        }.items(),
    )
