from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """Launch required nodes for a real VOG robot with a forklift module."""
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='', description='Project namespace.'),
            DeclareLaunchArgument(
                'robot_name', default_value='vog', description='Unique robot name.'
            ),
            DeclareLaunchArgument(
                'robot_params_file',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('robot_vog'),
                        'config',
                        'model_forklift',
                        'default_params.yaml',
                    ]
                ),
                description='Complete robot parameter YAML file.',
            ),
            DeclareLaunchArgument(
                'robot_params_file_allow_substs',
                default_value='True',
                choices=['True', 'true', 'False', 'false'],
                description='Allow ROS launch substitutions in robot_params_file.',
            ),
            DeclareLaunchArgument(
                'robot_xacro_args_file',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('robot_vog'),
                        'config',
                        'model_forklift',
                        'default_xacro_args.yaml',
                    ]
                ),
                description='YAML file with model-description xacro arguments.',
            ),
            DeclareLaunchArgument(
                'robot_rsp_node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            DeclareLaunchArgument(
                'robot_kinematics_node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
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
            rlh.RequireFile(path=LaunchConfiguration('robot_params_file')),
            rlh.SetRobotNamespace(
                namespace=LaunchConfiguration('namespace'),
                robot_name=LaunchConfiguration('robot_name'),
                output_context_key='robot_namespace',
            ),
            rlh.SetRobotPrefix(
                robot_name=LaunchConfiguration('robot_name'), output_context_key='robot_prefix'
            ),
            # Keep the original path when substitutions are disabled.
            SetLaunchConfiguration(
                'resolved_robot_params_file', LaunchConfiguration('robot_params_file')
            ),
            # When enabled, render once and replace the shared path before any child launch starts.
            rlh.RenderParamsFile(
                params_file=LaunchConfiguration('robot_params_file'),
                output_context_key='resolved_robot_params_file',
                condition=IfCondition(LaunchConfiguration('robot_params_file_allow_substs')),
            ),
            _include_robot_state_publisher(),
            _include_kinematics(),
            _include_fork_control(),
        ]
    )


def _include_robot_state_publisher() -> IncludeLaunchDescription:
    """Include robot_state_publisher with the shared parameter file prepared by the parent."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('robot_vog'), 'launch', '_robot_state_publisher.launch.py']
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'robot_model': 'forklift',
            'robot_name': LaunchConfiguration('robot_name'),
            'robot_rsp_params_file': LaunchConfiguration('resolved_robot_params_file'),
            'robot_rsp_params_file_allow_substs': 'False',
            'use_sim_time': 'False',
            'robot_xacro_args_file': LaunchConfiguration('robot_xacro_args_file'),
            'robot_sim_file': '',
            'node_args': LaunchConfiguration('robot_rsp_node_args'),
        }.items(),
    )


def _include_kinematics() -> IncludeLaunchDescription:
    """Include the kinematics node with the shared parameter file prepared by the parent."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('robot_vog'), 'launch', '_ground_vehicle_kinematics.launch.py']
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'robot_name': LaunchConfiguration('robot_name'),
            'robot_params_file': LaunchConfiguration('resolved_robot_params_file'),
            'robot_params_file_allow_substs': 'False',
            'use_sim_time': 'False',
            'node_args': LaunchConfiguration('robot_kinematics_node_args'),
        }.items(),
    )


def _include_fork_control() -> IncludeLaunchDescription:
    """Include fork control with the shared parameter file prepared by the parent."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('robot_vog'), 'launch', '_fork_control.launch.py']
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'robot_name': LaunchConfiguration('robot_name'),
            'robot_params_file': LaunchConfiguration('resolved_robot_params_file'),
            'robot_params_file_allow_substs': 'False',
            'use_sim_time': 'False',
            'robot_fork_controller_node_args': LaunchConfiguration(
                'robot_fork_controller_node_args'
            ),
            'robot_fork_serial_node_args': LaunchConfiguration('robot_fork_serial_node_args'),
        }.items(),
    )
