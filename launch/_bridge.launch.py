from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """
    Build the internal Gazebo bridge launch description for one robot model.

    This launch file is meant to be composed by Gazebo simulation launch files, but can be run
    directly as well. When robot_bridge_params_file_allow_substs is true, the caller can pass the
    launch keys used by the parameter file as extra CLI arguments even if this launch file does not
    declare those keys.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='', description='Project namespace'),
            DeclareLaunchArgument('robot_name', description='The unique name for the robot'),
            DeclareLaunchArgument(
                'robot_bridge_params_file',
                description='Path to the complete robot parameter YAML file.',
            ),
            DeclareLaunchArgument(
                'robot_bridge_params_file_allow_substs',
                choices=['True', 'true', 'False', 'false'],
                description='Allow ROS launch substitutions in robot_bridge_params_file',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                choices=['True', 'true', 'False', 'false'],
                description='Use ROS time from /clock if true.',
            ),
            DeclareLaunchArgument(
                'robot_bridge_config_file',
                description='Path to the robot bridge configuration file.',
            ),
            DeclareLaunchArgument(
                'node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            rlh.RequireFile(path=LaunchConfiguration('robot_bridge_params_file')),
            rlh.RequireFile(path=LaunchConfiguration('robot_bridge_config_file')),
            # Insert `robot_type`, `robot_namespace` and `robot_prefix` into the launch context.
            # Their values can then be substituted in the parameter file if needed.
            SetLaunchConfiguration('robot_type', 'rbvogui'),
            rlh.SetRobotNamespace(
                namespace=LaunchConfiguration('namespace'),
                robot_name=LaunchConfiguration('robot_name'),
                output_context_key='robot_namespace',
            ),
            rlh.SetRobotPrefix(
                robot_name=LaunchConfiguration('robot_name'), output_context_key='robot_prefix'
            ),
            OpaqueFunction(function=_launch_node),
        ]
    )


def _launch_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    """
    Launch the bridge for the robot model.

    The bridge transfers topics between ROS and Gazebo.
    The caller decides whether the bridge node uses ROS time from /clock through `use_sim_time`.
    """
    params_allow_substs = perform_typed_substitution(
        ctx,
        normalize_typed_substitution(
            LaunchConfiguration('robot_bridge_params_file_allow_substs'), bool
        ),
        bool,
    )

    return [
        Node(
            package='ros_gz_bridge',
            executable='bridge_node',
            namespace=LaunchConfiguration('robot_namespace'),
            parameters=[
                ParameterFile(
                    LaunchConfiguration('robot_bridge_params_file'),
                    allow_substs=params_allow_substs,
                ),
                # `expand_gz_topic_names` is always true because Gazebo topics are expected to
                # include the robot namespace so multiple robots can run in the same simulation.
                # `override_frame_id` is set to an empty string because Gazebo plugins publish the
                # required frame_id.
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    ),
                    'config_file': ParameterValue(
                        LaunchConfiguration('robot_bridge_config_file'), value_type=str
                    ),
                    'expand_gz_topic_names': True,
                    'override_frame_id': '',
                },
            ],
            **rlh.resolve_node_arguments(
                LaunchConfiguration('node_args').perform(ctx),
                default_arguments={'name': 'ros_gz_bridge'},
                extra_rejected_arguments={'namespace'},
            ),
        )
    ]
