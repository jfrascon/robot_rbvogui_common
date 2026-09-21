"""
Launch the RB-VOGUI base model in a package-local Gazebo debug world.

Use this launch file to inspect URDF/Xacro changes and Gazebo plugins without starting the complete
simulation stack.
"""

import json

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rclpy.exceptions import InvalidTopicNameException
import rclpy.validate_full_topic_name
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """Launch the RB-VOGUI base model in a Gazebo world for debugging and inspection."""
    actions: list[LaunchDescriptionEntity] = [
        SetLaunchConfiguration('namespace', '/sim_debug'),
        SetLaunchConfiguration('use_sim_time', 'True'),
        SetLaunchConfiguration('world_name', 'debug_world'),
        DeclareLaunchArgument(
            'robot_name', default_value='rbvogui', description='Unique robot name.'
        ),
        DeclareLaunchArgument(
            'robot_description_topic',
            default_value='robot_description',
            description=(
                'Topic shared by robot_state_publisher and the model spawner. '
                'Relative names resolve inside the robot namespace.'
            ),
        ),
        DeclareLaunchArgument(
            'robot_xacro_args_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_rbvogui_common'), 'config', 'default_xacro_args.yaml']
            ),
            description='YAML file with model-description xacro arguments.',
        ),
        DeclareLaunchArgument(
            'robot_sim_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_rbvogui_common'), 'config', 'default_simulation.yaml']
            ),
            description='Simulation YAML used while generating the robot description.',
        ),
        DeclareLaunchArgument(
            'robot_bridge_config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_rbvogui_common'), 'config', 'default_bridge.yaml']
            ),
            description='ROS-Gazebo bridge configuration for this model.',
        ),
        DeclareLaunchArgument(
            'robot_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_rbvogui_common'), 'config', 'default_params.yaml']
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
            'robot_rsp_node_args',
            default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
            description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
        ),
        DeclareLaunchArgument(
            'robot_bridge_node_args',
            default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
            description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
        ),
        DeclareLaunchArgument(
            'robot_kinematics_node_args',
            default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
            description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
        ),
        DeclareLaunchArgument(
            'rviz_enabled',
            default_value='True',
            choices=['True', 'true', 'False', 'false'],
            description='Launch RViz with the RB-VOGUI debug configuration.',
        ),
        DeclareLaunchArgument(
            'gzgui_enabled',
            default_value='True',
            choices=['True', 'true', 'False', 'false'],
            description='Launch the Gazebo graphical client.',
        ),
        rlh.RequireFile(path=LaunchConfiguration('robot_params_file')),
        rlh.RequireFile(path=LaunchConfiguration('robot_sim_file')),
        rlh.RequireFile(path=LaunchConfiguration('robot_bridge_config_file')),
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
    ]

    before_spawn_actions: list[LaunchDescriptionEntity] = [
        _include_spawn_world(),
        OpaqueFunction(function=_include_robot_state_publisher),
    ]

    spawn_and_wait_action = OpaqueFunction(
        function=_launch_spawn_sequence,
        kwargs={
            # These actions start only after Gazebo reports a successful model spawn.
            'after_spawn_actions': [
                # Keep model-dependent processes explicit and in launch order.
                _include_bridge(),
                _include_kinematics(),
                _launch_rviz(),
            ]
        },
    )

    actions.extend(before_spawn_actions)
    actions.append(spawn_and_wait_action)

    return LaunchDescription(actions)


def _include_spawn_world() -> IncludeLaunchDescription:
    """Start the shared Gazebo world used to inspect the base model."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_tools'), 'launch', 'spawn_world.launch.py']
            )
        ),
        launch_arguments={
            'gzserver_use_composition': 'False',
            'gzserver_create_own_container': 'False',
            'gzserver_container_name': '',
            'gzserver_initial_sim_time': '0.0',
            'gzserver_verbosity_level': '4',
            'gzgui_enabled': LaunchConfiguration('gzgui_enabled'),
            'gzgui_config_file': '',
            'world_sdf_file': PathJoinSubstitution(
                [FindPackageShare('robot_rbvogui_common'), 'worlds', 'debug_world.sdf']
            ),
            'world_sdf_string': '',
            'world_bridge_config_file': PathJoinSubstitution(
                [FindPackageShare('robot_rbvogui_common'), 'worlds', 'debug_world_bridge.yaml']
            ),
            'world_bridge_name': 'world_bridge',
            'world_bridge_subscription_heartbeat': '1000',
            'world_bridge_expand_gz_topic_names': 'True',
            'world_bridge_override_timestamps_with_wall_time': 'False',
            'world_bridge_override_frame_id': '',
            'world_bridge_use_respawn': 'False',
            'world_bridge_log_level': 'info',
        }.items(),
    )


def _include_robot_state_publisher(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Prepend the debug topic remapping and include robot_state_publisher."""
    robot_description_topic = LaunchConfiguration('robot_description_topic').perform(ctx).strip()
    _validate_robot_description_topic(robot_description_topic)

    node_arguments = rlh.resolve_node_arguments(
        LaunchConfiguration('robot_rsp_node_args').perform(ctx),
        extra_rejected_arguments={'namespace'},
    )

    # `node_args` can express remapping rules through three fields:
    # - `remappings` contains structured source and target pairs.
    # - `ros_arguments` contains ROS arguments such as `--remap source:=target`.
    # - `arguments` contains raw process arguments, which may include another `--ros-args` block.
    # Tests of the generated command and ROS 2 remapping confirm that rules from `arguments` are
    # evaluated before rules from `ros_arguments` and `remappings`.
    # ROS 2 uses the first matching rule, so the simulation-owned `robot_description` remapping is
    # prepended to the user-provided `arguments` list.
    user_arguments = node_arguments.pop('arguments', []) or []
    node_arguments['arguments'] = [
        '--ros-args',
        '--remap',
        f'robot_description:={robot_description_topic}',
        '--',
        *user_arguments,
    ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare('robot_rbvogui_common'),
                        'launch',
                        '_robot_state_publisher.launch.py',
                    ]
                )
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'robot_xacro_file': PathJoinSubstitution(
                    [FindPackageShare('robot_rbvogui_common'), 'urdf', 'model_base.xacro']
                ),
                'robot_name': LaunchConfiguration('robot_name'),
                'robot_rsp_params_file': LaunchConfiguration('resolved_robot_params_file'),
                'robot_rsp_params_file_allow_substs': 'False',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_xacro_args_file': LaunchConfiguration('robot_xacro_args_file'),
                'robot_sim_file': LaunchConfiguration('robot_sim_file'),
                'node_args': json.dumps(node_arguments),
            }.items(),
        )
    ]


def _launch_spawn_sequence(
    ctx: LaunchContext, *, after_spawn_actions: list[LaunchDescriptionEntity]
) -> list[LaunchDescriptionEntity]:
    """Spawn the model and register the actions that require a successful spawn."""
    robot_name = LaunchConfiguration('robot_name').perform(ctx)
    namespace = LaunchConfiguration('namespace').perform(ctx)
    configured_topic = LaunchConfiguration('robot_description_topic').perform(ctx).strip()
    robot_description_topic = _make_robot_description_topic(
        namespace, robot_name, configured_topic
    )
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[
            {
                'world': LaunchConfiguration('world_name'),
                'file': '',
                'string': '',
                'topic': robot_description_topic,
                'name': robot_name,
                'allow_renaming': False,
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'R': 0.0,
                'P': 0.0,
                'Y': 0.0,
            }
        ],
        ros_arguments=['--log-level', 'info'],
        output='screen',
    )

    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_model,
                on_exit=lambda event, context: _launch_after_spawn(
                    event, context, after_spawn_actions=after_spawn_actions
                ),
            )
        ),
        LogInfo(
            msg=[
                "Spawning model '",
                robot_name,
                "' into world '",
                LaunchConfiguration('world_name'),
                "'",
            ]
        ),
        spawn_model,
    ]


def _launch_after_spawn(
    event: ProcessExited,
    _ctx: LaunchContext,
    *,
    after_spawn_actions: list[LaunchDescriptionEntity],
) -> list[LaunchDescriptionEntity]:
    """Start model-dependent processes only after Gazebo finishes spawning the robot."""
    if event.returncode != 0:
        reason = f'Gazebo model spawn failed with return code {event.returncode}.'
        get_logger('robot_rbvogui_common').error(reason)
        return [EmitEvent(event=Shutdown(reason=reason))]

    return after_spawn_actions


def _include_bridge() -> IncludeLaunchDescription:
    """Start the model bridge after starting the Gazebo spawn process."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('robot_rbvogui_common'), 'launch', '_bridge.launch.py']
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'robot_name': LaunchConfiguration('robot_name'),
            'robot_bridge_params_file': LaunchConfiguration('resolved_robot_params_file'),
            'robot_bridge_params_file_allow_substs': 'False',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_bridge_config_file': LaunchConfiguration('robot_bridge_config_file'),
            'node_args': LaunchConfiguration('robot_bridge_node_args'),
        }.items(),
    )


def _include_kinematics() -> IncludeLaunchDescription:
    """Start the kinematics node after starting the model bridge."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('robot_rbvogui_common'),
                    'launch',
                    '_ground_vehicle_kinematics.launch.py',
                ]
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'robot_name': LaunchConfiguration('robot_name'),
            'robot_params_file': LaunchConfiguration('resolved_robot_params_file'),
            'robot_params_file_allow_substs': 'False',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'node_args': LaunchConfiguration('robot_kinematics_node_args'),
        }.items(),
    )


def _launch_rviz() -> Node:
    """Launch RViz with the package-local RB-VOGUI debug configuration."""
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=LaunchConfiguration('namespace'),
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare('robot_rbvogui_common'), 'rviz', 'sim_debug.rviz']
            ),
        ],
        output='both',
        condition=IfCondition(LaunchConfiguration('rviz_enabled')),
    )


def _make_robot_description_topic(namespace: str, robot_name: str, configured_topic: str) -> str:
    """
    Build the robot topic used by Gazebo from the parent namespace and robot name.

    ``make_robot_namespace`` validates ``robot_name`` and combines it with ``namespace``. An
    absolute configured topic ignores that robot namespace. A relative configured topic is appended
    without changing whether the resulting robot topic is relative or absolute.

    When the result is relative, a temporary ``/`` is added to a copy before calling the fully
    qualified ROS validator. The temporary value is never returned or passed to another node.
    """
    robot_namespace = rlh.make_robot_namespace(namespace, robot_name)
    _validate_robot_description_topic(configured_topic)

    if configured_topic.startswith('/'):
        resolved_topic = configured_topic
    elif robot_namespace in ('', '/'):
        resolved_topic = robot_namespace + configured_topic
    else:
        resolved_topic = f'{robot_namespace}/{configured_topic}'

    topic_to_validate = resolved_topic if resolved_topic.startswith('/') else f'/{resolved_topic}'

    try:
        rclpy.validate_full_topic_name.validate_full_topic_name(topic_to_validate)
    except InvalidTopicNameException as exc:
        raise ValueError(
            f'Resolved robot_description_topic is invalid: {resolved_topic!r}.'
        ) from exc

    return resolved_topic


def _validate_robot_description_topic(topic: str) -> None:
    """
    Validate one concrete relative or absolute topic shared by RSP and Gazebo spawn.

    A temporary ``/`` is added to a relative topic so the fully qualified ROS validator can check
    every name segment. This temporary value is only used for validation. It does not modify the
    configured topic and is never passed to ``robot_state_publisher``.
    """
    if '~' in topic or '{' in topic or '}' in topic:
        raise ValueError('robot_description_topic must be a concrete relative or absolute topic.')

    topic_to_validate = topic if topic.startswith('/') else f'/{topic}'

    try:
        rclpy.validate_full_topic_name.validate_full_topic_name(topic_to_validate)
    except InvalidTopicNameException as exc:
        raise ValueError(
            f'robot_description_topic must be a valid concrete ROS topic, got {topic!r}.'
        ) from exc
