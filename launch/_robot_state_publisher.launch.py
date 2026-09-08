from pathlib import Path
import shlex
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue
import ros2_launch_helpers as rlh

from robot_vog.model_utils import get_models


def generate_launch_description() -> LaunchDescription:
    # Launch arguments with no default value must be provided by the caller.
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='', description='Project namespace'),
            DeclareLaunchArgument(
                'robot_model', choices=get_models(), description='Robot model to publish'
            ),
            DeclareLaunchArgument('robot_name', description='The unique name for the robot'),
            DeclareLaunchArgument(
                'robot_rsp_params_file',
                description='Path to the complete robot parameter YAML file.',
            ),
            DeclareLaunchArgument(
                'robot_rsp_params_file_allow_substs',
                choices=['True', 'true', 'False', 'false'],
                description='Allow ROS launch substitutions in robot_rsp_params_file',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                choices=['True', 'true', 'False', 'false'],
                description='Use ROS time from /clock if true.',
            ),
            DeclareLaunchArgument(
                'robot_xacro_args_file',
                default_value='',
                description=(
                    'Path or URI to the YAML file with xacro arguments loaded from configuration.'
                ),
            ),
            DeclareLaunchArgument(
                'robot_sim_file',
                default_value='',
                description=(
                    'Optional simulation YAML. Empty means the xacro model is built without '
                    'simulation plugins.'
                ),
            ),
            DeclareLaunchArgument(
                'node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            rlh.RequireFile(path=LaunchConfiguration('robot_rsp_params_file')),
            # Insert `robot_type`, `robot_namespace` and `robot_prefix` into the launch context.
            # Their values can then be substituted in the parameter file if needed.
            SetLaunchConfiguration('robot_type', 'vog'),
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


def _build_xacro_command(
    xacro_file: str, robot_xacro_args_file: str, robot_sim_file: str
) -> list[Any]:
    """
    Build the xacro command list used to generate `robot_description`.

    The command passes namespace, robot_name, and robot_sim_file directly.
    It then appends the optional model arguments loaded from `robot_xacro_args_file`.
    """
    if not xacro_file:
        raise ValueError('xacro_file must be a non-empty string.')

    if not Path(xacro_file).is_file():
        raise FileNotFoundError(f"File '{xacro_file}' does not exist.")

    if robot_sim_file and not Path(robot_sim_file).is_file():
        raise FileNotFoundError(f"File '{robot_sim_file}' does not exist.")

    # Launch files own the runtime arguments namespace, robot_name, and sim_file. The YAML file
    # contains only model-description choices that are safe to change during development.
    # The rest of the xacro arguments are loaded from `robot_xacro_args_file`, which is an optional
    # YAML file.
    # They configure model details such as optional sensors, visuals, collisions, inertias, and
    # mesh choices.
    # These values may change during testing and development, so it is convenient to keep them
    # in a separate YAML file.
    robot_xacro_args = _load_robot_xacro_args(robot_xacro_args_file)

    cmd: list[Any] = [
        FindExecutable(name='xacro'),
        ' ',
        _quote_shell_token_if_needed(xacro_file),
        ' namespace:=',
        LaunchConfiguration('namespace'),
        ' robot_name:=',
        LaunchConfiguration('robot_name'),
        ' sim_file:=',
        _quote_shell_token_if_needed(robot_sim_file),
    ]

    # Add xacro arguments loaded from robot_xacro_args_file.
    # Quote the complete assignment so both its name and value form one safe shell token.
    for arg_name, arg_value in robot_xacro_args.items():
        if arg_value is None:
            arg_value = ''
        elif isinstance(arg_value, (bool, int, float, str)):
            arg_value = str(arg_value)
        else:
            raise TypeError(
                f"Robot xacro argument '{arg_name}' must be a YAML scalar, got "
                f'{type(arg_value).__name__}.'
            )

        cmd.extend([' ', _quote_shell_token_if_needed(f'{arg_name}:={arg_value}')])

    return cmd


def _launch_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    params_allow_substs = perform_typed_substitution(
        ctx,
        normalize_typed_substitution(
            LaunchConfiguration('robot_rsp_params_file_allow_substs'), bool
        ),
        bool,
    )

    robot_model = LaunchConfiguration('robot_model').perform(ctx)

    # Get the xacro file for the selected model.
    xacro_file = Path(get_package_share_directory('robot_vog')).joinpath(
        'urdf', 'models', f'model_{robot_model}.xacro'
    )

    # Get the robot xacro arguments file from the launch configuration, if present.
    # It is an optional YAML file that contains xacro arguments loaded from configuration.
    robot_xacro_args_file = LaunchConfiguration('robot_xacro_args_file').perform(ctx).strip()
    robot_sim_file = LaunchConfiguration('robot_sim_file').perform(ctx).strip()

    actions: list[LaunchDescriptionEntity] = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('robot_namespace'),
            parameters=[
                ParameterFile(
                    LaunchConfiguration('robot_rsp_params_file'), allow_substs=params_allow_substs
                ),
                {
                    'robot_description': ParameterValue(
                        Command(
                            _build_xacro_command(
                                str(xacro_file), robot_xacro_args_file, robot_sim_file
                            )
                        ),
                        value_type=str,
                    ),
                    # robot_description is always published on a topic.
                    'use_robot_description_topic': True,
                    # Link and joint names generated by xacro already include their frame prefixes.
                    'frame_prefix': '',
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    ),
                },
            ],
            **rlh.resolve_node_arguments(
                LaunchConfiguration('node_args').perform(ctx),
                default_arguments={'name': 'robot_state_publisher'},
                extra_rejected_arguments={'namespace'},
            ),
        )
    ]

    return actions


def _load_robot_xacro_args(robot_xacro_args_file: str) -> dict[str, Any]:
    """
    Load xacro arguments from the optional YAML configuration file.

    An empty `robot_xacro_args_file` means that no xacro arguments are loaded from YAML.

    This function does not catch exceptions raised by `rlh.read_yaml_file`.
    Resolution, filesystem, encoding, and YAML parsing errors propagate and fail the launch.
    See `ros2_launch_helpers.read_yaml_file` for the exact exception contract.

    :param robot_xacro_args_file: Path or URI to the YAML file with xacro arguments loaded from
        configuration.
    :return: Mapping from xacro argument name to xacro argument value.
    :raises TypeError: If the YAML top level is not a mapping or if a key is not a string.
    :raises ValueError: If the YAML file sets a launch-provided xacro argument.
    """
    if not robot_xacro_args_file:
        return {}

    resolved_robot_xacro_args_file, robot_xacro_args = rlh.read_yaml_file(robot_xacro_args_file)

    # rlh.read_yaml_file returns None when the YAML file contains only comments or whitespace.
    if robot_xacro_args is None:
        robot_xacro_args = {}

    if not isinstance(robot_xacro_args, dict):
        raise TypeError(
            f"Robot xacro args file '{resolved_robot_xacro_args_file}' must contain a YAML "
            f'mapping at the top level, got {type(robot_xacro_args).__name__}.'
        )

    for arg_name in robot_xacro_args:
        # Argument names must be strings because they are used as xacro argument names.
        if not isinstance(arg_name, str):
            raise TypeError(
                f"Robot xacro args file '{resolved_robot_xacro_args_file}' contains a "
                'non-string key '
                f'{arg_name!r} of type {type(arg_name).__name__}.'
            )
        # There are some xacro arguments that are always passed directly by the launch files and
        # must not be set in the YAML file.
        if arg_name in ('namespace', 'robot_name', 'sim_file'):
            raise ValueError(
                f"Robot xacro args file '{resolved_robot_xacro_args_file}' must not set the "
                f"argument '{arg_name}'"
            )

    return robot_xacro_args


def _quote_shell_token_if_needed(raw_token: str) -> str:
    """Quote one command token when the shell would otherwise split or reinterpret it."""
    quoted_token = shlex.quote(raw_token)
    return quoted_token if quoted_token != raw_token else raw_token
