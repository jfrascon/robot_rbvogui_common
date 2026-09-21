"""Render a robot Xacro entry point to a reusable URDF file."""

from pathlib import Path
import shlex
from tempfile import NamedTemporaryFile
from typing import Any

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """Declare the public Xacro-to-URDF launch interface used by RB-VOGUI variants."""
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='', description='Project namespace'),
            DeclareLaunchArgument('robot_name', description='The unique name for the robot'),
            DeclareLaunchArgument(
                'robot_xacro_file',
                description='Path to the Xacro file that generates the robot URDF.',
            ),
            DeclareLaunchArgument(
                'robot_xacro_args_file',
                default_value='',
                description=(
                    'Path or URI to the YAML file with model-description Xacro arguments.'
                ),
            ),
            DeclareLaunchArgument(
                'robot_sim_file',
                default_value='',
                description=(
                    'Optional simulation YAML. Empty renders the model without simulation plugins.'
                ),
            ),
            DeclareLaunchArgument(
                'robot_urdf_file', description='Path where the rendered robot URDF is written.'
            ),
            rlh.RequireFile(path=LaunchConfiguration('robot_xacro_file')),
            OpaqueFunction(function=_render_robot_urdf),
            # This check runs after the synchronous renderer and protects every following include.
            rlh.RequireFile(path=LaunchConfiguration('robot_urdf_file')),
        ]
    )


def _build_xacro_command(
    xacro_file: str, robot_xacro_args_file: str, robot_sim_file: str
) -> list[Any]:
    """Build the Xacro command with launch-owned runtime arguments and optional YAML arguments."""
    if not xacro_file:
        raise ValueError('xacro_file must be a non-empty string.')

    if not Path(xacro_file).is_file():
        raise FileNotFoundError(f"File '{xacro_file}' does not exist.")

    if robot_sim_file and not Path(robot_sim_file).is_file():
        raise FileNotFoundError(f"File '{robot_sim_file}' does not exist.")

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


def _render_robot_urdf(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Render Xacro into a sibling temporary file, then atomically replace the requested output."""
    robot_xacro_file = LaunchConfiguration('robot_xacro_file').perform(ctx).strip()
    robot_xacro_args_file = LaunchConfiguration('robot_xacro_args_file').perform(ctx).strip()
    robot_sim_file = LaunchConfiguration('robot_sim_file').perform(ctx).strip()
    robot_urdf_file = LaunchConfiguration('robot_urdf_file').perform(ctx).strip()

    if not robot_urdf_file:
        raise ValueError('robot_urdf_file must be a non-empty path.')

    output_path = Path(robot_urdf_file)
    if not output_path.parent.is_dir():
        raise FileNotFoundError(
            f"URDF output directory '{output_path.parent}' does not exist or is not a directory."
        )

    # Debug launchers reserve an empty path first. Remove it if rendering fails, but preserve an
    # existing non-empty file supplied by a direct caller.
    remove_empty_output_on_error = output_path.is_file() and output_path.stat().st_size == 0
    temporary_path: Path | None = None

    try:
        robot_urdf = Command(
            _build_xacro_command(robot_xacro_file, robot_xacro_args_file, robot_sim_file)
        ).perform(ctx)

        with NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            prefix=f'.{output_path.name}.',
            suffix='.tmp',
            dir=output_path.parent,
            delete=False,
        ) as temporary_file:
            temporary_file.write(robot_urdf)
            temporary_path = Path(temporary_file.name)

        temporary_path.replace(output_path)
    except Exception:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        if remove_empty_output_on_error:
            output_path.unlink(missing_ok=True)
        raise

    return [LogInfo(msg=f'Rendered robot URDF: {output_path}')]


def _load_robot_xacro_args(robot_xacro_args_file: str) -> dict[str, Any]:
    """Load optional YAML arguments while reserving launch-owned runtime arguments."""
    if not robot_xacro_args_file:
        return {}

    resolved_robot_xacro_args_file, robot_xacro_args = rlh.read_yaml_file(robot_xacro_args_file)

    if robot_xacro_args is None:
        robot_xacro_args = {}

    if not isinstance(robot_xacro_args, dict):
        raise TypeError(
            f"Robot xacro args file '{resolved_robot_xacro_args_file}' must contain a YAML "
            f'mapping at the top level, got {type(robot_xacro_args).__name__}.'
        )

    for arg_name in robot_xacro_args:
        if not isinstance(arg_name, str):
            raise TypeError(
                f"Robot xacro args file '{resolved_robot_xacro_args_file}' contains a "
                f'non-string key {arg_name!r} of type {type(arg_name).__name__}.'
            )
        if arg_name in ('namespace', 'robot_name', 'sim_file'):
            raise ValueError(
                f"Robot xacro args file '{resolved_robot_xacro_args_file}' must not set the "
                f"argument '{arg_name}'"
            )

    return robot_xacro_args


def _quote_shell_token_if_needed(raw_token: str) -> str:
    """Quote one command token when the shell would otherwise reinterpret it."""
    quoted_token = shlex.quote(raw_token)
    return quoted_token if quoted_token != raw_token else raw_token
