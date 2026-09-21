import importlib.util
import json
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import ros2_launch_helpers as rlh
import yaml

from conftest import PACKAGE_DIR


def _load_launch_module(filename: str) -> ModuleType:
    path = PACKAGE_DIR / 'launch' / filename
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _capture_rsp_node_arguments(
    module: ModuleType,
    robot_description_topic: str,
    raw_node_arguments: dict[str, object],
    monkeypatch: pytest.MonkeyPatch,
) -> dict[str, object]:
    captured_launch_arguments: dict[str, object] = {}

    def include_launch(*_args: object, launch_arguments: object, **_kwargs: object) -> object:
        captured_launch_arguments.update(dict(launch_arguments))
        return object()

    monkeypatch.setattr(module, 'IncludeLaunchDescription', include_launch)

    ctx = LaunchContext()
    ctx.launch_configurations['robot_description_topic'] = robot_description_topic
    ctx.launch_configurations['robot_rsp_node_args'] = json.dumps(raw_node_arguments)

    actions = module._include_robot_state_publisher(ctx)

    assert len(actions) == 1
    return json.loads(str(captured_launch_arguments['node_args']))


@pytest.mark.parametrize(
    'launch_file', ['debug_model_base.launch.py', 'debug_model_forklift.launch.py']
)
def test_debug_topic_remapping_precedes_every_user_remapping_mechanism(
    launch_file: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _load_launch_module(launch_file)
    node_arguments = _capture_rsp_node_arguments(
        module,
        'rdesc',
        {
            'arguments': ['--ros-args', '--remap', 'robot_description:=from_user_arguments'],
            'ros_arguments': ['--remap', 'robot_description:=from_ros_arguments'],
            'remappings': [['robot_description', 'from_remappings']],
        },
        monkeypatch,
    )

    assert node_arguments['arguments'] == [
        '--ros-args',
        '--remap',
        'robot_description:=rdesc',
        '--',
        '--ros-args',
        '--remap',
        'robot_description:=from_user_arguments',
    ]
    assert node_arguments['ros_arguments'] == ['--remap', 'robot_description:=from_ros_arguments']
    assert node_arguments['remappings'] == [['robot_description', 'from_remappings']]

    resolved_arguments = rlh.resolve_node_arguments(json.dumps(node_arguments))
    node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', **resolved_arguments
    )
    ctx = LaunchContext()
    node._perform_substitutions(ctx)
    command = [perform_substitutions(ctx, part) for part in node.cmd]

    assert command[1:] == [
        '--ros-args',
        '--remap',
        'robot_description:=rdesc',
        '--',
        '--ros-args',
        '--remap',
        'robot_description:=from_user_arguments',
        '--ros-args',
        '--remap',
        'robot_description:=from_ros_arguments',
        '--ros-args',
        '-r',
        'robot_description:=from_remappings',
    ]


@pytest.mark.parametrize(
    'launch_file', ['debug_model_base.launch.py', 'debug_model_forklift.launch.py']
)
@pytest.mark.parametrize(
    ('configured_topic', 'expected_topic'),
    [
        ('robot_description', '/sim_debug/rbvogui/robot_description'),
        ('rdesc', '/sim_debug/rbvogui/rdesc'),
        ('nested/rdesc', '/sim_debug/rbvogui/nested/rdesc'),
        ('/rdesc', '/rdesc'),
    ],
)
def test_spawn_uses_the_resolved_debug_robot_description_topic(
    launch_file: str, configured_topic: str, expected_topic: str
) -> None:
    module = _load_launch_module(launch_file)
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {
            'namespace': '/sim_debug',
            'robot_name': 'rbvogui',
            'robot_description_topic': configured_topic,
            'world_name': 'debug_world',
        }
    )

    spawn_model = module._launch_spawn_sequence(ctx, after_spawn_actions=[])[2]
    spawn_model._perform_substitutions(ctx)
    params_file = Path(spawn_model.__dict__['_Node__expanded_parameter_arguments'][0][0])

    try:
        params = yaml.safe_load(params_file.read_text(encoding='utf-8'))
        assert params['/**']['ros__parameters']['topic'] == expected_topic
    finally:
        params_file.unlink(missing_ok=True)


@pytest.mark.parametrize(
    'launch_file', ['debug_model_base.launch.py', 'debug_model_forklift.launch.py']
)
@pytest.mark.parametrize(
    'invalid_topic',
    [
        '',
        '/',
        '~private',
        '~/private',
        '{node}/rdesc',
        '{namespace}/rdesc',
        'invalid//topic',
        'rdesc/',
        '/rdesc/',
    ],
)
def test_debug_robot_description_topic_rejects_invalid_shared_names(
    launch_file: str, invalid_topic: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _load_launch_module(launch_file)
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {
            'namespace': '/sim_debug',
            'robot_name': 'rbvogui',
            'robot_description_topic': invalid_topic,
            'robot_rsp_node_args': '{}',
        }
    )

    with pytest.raises(ValueError):
        module._include_robot_state_publisher(ctx)

    with pytest.raises(ValueError):
        module._launch_spawn_sequence(ctx, after_spawn_actions=[])


@pytest.mark.parametrize(
    'launch_file', ['debug_model_base.launch.py', 'debug_model_forklift.launch.py']
)
@pytest.mark.parametrize('valid_topic', ['robot_description', 'nested/rdesc', '/rdesc'])
def test_debug_robot_description_topic_validator_has_no_return_value(
    launch_file: str, valid_topic: str
) -> None:
    module = _load_launch_module(launch_file)

    assert module._validate_robot_description_topic(valid_topic) is None


@pytest.mark.parametrize(
    'launch_file', ['debug_model_base.launch.py', 'debug_model_forklift.launch.py']
)
@pytest.mark.parametrize(
    ('namespace', 'robot_name', 'configured_topic', 'expected_topic'),
    [
        ('', 'robot', 'rdesc', 'robot/rdesc'),
        ('fleet', 'robot', 'rdesc', 'fleet/robot/rdesc'),
        ('/', 'robot', 'rdesc', '/robot/rdesc'),
        ('/fleet', 'robot', 'rdesc', '/fleet/robot/rdesc'),
        ('fleet', 'robot', '/rdesc', '/rdesc'),
    ],
)
def test_make_robot_description_topic_builds_the_robot_namespace(
    launch_file: str, namespace: str, robot_name: str, configured_topic: str, expected_topic: str
) -> None:
    module = _load_launch_module(launch_file)

    assert (
        module._make_robot_description_topic(namespace, robot_name, configured_topic)
        == expected_topic
    )
