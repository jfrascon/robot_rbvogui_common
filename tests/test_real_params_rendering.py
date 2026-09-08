import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.actions import SetLaunchConfiguration
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


def _find_mapping_key(
    value: object, expected_key: str, path: tuple[str, ...] = ()
) -> list[tuple[str, ...]]:
    matches: list[tuple[str, ...]] = []

    if isinstance(value, dict):
        for key, child in value.items():
            child_path = (*path, str(key))
            if key == expected_key:
                matches.append(child_path)
            matches.extend(_find_mapping_key(child, expected_key, child_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            matches.extend(_find_mapping_key(child, expected_key, (*path, str(index))))

    return matches


@pytest.mark.parametrize('robot_model', ['base', 'forklift'])
def test_model_params_do_not_define_use_sim_time(robot_model: str) -> None:
    params_file = PACKAGE_DIR / 'config' / f'model_{robot_model}' / 'default_params.yaml'
    params = yaml.safe_load(params_file.read_text(encoding='utf-8'))

    assert not _find_mapping_key(params, 'use_sim_time')


@pytest.mark.parametrize(
    ('robot_model', 'expected_node_names'),
    [
        ('base', {'robot_state_publisher', 'ros_gz_bridge', 'four_swerve_kinematics'}),
        (
            'forklift',
            {
                'robot_state_publisher',
                'ros_gz_bridge',
                'joint_position_controller_server',
                'prismatic_joint_position_serial_driver',
                'four_swerve_kinematics',
            },
        ),
    ],
)
def test_model_params_select_the_default_node_names(
    robot_model: str, expected_node_names: set[str]
) -> None:
    params_file = PACKAGE_DIR / 'config' / f'model_{robot_model}' / 'default_params.yaml'
    params = yaml.safe_load(params_file.read_text(encoding='utf-8'))
    configured_node_names = {selector.removeprefix('/**/') for selector in params}

    assert configured_node_names == expected_node_names


@pytest.mark.parametrize(
    'launch_file', ['real_model_base.launch.py', 'real_model_forklift.launch.py']
)
@pytest.mark.parametrize('allow_substs', [False, True])
def test_real_launch_renders_shared_params_once(
    launch_file: str, allow_substs: bool, tmp_path: Path
) -> None:
    module = _load_launch_module(launch_file)
    source_path = tmp_path / 'source.yaml'
    source_path.write_text(
        '/**:\n  ros__parameters:\n    value: $(var robot_prefix)wheel\n', encoding='utf-8'
    )

    ctx = LaunchContext()
    ctx.launch_configurations['namespace'] = '/fleet'
    ctx.launch_configurations['robot_name'] = 'front'
    ctx.launch_configurations['robot_params_file'] = str(source_path)
    ctx.launch_configurations['robot_params_file_allow_substs'] = str(allow_substs)

    preparation_actions = [
        action
        for action in module.generate_launch_description().entities
        if isinstance(
            action,
            (
                rlh.SetRobotNamespace,
                rlh.SetRobotPrefix,
                SetLaunchConfiguration,
                rlh.RenderParamsFile,
            ),
        )
    ]
    assert sum(isinstance(action, rlh.RenderParamsFile) for action in preparation_actions) == 1

    for action in preparation_actions:
        action.visit(ctx)

    resolved_path = Path(ctx.launch_configurations['resolved_robot_params_file'])

    if allow_substs:
        try:
            assert resolved_path != source_path
            assert yaml.safe_load(resolved_path.read_text(encoding='utf-8')) == {
                '/**': {'ros__parameters': {'value': 'front_wheel'}}
            }
        finally:
            resolved_path.unlink(missing_ok=True)
    else:
        assert resolved_path == source_path
