import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
import pytest

from conftest import PACKAGE_DIR


def _load_launch_module() -> ModuleType:
    path = PACKAGE_DIR / 'launch' / 'robot_state_publisher.launch.py'
    spec = importlib.util.spec_from_file_location(
        'robot_rbvogui_common_robot_state_publisher_launch', path
    )
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('allow_substs', [False, True])
def test_launch_node_reads_urdf_and_preserves_node_arguments(
    allow_substs: bool, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    module = _load_launch_module()
    urdf_file = tmp_path / 'robot.urdf'
    params_file = tmp_path / 'params.yaml'
    urdf_file.write_text('<robot name="rbv0"/>', encoding='utf-8')
    params_file.write_text('/**:\n  ros__parameters: {}\n', encoding='utf-8')

    captured: dict[str, object] = {}

    def parameter_file(params_source: object, allow_substs: bool) -> dict[str, object]:
        captured['parameter_file'] = params_source
        captured['parameter_file_allow_substs'] = allow_substs
        return {}

    def parameter_value(value: object, value_type: type) -> object:
        if value_type is str:
            captured['robot_description'] = value
            captured['robot_description_type'] = value_type
        return value

    class FakeNode:
        def __init__(self, **kwargs: object) -> None:
            captured['node_arguments'] = kwargs

    monkeypatch.setattr(module, 'ParameterFile', parameter_file)
    monkeypatch.setattr(module, 'ParameterValue', parameter_value)
    monkeypatch.setattr(module, 'Node', FakeNode)

    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {
            'robot_namespace': '/adapta/rbv0',
            'robot_rsp_params_file': str(params_file),
            'robot_rsp_params_file_allow_substs': str(allow_substs),
            'robot_urdf_file': str(urdf_file),
            'use_sim_time': 'False',
            'node_args': '{"name":"rsp","remappings":[["robot_description","~/model"]]}',
        }
    )

    actions = module._launch_node(ctx)

    assert len(actions) == 1
    assert captured['parameter_file_allow_substs'] is allow_substs
    assert isinstance(captured['parameter_file'], LaunchConfiguration)
    assert captured['parameter_file'].perform(ctx) == str(params_file)
    assert captured['robot_description'] == '<robot name="rbv0"/>'
    assert captured['robot_description_type'] is str

    node_arguments = captured['node_arguments']
    assert isinstance(node_arguments, dict)
    assert node_arguments['name'] == 'rsp'
    assert node_arguments['remappings'] == [('robot_description', '~/model')]


def test_launch_node_requires_readable_urdf_file(tmp_path: Path) -> None:
    module = _load_launch_module()
    params_file = tmp_path / 'params.yaml'
    params_file.write_text('/**:\n  ros__parameters: {}\n', encoding='utf-8')

    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {
            'robot_rsp_params_file': str(params_file),
            'robot_rsp_params_file_allow_substs': 'False',
            'robot_urdf_file': str(tmp_path / 'missing.urdf'),
            'use_sim_time': 'False',
            'node_args': '{}',
        }
    )

    with pytest.raises(FileNotFoundError):
        module._launch_node(ctx)
