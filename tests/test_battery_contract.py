import importlib.util
from types import ModuleType

from launch import LaunchContext
import pytest
import yaml

from conftest import PACKAGE_DIR


def test_default_simulation_enables_the_common_battery_plugin() -> None:
    simulation = yaml.safe_load(
        PACKAGE_DIR.joinpath('config', 'default_simulation.yaml').read_text(encoding='utf-8')
    )
    source = PACKAGE_DIR.joinpath('urdf', 'common.xacro').read_text(encoding='utf-8')

    assert simulation['battery'] == {
        'enabled': True,
        'voltage': 24.0,
        'capacity': 20.0,
        'power_load': 0.0,
        'fix_issue_225': True,
    }
    assert '<xacro:plugin_linear_battery' in source
    assert 'name="main_battery"' in source
    for field in (
        'open_circuit_voltage_constant_coef',
        'open_circuit_voltage_linear_coef',
        'initial_charge',
        'resistance',
        'smooth_current_tau',
        'invert_current_sign',
        'start_draining',
        'power_draining_topic',
        'stop_power_draining_topic',
        'enable_recharge',
        'charging_time',
        'recharge_by_topic',
    ):
        assert f"battery_sim_cfg.get('{field}', '')" in source


def test_bridge_uses_shared_battery_bridge_configuration(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_bridge_launch_module()

    assert module.create_battery_bridges.__module__ == 'robotics_description.bridge_configurations'
    captured: dict[str, object] = {}

    def battery_bridges(**kwargs: str) -> dict[str, object]:
        captured['battery_arguments'] = kwargs
        return {'bridge_names': ['battery_state']}

    class FakeNode:
        def __init__(self, **kwargs: object) -> None:
            captured['node_arguments'] = kwargs

    monkeypatch.setattr(module, 'create_battery_bridges', battery_bridges)
    monkeypatch.setattr(module, 'Node', FakeNode)

    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {
            'robot_name': 'rbv0',
            'robot_namespace': '/sim/rbv0',
            'robot_bridge_params_file': '/tmp/params.yaml',
            'robot_bridge_params_file_allow_substs': 'False',
            'robot_bridge_config_file': '/tmp/bridge.yaml',
            'use_sim_time': 'True',
            'node_args': '{}',
        }
    )

    module._launch_node(ctx)

    assert captured['battery_arguments'] == {
        'model_name': 'rbv0',
        'battery_name': 'main_battery',
        'battery_state_ros_topic': 'main_battery/state',
        'battery_recharge_start_ros_topic': 'main_battery/recharge/start',
        'battery_recharge_stop_ros_topic': 'main_battery/recharge/stop',
    }
    node_arguments = captured['node_arguments']
    assert isinstance(node_arguments, dict)
    assert node_arguments['parameters'][-1] == {'bridge_names': ['battery_state']}


def _load_bridge_launch_module() -> ModuleType:
    module_path = PACKAGE_DIR / 'launch' / 'bridge.launch.py'
    spec = importlib.util.spec_from_file_location(
        'robot_rbvogui_common_bridge_launch', module_path
    )

    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
