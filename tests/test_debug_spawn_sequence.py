import importlib.util
from types import ModuleType
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
import pytest

from conftest import PACKAGE_DIR


def _load_launch_module(filename: str) -> ModuleType:
    path = PACKAGE_DIR / 'launch' / filename
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    'launch_file', ['debug_model_base.launch.py', 'debug_model_forklift.launch.py']
)
def test_spawn_process_registers_exit_handler_before_starting(launch_file: str) -> None:
    module = _load_launch_module(launch_file)
    ctx = LaunchContext()
    ctx.launch_configurations['namespace'] = '/sim_debug'
    ctx.launch_configurations['robot_name'] = 'rbvogui'
    ctx.launch_configurations['robot_description_topic'] = 'robot_description'

    actions = module._launch_spawn_sequence(ctx, after_spawn_actions=[])

    assert isinstance(actions[0], RegisterEventHandler)
    assert isinstance(actions[1], LogInfo)
    assert isinstance(actions[2], Node)


@pytest.mark.parametrize(
    'launch_file', ['debug_model_base.launch.py', 'debug_model_forklift.launch.py']
)
def test_successful_spawn_returns_the_actions_declared_by_generate(launch_file: str) -> None:
    module = _load_launch_module(launch_file)
    expected_actions = [object(), object()]

    actions = module._launch_after_spawn(
        SimpleNamespace(returncode=0), LaunchContext(), after_spawn_actions=expected_actions
    )

    assert actions == expected_actions


@pytest.mark.parametrize(
    'launch_file', ['debug_model_base.launch.py', 'debug_model_forklift.launch.py']
)
def test_failed_spawn_reports_error_and_shuts_down(
    launch_file: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _load_launch_module(launch_file)
    errors: list[str] = []

    class FakeLogger:
        def error(self, message: str) -> None:
            errors.append(message)

    monkeypatch.setattr(module, 'get_logger', lambda _name: FakeLogger())

    actions = module._launch_after_spawn(
        SimpleNamespace(returncode=1), LaunchContext(), after_spawn_actions=[object()]
    )

    assert errors == ['Gazebo model spawn failed with return code 1.']
    assert len(actions) == 1
    assert isinstance(actions[0], EmitEvent)
