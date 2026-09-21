from datetime import datetime
import importlib.util
import os
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
import pytest

from conftest import PACKAGE_DIR


def _load_launch_module() -> ModuleType:
    path = PACKAGE_DIR / 'launch' / 'debug_model_base.launch.py'
    spec = importlib.util.spec_from_file_location('robot_rbvogui_common_debug_launch', path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_robot_urdf_temp_file_uses_flattened_namespace(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_launch_module()

    class FixedDateTime:
        @classmethod
        def now(cls) -> datetime:
            return datetime(2026, 9, 18, 14, 35, 27)

    expected_path = Path('/tmp/adapta_rbv0_20260918_143527_mkstemp01.urdf')
    expected_path.unlink(missing_ok=True)
    captured: dict[str, str] = {}

    def create_temp_file(*, prefix: str, suffix: str, **kwargs: str) -> tuple[int, str]:
        directory = kwargs['dir']
        captured.update({'prefix': prefix, 'suffix': suffix, 'dir': directory})
        file_descriptor = os.open(expected_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        return file_descriptor, str(expected_path)

    monkeypatch.setattr(module, 'datetime', FixedDateTime)
    monkeypatch.setattr(module, 'mkstemp', create_temp_file)

    try:
        ctx = LaunchContext()
        ctx.launch_configurations['robot_namespace'] = '/adapta/rbv0'

        actions = module._set_robot_urdf_file(ctx)

        assert len(actions) == 1
        assert ctx.launch_configurations['robot_urdf_file'] == str(expected_path)
        assert expected_path.is_file()
        assert captured == {
            'prefix': 'adapta_rbv0_20260918_143527_',
            'suffix': '.urdf',
            'dir': '/tmp',
        }
    finally:
        expected_path.unlink(missing_ok=True)


def test_debug_spawn_reads_rendered_urdf_file() -> None:
    source = PACKAGE_DIR.joinpath('launch', 'debug_model_base.launch.py').read_text(
        encoding='utf-8'
    )

    assert "'file': LaunchConfiguration('robot_urdf_file')" in source
    assert "'string': ''" in source
    assert "'topic': ''" in source
    assert 'robot_description_topic' not in source


def test_debug_identity_matches_script_and_rviz_configuration() -> None:
    launch_source = PACKAGE_DIR.joinpath('launch', 'debug_model_base.launch.py').read_text(
        encoding='utf-8'
    )
    script_source = PACKAGE_DIR.joinpath('scripts', 'debug_model_base.sh').read_text(
        encoding='utf-8'
    )
    rviz_source = PACKAGE_DIR.joinpath('rviz', 'sim_debug.rviz').read_text(encoding='utf-8')

    assert "'robot_name', default_value='rbv0'" in launch_source
    assert 'robot_name:=rbv0' in script_source
    assert '/sim_debug/rbv0/robot_description' in rviz_source


def test_debug_waits_for_world_service_before_spawning_model() -> None:
    source = PACKAGE_DIR.joinpath('launch', 'debug_model_base.launch.py').read_text(
        encoding='utf-8'
    )

    assert "FindPackagePrefix('ros_gz_tools')" in source
    assert "'wait_for_gz_service.py'" in source
    assert "['/world/', LaunchConfiguration('world_name'), '/create']" in source
    assert 'target_action=wait_for_create_service' in source
    assert 'def _launch_actions_after_world_ready(' in source
    assert 'def _launch_actions_after_model_ready(' in source
    assert source.index('_include_spawn_world(),') < source.index('wait_for_create_service')
    assert source.index('_include_robot_state_publisher(),') > source.index(
        'def _launch_actions_after_model_ready('
    )
