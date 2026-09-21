import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
import pytest
import yaml

from conftest import PACKAGE_DIR


def _load_launch_module() -> ModuleType:
    path = PACKAGE_DIR / 'launch' / 'render_robot_urdf.launch.py'
    spec = importlib.util.spec_from_file_location('robot_rbvogui_common_render_urdf_launch', path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_xacro_command_accepts_file_uri_for_optional_arguments() -> None:
    module = _load_launch_module()
    xacro_file = PACKAGE_DIR / 'urdf' / 'model_base.xacro'
    xacro_args_file = PACKAGE_DIR / 'config' / 'default_xacro_args.yaml'

    command = module._build_xacro_command(str(xacro_file), xacro_args_file.as_uri(), '')

    assert any('body_color:=' in token for token in command if isinstance(token, str))


def test_quote_shell_token_uses_shell_quoting_when_needed() -> None:
    module = _load_launch_module()

    assert module._quote_shell_token_if_needed('plain_value') == 'plain_value'
    assert module._quote_shell_token_if_needed('') == "''"
    assert module._quote_shell_token_if_needed('value with spaces') == "'value with spaces'"
    assert module._quote_shell_token_if_needed('value "with quotes"') == '\'value "with quotes"\''


def test_build_xacro_command_quotes_paths_and_complete_yaml_assignments(tmp_path: Path) -> None:
    module = _load_launch_module()
    xacro_file = tmp_path / 'model files' / 'model_base.xacro'
    xacro_args_file = tmp_path / 'model args.yaml'
    xacro_file.parent.mkdir()
    xacro_file.touch()
    xacro_args_file.write_text(
        yaml.safe_dump({'custom arg': 'value with spaces'}), encoding='utf-8'
    )

    command = module._build_xacro_command(str(xacro_file), str(xacro_args_file), '')

    assert f"'{xacro_file}'" in command
    assert "'custom arg:=value with spaces'" in command


@pytest.mark.parametrize('runtime_argument', ['namespace', 'robot_name', 'sim_file'])
def test_xacro_args_file_rejects_launch_owned_argument(
    runtime_argument: str, tmp_path: Path
) -> None:
    module = _load_launch_module()
    args_file = tmp_path / 'xacro_args.yaml'
    args_file.write_text(yaml.safe_dump({runtime_argument: 'invalid'}), encoding='utf-8')

    with pytest.raises(ValueError, match=runtime_argument):
        module._load_robot_xacro_args(str(args_file))


def test_xacro_args_file_requires_a_top_level_mapping(tmp_path: Path) -> None:
    module = _load_launch_module()
    args_file = tmp_path / 'xacro_args.yaml'
    args_file.write_text('- invalid\n', encoding='utf-8')

    with pytest.raises(TypeError, match='mapping at the top level'):
        module._load_robot_xacro_args(str(args_file))


def test_render_robot_urdf_replaces_output_atomically(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    module = _load_launch_module()
    xacro_file = tmp_path / 'robot.xacro'
    output_file = tmp_path / 'robot.urdf'
    xacro_file.touch()
    output_file.touch()

    class FakeCommand:
        def __init__(self, _command: object) -> None:
            pass

        def perform(self, _ctx: LaunchContext) -> str:
            return '<robot name="rbv0"/>'

    monkeypatch.setattr(module, 'Command', FakeCommand)

    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {
            'namespace': '/adapta',
            'robot_name': 'rbv0',
            'robot_xacro_file': str(xacro_file),
            'robot_xacro_args_file': '',
            'robot_sim_file': '',
            'robot_urdf_file': str(output_file),
        }
    )

    actions = module._render_robot_urdf(ctx)

    assert len(actions) == 1
    assert output_file.read_text(encoding='utf-8') == '<robot name="rbv0"/>'
    assert list(tmp_path.glob('*.tmp')) == []


def test_render_failure_removes_precreated_empty_output(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    module = _load_launch_module()
    xacro_file = tmp_path / 'robot.xacro'
    output_file = tmp_path / 'robot.urdf'
    xacro_file.touch()
    output_file.touch()

    class FailingCommand:
        def __init__(self, _command: object) -> None:
            pass

        def perform(self, _ctx: LaunchContext) -> str:
            raise RuntimeError('xacro failed')

    monkeypatch.setattr(module, 'Command', FailingCommand)

    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {
            'namespace': '/adapta',
            'robot_name': 'rbv0',
            'robot_xacro_file': str(xacro_file),
            'robot_xacro_args_file': '',
            'robot_sim_file': '',
            'robot_urdf_file': str(output_file),
        }
    )

    with pytest.raises(RuntimeError, match='xacro failed'):
        module._render_robot_urdf(ctx)

    assert not output_file.exists()
