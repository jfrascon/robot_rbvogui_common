import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
import pytest
import yaml

from conftest import PACKAGE_DIR


def _load_launch_module() -> ModuleType:
    path = PACKAGE_DIR / 'launch' / '_robot_state_publisher.launch.py'
    spec = importlib.util.spec_from_file_location('robot_vog_robot_state_publisher_launch', path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_xacro_command_accepts_file_uri_for_optional_arguments() -> None:
    module = _load_launch_module()
    xacro_file = PACKAGE_DIR / 'urdf' / 'models' / 'model_base.xacro'
    xacro_args_file = PACKAGE_DIR / 'config' / 'model_base' / 'default_xacro_args.yaml'

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


@pytest.mark.parametrize('allow_substs', [False, True])
def test_launch_node_delegates_params_rendering_to_parameter_file(
    allow_substs: bool, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    module = _load_launch_module()
    package_share = tmp_path / 'robot_vog'
    xacro_file = package_share / 'urdf' / 'models' / 'model_base.xacro'
    params_file = tmp_path / 'params.yaml'
    xacro_file.parent.mkdir(parents=True)
    xacro_file.touch()
    params_file.write_text('/**:\n  ros__parameters: {}\n', encoding='utf-8')

    captured: dict[str, object] = {}

    def parameter_file(params_source: object, allow_substs: bool) -> dict[str, object]:
        captured['parameter_file'] = params_source
        captured['parameter_file_allow_substs'] = allow_substs
        return {}

    def build_xacro_command(xacro_path: str, xacro_args_file: str, sim_file: str) -> list[str]:
        del xacro_path, xacro_args_file, sim_file
        return []

    class FakeNode:
        def __init__(self, **_kwargs: object) -> None:
            pass

    monkeypatch.setattr(
        module, 'get_package_share_directory', lambda _package_name: str(package_share)
    )
    monkeypatch.setattr(module, 'ParameterFile', parameter_file)
    monkeypatch.setattr(module, '_build_xacro_command', build_xacro_command)
    monkeypatch.setattr(module, 'Node', FakeNode)

    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {
            'robot_rsp_params_file': str(params_file),
            'robot_rsp_params_file_allow_substs': str(allow_substs),
            'robot_model': 'base',
            'robot_xacro_args_file': '',
            'robot_sim_file': '',
            'use_sim_time': 'False',
            'node_args': '{}',
        }
    )

    actions = module._launch_node(ctx)

    assert len(actions) == 1
    assert captured['parameter_file_allow_substs'] is allow_substs
    assert isinstance(captured['parameter_file'], LaunchConfiguration)
    assert captured['parameter_file'].perform(ctx) == str(params_file)
