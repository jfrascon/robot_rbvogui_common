from pathlib import Path
import re

import pytest
import yaml

from conftest import PACKAGE_DIR

RUNTIME_XACRO_ARGS = {'namespace', 'robot_name', 'sim_file'}


def _xacro_arg_defaults(path: Path) -> dict[str, str]:
    source = path.read_text(encoding='utf-8')
    return dict(re.findall(r'<xacro:arg\s+name="([^"]+)"\s+default="([^"]*)"', source))


def _normalize_scalar(value: object) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


@pytest.mark.parametrize('model_name', ['base'])
def test_default_model_xacro_args_match_the_xacro_contract(model_name: str) -> None:
    common_defaults = _xacro_arg_defaults(PACKAGE_DIR / 'urdf' / 'common.xacro')
    model_defaults = common_defaults

    expected_defaults = {
        name: value for name, value in model_defaults.items() if name not in RUNTIME_XACRO_ARGS
    }
    args_path = PACKAGE_DIR / 'config' / 'default_xacro_args.yaml'
    configured_defaults = yaml.safe_load(args_path.read_text(encoding='utf-8'))

    assert isinstance(configured_defaults, dict)
    assert set(configured_defaults) == set(expected_defaults)

    for name, expected in expected_defaults.items():
        assert _normalize_scalar(configured_defaults[name]).lower() == expected.lower()
