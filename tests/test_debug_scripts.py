import os

from conftest import PACKAGE_DIR
from conftest import run_bash


def test_debug_script_is_executable_and_has_valid_bash_syntax() -> None:
    script_path = PACKAGE_DIR / 'scripts' / 'debug_model_base.sh'

    assert os.access(script_path, os.X_OK)

    result = run_bash(f'bash -n "{script_path}"')
    output = result.stdout + result.stderr

    assert result.returncode == 0, output
