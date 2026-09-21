import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import pytest

EXPECTED_RESOURCES = (
    'LICENSE',
    'README.md',
    'config/model_base/default_bridge.yaml',
    'config/model_base/default_params.yaml',
    'config/model_base/default_simulation.yaml',
    'config/model_base/default_xacro_args.yaml',
    'launch/_bridge.launch.py',
    'launch/_ground_vehicle_kinematics.launch.py',
    'launch/_robot_state_publisher.launch.py',
    'launch/debug_model_base.launch.py',
    'rviz/sim_debug.rviz',
    'scripts/debug_model_base_with_defaults.sh',
    'urdf/common.xacro',
    'urdf/model_base.xacro',
    'worlds/debug_world.sdf',
    'worlds/debug_world_bridge.yaml',
)

REMOVED_RESOURCES = (
    'config/model_base/example_bridge.yaml',
    'config/model_base/example_params.yaml',
    'config/model_base/example_simulation.yaml',
    'config/model_forklift',
    'launch/_rsp.launch.py',
    'launch/_fork_control.launch.py',
    'launch/model_base.launch.py',
    'launch/model_forklift.launch.py',
    'launch/debug_model_forklift.launch.py',
    'launch/real_model_base.launch.py',
    'launch/real_model_forklift.launch.py',
    'scripts/debug_model_forklift_with_defaults.sh',
    'urdf/model_forklift.xacro',
)


@pytest.fixture(scope='module')
def package_share() -> Path:
    return Path(get_package_share_directory('robot_rbvogui_common'))


@pytest.mark.parametrize('relative_path', EXPECTED_RESOURCES)
def test_required_resource_is_installed(package_share: Path, relative_path: str) -> None:
    assert package_share.joinpath(relative_path).is_file()


@pytest.mark.parametrize('relative_path', REMOVED_RESOURCES)
def test_removed_resource_is_not_installed(package_share: Path, relative_path: str) -> None:
    assert not package_share.joinpath(relative_path).exists()


@pytest.mark.parametrize('relative_path', ['scripts/debug_model_base_with_defaults.sh'])
def test_installed_debug_script_is_executable(package_share: Path, relative_path: str) -> None:
    assert os.access(package_share / relative_path, os.X_OK)
