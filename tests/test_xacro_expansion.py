from pathlib import Path
import re

import pytest
import yaml

from conftest import PACKAGE_DIR
from conftest import run_bash

WHEEL_NAMES_BY_KINEMATICS_INDEX = (
    'front_left_wheel',
    'front_right_wheel',
    'rear_left_wheel',
    'rear_right_wheel',
)
ACTIVE_XACRO_FILES = ('urdf/common.xacro', 'urdf/model_base.xacro', 'urdf/model_forklift.xacro')
ENABLED_SIMULATION_TOPIC_CASES = (
    ('base', 'base_velocity_controller', ('topic',)),
    ('base', 'pose_publisher', ('topic', 'topic_with_covariance', 'tf_topic')),
    ('base', 'front_left_wheel_steering_joint_controller', ('topic',)),
    ('base', 'front_right_wheel_steering_joint_controller', ('topic',)),
    ('base', 'rear_left_wheel_steering_joint_controller', ('topic',)),
    ('base', 'rear_right_wheel_steering_joint_controller', ('topic',)),
    ('base', 'front_left_wheel_rotation_joint_controller', ('topic',)),
    ('base', 'front_right_wheel_rotation_joint_controller', ('topic',)),
    ('base', 'rear_left_wheel_rotation_joint_controller', ('topic',)),
    ('base', 'rear_right_wheel_rotation_joint_controller', ('topic',)),
    ('base', 'joint_state_publisher', ('topic',)),
    ('forklift', 'fork_controller', ('topic',)),
    ('forklift', 'joint_state_publisher', ('topic',)),
)


@pytest.mark.parametrize('robot_model', ['base', 'forklift'])
def test_xacro_expands_to_valid_urdf(robot_model: str, tmp_path: Path) -> None:
    urdf_path = tmp_path / f'{robot_model}.urdf'
    xacro_path = PACKAGE_DIR / 'urdf' / f'model_{robot_model}.xacro'

    result = run_bash(f'xacro "{xacro_path}" > "{urdf_path}" && check_urdf "{urdf_path}"')

    output = result.stdout + result.stderr

    assert result.returncode == 0, output
    assert urdf_path.is_file(), output
    assert 'Successfully Parsed XML' in output, output

    urdf = urdf_path.read_text(encoding='utf-8')
    for wheel_name in WHEEL_NAMES_BY_KINEMATICS_INDEX:
        assert f'rbvogui_{wheel_name}_steering_joint' in urdf
        assert f'rbvogui_{wheel_name}_rotation_joint' in urdf

    assert 'steerable_wheel_0' not in urdf

    basket_joint = re.search(
        r'<joint name="rbvogui_basket_chassis_closed_livox_supports_joint"[\s\S]*?</joint>', urdf
    )
    fork_joint = re.search(r'<joint name="rbvogui_fork_root_joint"[\s\S]*?</joint>', urdf)
    front_top_lidar_joint = re.search(
        r'<joint name="rbvogui_front_top_lidar_root_joint"[\s\S]*?</joint>', urdf
    )
    back_top_lidar_joint = re.search(
        r'<joint name="rbvogui_back_top_lidar_root_joint"[\s\S]*?</joint>', urdf
    )
    if robot_model == 'base':
        assert basket_joint is None
        assert fork_joint is None
        assert front_top_lidar_joint is None
        assert back_top_lidar_joint is None
    else:
        assert basket_joint is not None
        assert '<origin rpy="0 0 0" xyz="0.027732 0 0.2445"/>' in basket_joint.group()
        assert '<parent link="rbvogui_chassis_link"/>' in basket_joint.group()
        assert front_top_lidar_joint is not None
        assert '<parent link="rbvogui_basket_root_link"/>' in front_top_lidar_joint.group()
        assert (
            '<origin rpy="0.0 0.5235987755982988 0.0" xyz="0.4882 0.0 0.1952"/>'
            in front_top_lidar_joint.group()
        )
        assert back_top_lidar_joint is not None
        assert '<parent link="rbvogui_basket_root_link"/>' in back_top_lidar_joint.group()
        assert (
            '<origin rpy="0.0 0.5235987755982988 3.141592653589793" '
            'xyz="-0.4844 0.0 0.1217"/>' in back_top_lidar_joint.group()
        )
        assert fork_joint is not None
        assert '<origin rpy="0 0 0" xyz="0.622 0 -0.0905"/>' in fork_joint.group()
        assert '<parent link="rbvogui_base_link"/>' in fork_joint.group()
        assert '<axis xyz="0 0 1"/>' in fork_joint.group()
        assert '<dynamics damping="10.0" friction="1.0"/>' in fork_joint.group()
        assert '<limit effort="20000.0" lower="-0.005" upper="0.2" velocity="0.04"/>' in (
            fork_joint.group()
        )
        assert 'rbvogui_fork_carriage_joint' not in urdf
        assert 'rbvogui_fork_camera_link' not in urdf


@pytest.mark.parametrize('robot_model', ['base', 'forklift'])
def test_simulation_xacro_expands_with_current_gazebo_apis(
    robot_model: str, tmp_path: Path
) -> None:
    urdf_path = tmp_path / f'{robot_model}_simulation.urdf'
    xacro_path = PACKAGE_DIR / 'urdf' / f'model_{robot_model}.xacro'
    sim_path = PACKAGE_DIR / 'config' / f'model_{robot_model}' / 'default_simulation.yaml'

    result = run_bash(
        f'xacro "{xacro_path}" sim_file:="{sim_path}" > "{urdf_path}" && check_urdf "{urdf_path}"'
    )
    output = result.stdout + result.stderr

    assert result.returncode == 0, output
    assert 'Successfully Parsed XML' in output, output

    urdf = urdf_path.read_text(encoding='utf-8')
    assert 'gz::sim::systems::OdometryPublisher' in urdf
    assert 'gz::sim::systems::JointStatePublisher' in urdf
    if robot_model == 'forklift':
        assert '<joint_name>rbvogui_fork_root_joint</joint_name>' in urdf
        assert '<use_velocity_commands>true</use_velocity_commands>' in urdf
        assert '<cmd_max>0.04</cmd_max>' in urdf
        assert '<p_gain>' not in urdf
        assert '<i_gain>' not in urdf
        assert '<d_gain>' not in urdf


@pytest.mark.parametrize('robot_model', ['base', 'forklift'])
def test_whitespace_only_sim_file_builds_real_description(
    robot_model: str, tmp_path: Path
) -> None:
    urdf_path = tmp_path / f'{robot_model}_whitespace_sim_file.urdf'
    xacro_path = PACKAGE_DIR / 'urdf' / f'model_{robot_model}.xacro'

    result = run_bash(
        f'xacro "{xacro_path}" sim_file:="   " > "{urdf_path}" && check_urdf "{urdf_path}"'
    )
    output = result.stdout + result.stderr

    assert result.returncode == 0, output
    assert '<plugin ' not in urdf_path.read_text(encoding='utf-8')


@pytest.mark.parametrize('robot_model', ['base', 'forklift'])
def test_nonexistent_sim_file_is_rejected(robot_model: str, tmp_path: Path) -> None:
    urdf_path = tmp_path / f'{robot_model}_missing_sim_file.urdf'
    missing_sim_path = tmp_path / 'missing_simulation.yaml'
    xacro_path = PACKAGE_DIR / 'urdf' / f'model_{robot_model}.xacro'

    result = run_bash(f'xacro "{xacro_path}" sim_file:="{missing_sim_path}" > "{urdf_path}"')
    output = result.stdout + result.stderr

    assert result.returncode != 0
    assert str(missing_sim_path) in output


@pytest.mark.parametrize(
    ('robot_model', 'component_name', 'topic_keys'), ENABLED_SIMULATION_TOPIC_CASES
)
def test_enabled_simulation_component_requires_a_topic(
    robot_model: str, component_name: str, topic_keys: tuple[str, ...], tmp_path: Path
) -> None:
    source_path = PACKAGE_DIR / 'config' / f'model_{robot_model}' / 'default_simulation.yaml'
    simulation_config = yaml.safe_load(source_path.read_text(encoding='utf-8'))

    assert simulation_config[component_name]['enabled'] is True
    for topic_key in topic_keys:
        simulation_config[component_name][topic_key] = ''

    invalid_sim_path = tmp_path / f'{component_name}.yaml'
    invalid_sim_path.write_text(yaml.safe_dump(simulation_config), encoding='utf-8')
    xacro_path = PACKAGE_DIR / 'urdf' / f'model_{robot_model}.xacro'

    result = run_bash(f'xacro "{xacro_path}" sim_file:="{invalid_sim_path}"')
    output = result.stdout + result.stderr

    assert result.returncode != 0
    assert component_name in output


@pytest.mark.parametrize(
    ('xacro_args', 'expected_error'),
    [
        ('fork_limits:="0 1 2"', 'fork_limits must contain exactly 4 values'),
        ('fork_limits:="0.01 0.2 0.2 20000"', 'fork_mockup: limits lower value must be <= 0'),
        ('fork_limits:="-0.005 -0.001 0.2 20000"', 'fork_mockup: limits upper value must be >= 0'),
        (
            'fork_limits:="-0.03 0.2 0.2 20000"',
            'fork_limits lower value exceeds fork_rest_clearance',
        ),
        ('fork_dynamics:="10.0"', 'fork_mockup: dynamics must contain exactly 2 values'),
        ('fork_dynamics:="-1.0 1.0"', 'fork_mockup: dynamics damping value must be >= 0'),
        ('fork_dynamics:="10.0 -1.0"', 'fork_mockup: dynamics friction value must be >= 0'),
    ],
)
def test_forklift_xacro_rejects_invalid_installation_limits(
    xacro_args: str, expected_error: str, tmp_path: Path
) -> None:
    urdf_path = tmp_path / 'forklift_invalid_limits.urdf'
    xacro_path = PACKAGE_DIR / 'urdf' / 'model_forklift.xacro'

    result = run_bash(f'xacro "{xacro_path}" {xacro_args} > "{urdf_path}"')
    output = result.stdout + result.stderr

    assert result.returncode != 0
    assert expected_error in output


@pytest.mark.parametrize('robot_model', ['base', 'forklift'])
def test_kinematics_slots_map_to_semantic_wheel_names(robot_model: str) -> None:
    params = (PACKAGE_DIR / 'config' / f'model_{robot_model}' / 'default_params.yaml').read_text(
        encoding='utf-8'
    )

    for wheel_index, wheel_name in enumerate(WHEEL_NAMES_BY_KINEMATICS_INDEX):
        section_start = params.index(f'      steerable_wheel_{wheel_index}:')
        next_slot = f'      steerable_wheel_{wheel_index + 1}:'
        section_end = params.find(next_slot, section_start)
        section = (
            params[section_start:] if section_end == -1 else params[section_start:section_end]
        )

        assert f'$(var robot_prefix){wheel_name}' in section
        assert f'$(var robot_prefix){wheel_name}_steering_joint' in section
        assert f'$(var robot_prefix){wheel_name}_rotation_joint' in section


def test_xacro_uses_primitive_wheel_visual_when_s_wheel_use_v_mesh_is_false(
    tmp_path: Path,
) -> None:
    urdf_path = tmp_path / 'model_base_no_wheel_mesh.urdf'
    xacro_path = PACKAGE_DIR / 'urdf' / 'model_base.xacro'

    result = run_bash(
        f'xacro "{xacro_path}" s_wheel_use_v_mesh:=False > "{urdf_path}" '
        f'&& check_urdf "{urdf_path}"'
    )

    output = result.stdout + result.stderr

    assert result.returncode == 0, output
    assert urdf_path.is_file(), output
    assert 'rubber_wheel_2.stl' not in urdf_path.read_text(), output


@pytest.mark.parametrize(
    ('robot_model', 'xacro_args'),
    [
        (
            'base',
            'body_use_visual:=False body_use_collision:=True body_use_inertial:=False '
            's_wheel_use_visual:=False s_wheel_use_inertial:=False',
        )
    ],
)
def test_xacro_geometry_selections_expand_independently(
    robot_model: str, xacro_args: str, tmp_path: Path
) -> None:
    urdf_path = tmp_path / f'{robot_model}_geometry_selection.urdf'
    xacro_path = PACKAGE_DIR / 'urdf' / f'model_{robot_model}.xacro'

    result = run_bash(
        f'xacro "{xacro_path}" {xacro_args} > "{urdf_path}" && check_urdf "{urdf_path}"'
    )
    output = result.stdout + result.stderr

    assert result.returncode == 0, output
    assert 'Successfully Parsed XML' in output, output


def test_xacro_rejects_body_color_without_rgba_components(tmp_path: Path) -> None:
    urdf_path = tmp_path / 'model_base_invalid_body_color.urdf'
    xacro_path = PACKAGE_DIR / 'urdf' / 'model_base.xacro'

    result = run_bash(f'xacro "{xacro_path}" body_color:="1 2 3" > "{urdf_path}"')
    output = result.stdout + result.stderr

    assert result.returncode != 0
    assert 'body_color must contain exactly 4 values' in output


@pytest.mark.parametrize('relative_path', ACTIVE_XACRO_FILES)
def test_xacro_declares_only_used_xml_namespaces(relative_path: str) -> None:
    source = (PACKAGE_DIR / relative_path).read_text(encoding='utf-8')
    robot_tag = re.search(r'<robot\b[^>]*>', source)

    assert robot_tag is not None

    declared_prefixes = set(re.findall(r'xmlns:([A-Za-z_][\w.-]*)=', robot_tag.group()))
    used_prefixes = set(re.findall(r'</?([A-Za-z_][\w.-]*):', source))

    assert declared_prefixes == used_prefixes


@pytest.mark.parametrize('robot_model', ['base', 'forklift'])
def test_urdf_and_kinematics_use_the_same_wheel_radius(robot_model: str) -> None:
    common = (PACKAGE_DIR / 'urdf' / 'common.xacro').read_text(encoding='utf-8')
    params = (PACKAGE_DIR / 'config' / f'model_{robot_model}' / 'default_params.yaml').read_text(
        encoding='utf-8'
    )
    radius_match = re.search(r'name="s_wheel_radius"\s+value="([^"]+)"', common)

    assert radius_match is not None
    assert set(re.findall(r'^\s+radius:\s+([^\s#]+)', params, flags=re.MULTILINE)) == {
        radius_match.group(1)
    }


@pytest.mark.parametrize(
    ('launch_file', 'is_debug'),
    [
        ('real_model_base.launch.py', False),
        ('real_model_forklift.launch.py', False),
        ('debug_model_base.launch.py', True),
        ('debug_model_forklift.launch.py', True),
    ],
)
def test_launch_show_args_lists_expected_static_launch_arguments(
    launch_file: str, is_debug: bool
) -> None:
    result = run_bash(f'ros2 launch robot_rbvogui_common {launch_file} --show-args')
    output = result.stdout + result.stderr

    assert result.returncode == 0, output
    assert "'robot_name'" in output, output
    assert "'robot_params_file'" in output, output
    assert "'robot_xacro_args_file'" in output, output

    if is_debug:
        assert "'robot_description_topic'" in output, output
        assert "'robot_sim_file'" in output, output
        assert "'robot_bridge_config_file'" in output, output
