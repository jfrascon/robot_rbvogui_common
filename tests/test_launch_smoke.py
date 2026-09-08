import pytest

from conftest import run_bash


@pytest.mark.parametrize(
    ('launch_file', 'launch_args', 'expected_text'),
    [
        ('real_model_base.launch.py', '', 'FourSwerveKinematicsSolverRos node initialized.'),
        (
            'debug_model_base.launch.py',
            'rviz_enabled:=False gzgui_enabled:=False',
            'Creating ROS->GZ Bridge: [cmd_vel',
        ),
        (
            'debug_model_base.launch.py',
            'robot_description_topic:=rdesc rviz_enabled:=False gzgui_enabled:=False',
            'Creating ROS->GZ Bridge: [cmd_vel',
        ),
        (
            'debug_model_forklift.launch.py',
            'rviz_enabled:=False gzgui_enabled:=False',
            'JointPositionControllerServer node initialized.',
        ),
    ],
)
def test_robot_launch_smoke(launch_file: str, launch_args: str, expected_text: str) -> None:
    result = run_bash(f'timeout --signal=INT 8s ros2 launch robot_vog {launch_file} {launch_args}')

    output = result.stdout + result.stderr

    # timeout returns 124 when the launch keeps running as expected until interrupted.
    assert result.returncode in {0, 124}, output
    assert 'process started with pid' in output, output
    assert expected_text in output, output
