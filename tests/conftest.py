import os
from pathlib import Path
import subprocess
import sys

PACKAGE_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_DIR))
sys.path.insert(0, str(PACKAGE_DIR.parent / 'ros2_launch_helpers'))


def workspace_dir() -> Path:
    """Return the workspace root that contains install/setup.bash."""
    for parent in PACKAGE_DIR.parents:
        if parent.joinpath('install', 'setup.bash').is_file():
            return parent

    raise FileNotFoundError('Could not locate workspace root with install/setup.bash')


WORKSPACE_DIR = workspace_dir()


def run_bash(command: str, *, timeout: int = 60) -> subprocess.CompletedProcess[str]:
    """Run one bash command in the workspace with ROS and workspace setup sourced."""
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    ros_setup = os.environ.get('ROS_SETUP_BASH', f'/opt/ros/{ros_distro}/setup.bash')
    bash_command = (
        f'source "{ros_setup}" && source "{WORKSPACE_DIR / "install" / "setup.bash"}" && {command}'
    )

    return subprocess.run(
        ['bash', '-lc', bash_command],
        cwd=WORKSPACE_DIR,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )
