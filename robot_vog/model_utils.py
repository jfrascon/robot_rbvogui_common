from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def get_models() -> list[str]:
    """
    Return the public robot model names available in this package.

    Public model xacro files use the path `urdf/models/model_<robot_model>.xacro`.
    Launch files use the short model name, such as `base` or `forklift`.
    """
    model_file_prefix = 'model_'
    urdf_dir = Path(get_package_share_directory('robot_vog')).joinpath('urdf', 'models')

    if not urdf_dir.is_dir():
        raise FileNotFoundError(f'URDF directory {urdf_dir!r} does not exist.')

    return sorted(
        path.stem.removeprefix(model_file_prefix)
        for path in urdf_dir.glob(f'{model_file_prefix}*.xacro')
        if path.is_file()
    )
