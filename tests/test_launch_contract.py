import ast
from pathlib import Path

import pytest

from conftest import PACKAGE_DIR

LAUNCH_FILES = tuple(sorted((PACKAGE_DIR / 'launch').glob('*.launch.py')))
DEBUG_LAUNCH_FILES = tuple(path for path in LAUNCH_FILES if path.name.startswith('debug_model_'))
REAL_LAUNCH_FILES = tuple(path for path in LAUNCH_FILES if path.name.startswith('real_model_'))
MODEL_LAUNCH_FILES = (*DEBUG_LAUNCH_FILES, *REAL_LAUNCH_FILES)
DEFAULT_NODE_ARGS = '{"output":"both","ros_arguments":["--log-level","info"]}'


@pytest.mark.parametrize('launch_file', LAUNCH_FILES, ids=lambda path: path.name)
def test_child_launch_arguments_are_explicit_dicts(launch_file: Path) -> None:
    tree = ast.parse(launch_file.read_text(encoding='utf-8'))
    includes = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'IncludeLaunchDescription'
    ]

    for include in includes:
        launch_arguments = next(
            (keyword.value for keyword in include.keywords if keyword.arg == 'launch_arguments'),
            None,
        )

        assert isinstance(launch_arguments, ast.Call), launch_file
        assert isinstance(launch_arguments.func, ast.Attribute), launch_file
        assert launch_arguments.func.attr == 'items', launch_file
        assert isinstance(launch_arguments.func.value, ast.Dict), launch_file


@pytest.mark.parametrize('launch_file', LAUNCH_FILES, ids=lambda path: path.name)
def test_launch_files_do_not_use_ambient_context_isolation_helpers(launch_file: Path) -> None:
    source = launch_file.read_text(encoding='utf-8')

    assert 'GroupAction' not in source
    assert 'forwarding=' not in source
    assert 'scoped=' not in source
    assert 'def _include_launch(' not in source
    assert 'def _model_config_path(' not in source
    assert 'def _package_path(' not in source


@pytest.mark.parametrize('launch_file', LAUNCH_FILES, ids=lambda path: path.name)
def test_launch_files_use_the_current_node_argument_api(launch_file: Path) -> None:
    source = launch_file.read_text(encoding='utf-8')

    assert 'node_options' not in source
    assert 'node_logging_options' not in source
    assert '_default_node_name' not in source


def test_node_arguments_use_the_standard_default() -> None:
    """Require every public node-argument input to use the shared logging defaults."""
    declarations_found = 0

    for launch_file in LAUNCH_FILES:
        tree = ast.parse(launch_file.read_text(encoding='utf-8'))
        declarations = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'DeclareLaunchArgument'
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str)
            and 'node_args' in node.args[0].value
        ]

        for declaration in declarations:
            default_value = next(
                keyword.value for keyword in declaration.keywords if keyword.arg == 'default_value'
            )
            assert isinstance(default_value, ast.Constant), launch_file
            assert default_value.value == DEFAULT_NODE_ARGS, launch_file
            declarations_found += 1

    assert declarations_found > 0


@pytest.mark.parametrize('launch_file', DEBUG_LAUNCH_FILES, ids=lambda path: path.name)
def test_debug_generate_function_declares_the_spawn_phases(launch_file: Path) -> None:
    tree = ast.parse(launch_file.read_text(encoding='utf-8'))
    generate = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == 'generate_launch_description'
    )
    assignments = {
        target.id: value
        for statement in generate.body
        if isinstance(statement, ast.AnnAssign) and isinstance(statement.target, ast.Name)
        for target, value in [(statement.target, statement.value)]
    }

    assert isinstance(assignments['before_spawn_actions'], ast.List)

    before_calls = [
        element.func.id
        for element in assignments['before_spawn_actions'].elts
        if isinstance(element, ast.Call) and isinstance(element.func, ast.Name)
    ]

    spawn_assignment = next(
        statement
        for statement in generate.body
        if isinstance(statement, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == 'spawn_and_wait_action'
            for target in statement.targets
        )
    )
    assert isinstance(spawn_assignment.value, ast.Call)

    opaque_kwargs = next(
        keyword.value for keyword in spawn_assignment.value.keywords if keyword.arg == 'kwargs'
    )
    assert isinstance(opaque_kwargs, ast.Dict)

    after_spawn_actions = next(
        opaque_kwargs.values[index]
        for index, key in enumerate(opaque_kwargs.keys)
        if isinstance(key, ast.Constant) and key.value == 'after_spawn_actions'
    )
    assert isinstance(after_spawn_actions, ast.List)

    after_calls = [
        element.func.id
        for element in after_spawn_actions.elts
        if isinstance(element, ast.Call) and isinstance(element.func, ast.Name)
    ]

    assert before_calls == ['_include_spawn_world', 'OpaqueFunction']
    assert after_calls[:2] == ['_include_bridge', '_include_kinematics']
    assert after_calls[-1] == '_launch_rviz'


@pytest.mark.parametrize('launch_file', MODEL_LAUNCH_FILES, ids=lambda path: path.name)
def test_model_launch_renders_one_shared_params_file_for_all_children(launch_file: Path) -> None:
    tree = ast.parse(launch_file.read_text(encoding='utf-8'))
    generate = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == 'generate_launch_description'
    )
    actions_assignment = next(
        (
            statement
            for statement in generate.body
            if isinstance(statement, ast.AnnAssign)
            and isinstance(statement.target, ast.Name)
            and statement.target.id == 'actions'
        ),
        None,
    )
    if actions_assignment is not None:
        assert isinstance(actions_assignment.value, ast.List)
        initial_actions = actions_assignment.value.elts
    else:
        return_statement = next(
            statement for statement in generate.body if isinstance(statement, ast.Return)
        )
        assert isinstance(return_statement.value, ast.Call)
        assert isinstance(return_statement.value.func, ast.Name)
        assert return_statement.value.func.id == 'LaunchDescription'
        assert isinstance(return_statement.value.args[0], ast.List)
        initial_actions = return_statement.value.args[0].elts

    robot_namespace_index = next(
        index
        for index, action in enumerate(initial_actions)
        if isinstance(action, ast.Call)
        and isinstance(action.func, ast.Attribute)
        and action.func.attr == 'SetRobotNamespace'
    )
    robot_prefix_index = next(
        index
        for index, action in enumerate(initial_actions)
        if isinstance(action, ast.Call)
        and isinstance(action.func, ast.Attribute)
        and action.func.attr == 'SetRobotPrefix'
    )
    resolved_path_index = next(
        index
        for index, action in enumerate(initial_actions)
        if isinstance(action, ast.Call)
        and isinstance(action.func, ast.Name)
        and action.func.id == 'SetLaunchConfiguration'
        and action.args
        and isinstance(action.args[0], ast.Constant)
        and action.args[0].value == 'resolved_robot_params_file'
    )
    render_index = next(
        index
        for index, action in enumerate(initial_actions)
        if isinstance(action, ast.Call)
        and isinstance(action.func, ast.Attribute)
        and action.func.attr == 'RenderParamsFile'
    )

    assert robot_namespace_index < render_index
    assert robot_prefix_index < render_index
    assert resolved_path_index < render_index

    render_calls = [
        node
        for node in ast.walk(generate)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == 'rlh'
        and node.func.attr == 'RenderParamsFile'
    ]
    assert len(render_calls) == 1

    render_keywords = {keyword.arg: keyword.value for keyword in render_calls[0].keywords}
    assert isinstance(render_keywords['output_context_key'], ast.Constant)
    assert render_keywords['output_context_key'].value == 'resolved_robot_params_file'
    assert isinstance(render_keywords['condition'], ast.Call)

    child_params_count = 0
    for include in [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'IncludeLaunchDescription'
    ]:
        launch_arguments_call = next(
            (keyword.value for keyword in include.keywords if keyword.arg == 'launch_arguments'),
            None,
        )
        if not isinstance(launch_arguments_call, ast.Call):
            continue
        if not isinstance(launch_arguments_call.func, ast.Attribute):
            continue
        arguments_dict = launch_arguments_call.func.value
        if not isinstance(arguments_dict, ast.Dict):
            continue

        arguments = {
            key.value: value
            for key, value in zip(arguments_dict.keys, arguments_dict.values, strict=True)
            if isinstance(key, ast.Constant) and isinstance(key.value, str)
        }
        params_key = next(
            (
                key
                for key in (
                    'robot_rsp_params_file',
                    'robot_bridge_params_file',
                    'robot_params_file',
                )
                if key in arguments
            ),
            None,
        )
        if params_key is None:
            continue

        params_value = arguments[params_key]
        assert isinstance(params_value, ast.Call)
        assert isinstance(params_value.func, ast.Name)
        assert params_value.func.id == 'LaunchConfiguration'
        assert isinstance(params_value.args[0], ast.Constant)
        assert params_value.args[0].value == 'resolved_robot_params_file'

        allow_substs_key = params_key.replace('_file', '_file_allow_substs')
        allow_substs_value = arguments[allow_substs_key]
        assert isinstance(allow_substs_value, ast.Constant)
        assert allow_substs_value.value == 'False'
        child_params_count += 1

    expected_child_count = 2
    expected_child_count += int('forklift' in launch_file.name)
    expected_child_count += int(launch_file.name.startswith('debug_model_'))
    assert child_params_count == expected_child_count
