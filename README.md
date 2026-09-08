# [`robot_vog`](https://github.com/jfrascon/robot_vog/)

`robot_vog` describes a family of ground robots with four steerable wheels.

The package provides two public robot models:

- `base`: the mobile platform without a fork.
- `forklift`: the mobile platform with its fork and sensor layout.

## Public Launch Files

Each model has a real-robot entry point and a standalone Gazebo debug entry point:

- `real_model_base.launch.py`
- `real_model_forklift.launch.py`
- `debug_model_base.launch.py`
- `debug_model_forklift.launch.py`

Launch the real base model:

```bash
ros2 launch robot_vog real_model_base.launch.py
```

Launch the real forklift model:

```bash
ros2 launch robot_vog real_model_forklift.launch.py
```

Launch either model in the package-local debug world:

```bash
ros2 launch robot_vog debug_model_base.launch.py
ros2 launch robot_vog debug_model_forklift.launch.py
```

The debug world places four worker models around the robot.
The workers provide visible objects for testing the simulated lidar and camera data in RViz.

The package also installs convenience scripts that pass every default configuration file explicitly:

```bash
$(ros2 pkg prefix robot_vog)/share/robot_vog/scripts/debug_model_base_with_defaults.sh
$(ros2 pkg prefix robot_vog)/share/robot_vog/scripts/debug_model_forklift_with_defaults.sh
```

Additional launch arguments can be appended to either command.
For example, this starts the base model without RViz or the Gazebo GUI:

```bash
$(ros2 pkg prefix robot_vog)/share/robot_vog/scripts/debug_model_base_with_defaults.sh \
    rviz_enabled:=False gzgui_enabled:=False
```

## Internal Launch Files

The public launch files compose these internal launch files:

- `_robot_state_publisher.launch.py`: generates `robot_description` and starts `robot_state_publisher`.
- `_bridge.launch.py`: starts the model-specific ROS-Gazebo bridge.
- `_ground_vehicle_kinematics.launch.py`: starts the four-swerve kinematics node.
- `_fork_control.launch.py`: starts fork control and, in real mode, the serial driver.

Every `IncludeLaunchDescription` passes its child inputs explicitly through `launch_arguments`.
This prevents a child from accidentally inheriting a value left in the launch context by an earlier include.

Node action properties use one JSON launch argument named `node_args` in launch files that own one node.
Launch files that compose several nodes expose one descriptive `*_node_args` argument for each node.
The node name provided inside `node_args` overrides the default name defined by the child launch file.

The debug launch files expose `robot_description_topic` as the topic shared by `robot_state_publisher` and Gazebo spawn.
A relative value resolves inside the robot namespace, while an absolute value keeps its root namespace.
The debug launch prepends this topic remapping before the advanced remappings in `robot_rsp_node_args`.

## Model Configuration

Each model keeps its complete default configuration in one directory:

```text
config/
├── model_base/
│   ├── default_params.yaml
│   ├── default_xacro_args.yaml
│   ├── default_simulation.yaml
│   └── default_bridge.yaml
└── model_forklift/
    ├── default_params.yaml
    ├── default_xacro_args.yaml
    ├── default_simulation.yaml
    └── default_bridge.yaml
```

The files have separate responsibilities:

- `default_params.yaml` configures ROS nodes.
- `default_xacro_args.yaml` configures model-description choices.
- `default_simulation.yaml` configures Gazebo plugins.
- `default_bridge.yaml` configures ROS-Gazebo topic bridges.

An enabled simulation component must define the topic required by its Gazebo plugin.
The `pose_publisher` component requires at least one of `topic`, `topic_with_covariance`, or `tf_topic`.
Xacro expansion fails instead of silently omitting an enabled plugin when this contract is violated.

`use_sim_time` is also owned by each node launch file.
It must not be placed in `default_params.yaml`.

The real-robot launch files render `default_params.yaml` once when `robot_params_file_allow_substs` is true.
Every child then receives the same rendered file with parameter substitutions disabled.
When substitutions are disabled, every child receives the original file unchanged.

Runtime Xacro arguments such as `namespace`, `robot_name`, and `sim_file` are owned by launch files.
They must not be placed in `default_xacro_args.yaml`.

## Debug Startup Order

The debug launch files start actions in this order:

1. Gazebo debug world.
2. `robot_state_publisher`.
3. Gazebo model spawn process.
4. ROS-Gazebo bridge.
5. Ground-vehicle kinematics.
6. Fork control for the forklift model.
7. RViz when enabled.

The current launch files start these processes in the listed order.
The bridge, kinematics, fork control and RViz start only after the Gazebo spawn process exits successfully.
If Gazebo cannot spawn the model, the debug launch reports the return code and shuts down.

## Inspecting Launch Arguments

Use `--show-args` with any public launch file:

```bash
ros2 launch robot_vog real_model_base.launch.py --show-args
ros2 launch robot_vog debug_model_forklift.launch.py --show-args
```
