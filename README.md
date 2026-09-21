# robot_rbvogui_common

`robot_rbvogui_common` contains shared RB-VOGUI base description resources,
debug launch helpers, Gazebo bridge launch, ground-vehicle kinematics launch,
robot_state_publisher launch, meshes, RViz, and debug world assets.

## Quick Start

Build the package from the workspace root:

```bash
colcon build --merge-install --packages-select robot_rbvogui_common
source install/setup.bash
```

Start the base model in the debug world:

```bash
ros2 launch robot_rbvogui_common debug_model_base.launch.py
```

For a lighter run without RViz or Gazebo GUI:

```bash
ros2 launch robot_rbvogui_common debug_model_base.launch.py \
  rviz_enabled:=False gzgui_enabled:=False
```

## Launch Files

Public launch files:

- `debug_model_base.launch.py`: base model in Gazebo debug simulation.

Internal launch files:

- `_bridge.launch.py`
- `_ground_vehicle_kinematics.launch.py`
- `_robot_state_publisher.launch.py`

## Configuration Files

The base model defaults live in `config/model_base/`:

```text
config/model_base/
|-- default_params.yaml
|-- default_xacro_args.yaml
|-- default_simulation.yaml
`-- default_bridge.yaml
```

## Robot Description

The base model entry point is:

```text
urdf/model_base.xacro
```

The shared Xacro include is:

```text
urdf/common.xacro
```

Forklift-specific files live in `robot_rbvogui_forklift`.
