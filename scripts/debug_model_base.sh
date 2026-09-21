#!/usr/bin/env bash
set -euo pipefail

package_share="$(ros2 pkg prefix robot_rbvogui_common)/share/robot_rbvogui_common"

# The launch arguments below are defaults passed explicitly by this script.
# To override any of them, append the replacement argument after the script name.
# Example:
#   debug_model_base.sh rviz_enabled:=False gzgui_enabled:=False
ros2 launch robot_rbvogui_common debug_model_base.launch.py \
    robot_name:=rbv0 \
    robot_params_file:="${package_share}/config/default_params.yaml" \
    robot_params_file_allow_substs:=True \
    robot_xacro_args_file:="${package_share}/config/default_xacro_args.yaml" \
    robot_sim_file:="${package_share}/config/default_simulation.yaml" \
    robot_bridge_config_file:="${package_share}/config/default_bridge.yaml" \
    rviz_enabled:=True \
    gzgui_enabled:=True \
    "$@"
