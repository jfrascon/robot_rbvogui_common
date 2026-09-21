#!/usr/bin/env bash
set -euo pipefail

package_share="$(ros2 pkg prefix robot_rbvogui_common)/share/robot_rbvogui_common"

ros2 launch robot_rbvogui_common debug_model_base.launch.py \
    robot_name:=rbvogui \
    robot_params_file:="${package_share}/config/model_base/default_params.yaml" \
    robot_params_file_allow_substs:=True \
    robot_xacro_args_file:="${package_share}/config/model_base/default_xacro_args.yaml" \
    robot_sim_file:="${package_share}/config/model_base/default_simulation.yaml" \
    robot_bridge_config_file:="${package_share}/config/model_base/default_bridge.yaml" \
    rviz_enabled:=True \
    gzgui_enabled:=True \
    "$@"
