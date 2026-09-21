cmake_minimum_required(VERSION 3.8)
project(robot_rbvogui_common)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config doc launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(
  PROGRAMS scripts/debug_model_base.sh
  DESTINATION share/${PROJECT_NAME}/scripts)

install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Exclude linters that require package-specific configuration.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8)

  # Register the remaining linters declared as test dependencies in package.xml.
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_flake8 REQUIRED)

  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")

  ament_add_pytest_test(
    test_installed_resources tests/test_installed_resources.py TIMEOUT 120 ENV
    "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
  ament_add_pytest_test(test_debug_scripts tests/test_debug_scripts.py TIMEOUT 120)
  ament_add_pytest_test(
    test_debug_launch_helpers tests/test_debug_launch_helpers.py TIMEOUT 120 ENV
    "ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(
    test_model_xacro_args_contract tests/test_model_xacro_args_contract.py TIMEOUT 120)
  ament_add_pytest_test(
    test_render_robot_urdf_launch_helpers tests/test_render_robot_urdf_launch_helpers.py
    TIMEOUT 120 ENV "ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(
    test_robot_state_publisher_launch_helpers tests/test_robot_state_publisher_launch_helpers.py
    TIMEOUT 120 ENV "ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(
    test_battery_contract tests/test_battery_contract.py TIMEOUT 120 ENV
    "ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_logs")
endif()

ament_package()
