cmake_minimum_required(VERSION 3.8)
project(nav3d_hardware_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/go2_odom_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_odom_bridge)

install(PROGRAMS scripts/livox_imu_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME livox_imu_adapter)

install(PROGRAMS scripts/lio_base_odom_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lio_base_odom_adapter)

install(FILES scripts/go2_command_gate_core.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES scripts/go2_safety_supervisor_core.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/go2_command_gate_dry_run.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_command_gate_dry_run)

install(PROGRAMS scripts/go2_safety_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_safety_supervisor)

install(PROGRAMS scripts/go2_zero_velocity_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_zero_velocity_test)

install(PROGRAMS scripts/go2_base_controller_ros2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_base_controller_ros2)

install(PROGRAMS scripts/offline_base_sim_ros2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME offline_base_sim_ros2)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(go2_command_gate_core
    test/test_go2_command_gate_core.py)
  ament_add_pytest_test(go2_safety_supervisor_core
    test/test_go2_safety_supervisor_core.py)
  ament_add_pytest_test(go2_zero_velocity_test
    test/test_go2_zero_velocity_test.py)
  ament_add_pytest_test(lio_base_odom_adapter
    test/test_lio_base_odom_adapter.py)
  ament_add_pytest_test(go2_base_controller_ros2
    test/test_go2_base_controller_ros2.py)
endif()

ament_package()
