cmake_minimum_required(VERSION 3.8)
project(nmpc_planner_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

set(ROS1_NMPC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../3D_NAV/local_planner/nmpc_planner")

find_path(CASADI_INCLUDE_DIR casadi/casadi.hpp
  HINTS "$ENV{CASADI_ROOT}/include"
  PATHS /opt/casadi/include /usr/local/include)
find_library(CASADI_LIBRARY casadi
  HINTS "$ENV{CASADI_LIB_PATH}" "$ENV{CASADI_ROOT}/lib"
  PATHS /opt/casadi/lib /usr/local/lib)

if(NOT CASADI_INCLUDE_DIR OR NOT CASADI_LIBRARY)
  message(FATAL_ERROR "CasADi 3.5.5 is required for the original NMPC solver")
endif()

add_executable(nmpc_local_planner_node
  src/local_planner_node.cpp
  ${ROS1_NMPC_DIR}/src/mpc_solver.cpp)
target_include_directories(nmpc_local_planner_node PRIVATE
  ${ROS1_NMPC_DIR}/include
  ${CASADI_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(nmpc_local_planner_node
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(nmpc_local_planner_node ${CASADI_LIBRARY})

add_executable(nmpc_controller_node
  src/controller_node.cpp
  ${ROS1_NMPC_DIR}/src/keyboard_input.cpp)
target_include_directories(nmpc_controller_node PRIVATE ${ROS1_NMPC_DIR}/include)
ament_target_dependencies(nmpc_controller_node
  geometry_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS nmpc_local_planner_node nmpc_controller_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
