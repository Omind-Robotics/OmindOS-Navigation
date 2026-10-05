cmake_minimum_required(VERSION 3.0.2)
project(nmpc_planner)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

add_definitions(-DPTHREAD_STACK_MIN=16384)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## Find system dependencies
find_package(Eigen3 REQUIRED)

# Prefer the local CasADi C++ tree. The Python wheel is built with a different
# C++ ABI, so it cannot be linked directly by these ROS C++ nodes.
find_path(CASADI_INCLUDE_DIR casadi/casadi.hpp
  HINTS "$ENV{CASADI_ROOT}/include"
  PATHS /usr/local/include /opt/casadi/include
)

find_library(CASADI_LIBRARY casadi
  HINTS "$ENV{CASADI_LIB_PATH}" "$ENV{CASADI_ROOT}/lib"
  PATHS /usr/local/lib /opt/casadi/lib
)

if(CASADI_INCLUDE_DIR AND CASADI_LIBRARY)
  message(STATUS "Found CasADi: ${CASADI_LIBRARY}")
else()
  message(FATAL_ERROR "CasADi C++ library not found. Set CASADI_ROOT or CASADI_LIB_PATH.")
endif()

## catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nmpc_planner
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
  DEPENDS EIGEN3
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIR}
)

## Declare a C++ library
add_library(nmpc_planner
  src/mpc_solver.cpp
  src/local_planner.cpp
  src/controller_node.cpp
  src/keyboard_input.cpp
)

## Add cmake target dependencies of the library
add_dependencies(nmpc_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(nmpc_planner
  ${catkin_LIBRARIES}
  ${CASADI_LIBRARY}
)

## Declare C++ executables
add_executable(nmpc_controller_node src/controller_main.cpp)
add_executable(nmpc_local_planner_node src/local_planner_main.cpp)

## Add cmake target dependencies of the executables
add_dependencies(nmpc_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nmpc_local_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link executables against
target_link_libraries(nmpc_controller_node
  nmpc_planner
  ${catkin_LIBRARIES}
  ${CASADI_LIBRARY}
)

target_link_libraries(nmpc_local_planner_node
  nmpc_planner
  ${catkin_LIBRARIES}
  ${CASADI_LIBRARY}
)

## Install targets
install(TARGETS nmpc_planner nmpc_controller_node nmpc_local_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
