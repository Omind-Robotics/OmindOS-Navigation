cmake_minimum_required(VERSION 3.8)
project(planning_3d_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(planning_3d_prm_node src/planning_3d_prm_node.cpp)
target_include_directories(planning_3d_prm_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(planning_3d_prm_node PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(planning_3d_prm_node
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(planning_3d_prm_node ${PCL_LIBRARIES})

add_executable(bezier_path_optimizer_node src/bezier_path_optimizer_node.cpp)
target_include_directories(bezier_path_optimizer_node PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(bezier_path_optimizer_node PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(bezier_path_optimizer_node
  geometry_msgs
  nav_msgs
  rclcpp
)
target_link_libraries(bezier_path_optimizer_node ${PCL_LIBRARIES})

install(TARGETS
  planning_3d_prm_node
  bezier_path_optimizer_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
