cmake_minimum_required(VERSION 3.0.2)
project(obstacle_processor)

add_compile_options(-std=c++14)

add_definitions(-DPTHREAD_STACK_MIN=16384)

# Keep release binaries portable across x86_64 and arm64 by default. Native
# SIMD can be enabled for a known deployment CPU when maximum throughput is
# more important than redistributing the binary.
option(NAV3D_ENABLE_AVX2 "Compile obstacle_processor with AVX2" OFF)
if(NAV3D_ENABLE_AVX2)
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
  if(NOT COMPILER_SUPPORTS_AVX2)
    message(FATAL_ERROR "NAV3D_ENABLE_AVX2=ON, but the compiler does not support -mavx2")
  endif()
  add_compile_options(-mavx2)
  message(STATUS "AVX2 explicitly enabled")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  tf
)

find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES obstacle_processor
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs pcl_ros pcl_conversions tf
  DEPENDS PCL EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(obstacle_processor_node src/obstacle_processor_node.cpp src/execution_classes.cpp)

target_link_libraries(obstacle_processor_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(obstacle_processor_node OpenMP::OpenMP_CXX)
endif()

add_dependencies(obstacle_processor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
