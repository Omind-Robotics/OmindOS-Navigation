cmake_minimum_required(VERSION 3.0.2)
project(map_publisher)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE")

add_definitions(-DPTHREAD_STACK_MIN=16384)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  eigen_conversions
  open3d_loc  # 依赖open3d_loc来使用open3d_conversions
)

## Find Open3D
if(NOT Open3D_DIR)
  set(Open3D_DIR "$ENV{HOME}/.local/opt/open3d-0.15.1/lib/cmake/Open3D")
endif()
find_package(Open3D REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    tf
    eigen_conversions
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../open3d_loc/include
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node
  src/map_publisher_node.cpp
  ../open3d_loc/src/open3d_conversions/open3d_conversions.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Open3D_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

## Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)
