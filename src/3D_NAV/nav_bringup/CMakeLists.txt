cmake_minimum_required(VERSION 3.0.2)
project(nav_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/go2_base_control_roslaunch.sh
  scripts/m20_stair_workflow.sh
  scripts/offline_base_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
