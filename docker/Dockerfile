ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base-jammy

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential python3-colcon-common-extensions python3-pytest python3-rosdep \
      ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-std-msgs \
      ros-${ROS_DISTRO}-tf2-ros && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/nav3d_ws
COPY . /opt/nav3d_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/build_workspace.sh

ENV NAV3D_WS=/opt/nav3d_ws \
    ROS_DISTRO=${ROS_DISTRO} \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp
COPY docker/entrypoint.sh /ros_entrypoint_nav3d.sh
ENTRYPOINT ["/ros_entrypoint_nav3d.sh"]
CMD ["bash"]
