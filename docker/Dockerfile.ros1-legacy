ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base-focal AS casadi-builder

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG CASADI_VERSION=3.5.5

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential ca-certificates cmake coinor-libipopt-dev gfortran git \
      liblapack-dev pkg-config && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch "${CASADI_VERSION}" \
      https://github.com/casadi/casadi.git /tmp/casadi && \
    cmake -S /tmp/casadi -B /tmp/casadi/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/casadi \
      -DWITH_IPOPT=ON \
      -DWITH_PYTHON=OFF \
      -DWITH_EXAMPLES=OFF && \
    cmake --build /tmp/casadi/build --parallel "$(nproc)" && \
    cmake --install /tmp/casadi/build

FROM ros:${ROS_DISTRO}-ros-base-focal

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake coinor-libipopt1v5 git libeigen3-dev libpcl-dev \
      python3-catkin-tools python3-rosdep python3-rospkg \
      ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-pcl-ros \
      ros-${ROS_DISTRO}-pluginlib ros-${ROS_DISTRO}-roscpp \
      ros-${ROS_DISTRO}-rospy ros-${ROS_DISTRO}-rviz \
      ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-std-msgs \
      ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-tf2 \
      ros-${ROS_DISTRO}-tf2-geometry-msgs ros-${ROS_DISTRO}-tf2-ros && \
    rm -rf /var/lib/apt/lists/*

COPY --from=casadi-builder /opt/casadi /opt/casadi
COPY . /opt/nav3d_ws

ENV NAV3D_WS=/opt/nav3d_ws \
    NAV3D_DATA_ROOT=/data \
    CASADI_ROOT=/opt/casadi \
    CASADI_LIB_PATH=/opt/casadi/lib \
    LD_LIBRARY_PATH=/opt/casadi/lib

WORKDIR /opt/nav3d_ws
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    ./scripts/build_workspace.sh core

COPY docker/entrypoint.sh /ros_entrypoint_nav3d.sh
ENTRYPOINT ["/ros_entrypoint_nav3d.sh"]
CMD ["bash"]
