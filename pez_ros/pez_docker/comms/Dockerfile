# Use the ROS 2 Humble base for ARM64 (Raspberry Pi 4+)
FROM arm64v8/ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# 1) Install only what's needed to build & run pez_comms
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-serial \
      python3-yaml \
      socat \
      nano \
    && rm -rf /var/lib/apt/lists/*

# 2) Initialize rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update

####################################################
# 3) Copy & Build your local ROS 2 packages
####################################################
RUN mkdir -p /pez_ws/src
COPY pez_ros/pez_humble/pez_ws/src/      /pez_ws/src/

RUN source /opt/ros/humble/setup.bash \
 && cd /pez_ws \
 && colcon build --merge-install

# 5) Source ROS 2 and your overlay for every container start
COPY pez_ros/pez_docker/comms/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
