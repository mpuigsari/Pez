# syntax=docker/dockerfile:1
ARG UBUNTU_CODENAME=jammy
ARG ROS_DISTRO=humble

########################################
# 1) Builder stage: compile ROS 2 from source
########################################
FROM arm32v7/ubuntu:${UBUNTU_CODENAME} AS ros2-builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8

# Install build tools, ROS apt repo keys, rosdep, and colcon
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg2 lsb-release locales \
      python3-colcon-common-extensions python3-vcstool python3-rosdep \
      build-essential cmake git python3-pip \
    && locale-gen en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
         http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
       > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep, fetch all ROS 2 Humble repos, install deps, build
RUN apt-get update \
 && rosdep init \
 && rosdep update \
 && mkdir -p /root/ros2_ws/src \
 && vcs import /root/ros2_ws/src \
      < https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos \
 && rosdep install --from-paths /root/ros2_ws/src --ignore-src -y \
 && cd /root/ros2_ws \
 && colcon build --install-base /opt/ros/${ROS_DISTRO} \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf /var/lib/apt/lists/*

########################################
# 2) Runtime stage: package ROS 2 + navigator-lib + BlueOS labels
########################################
FROM arm32v7/ubuntu:${UBUNTU_CODENAME}
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# ——— BlueOS Extension metadata ———
LABEL permissions='{"NetworkMode": "host", "HostConfig": {"Binds": ["/dev:/dev:rw"],"Privileged": true, "NetworkMode": "host"}, "NetworkMode": "host"}' \
      version="0.0.1" \
      authors='[{"name":"Max","email":"al415556@uji.es"}]' \
      company='{"name":"CIRTESU","about":"ROS2 Navigator Lib","email":"support@your.org"}' \
      type="tool" \
      tags='["navigation","ros"]'

# Copy the built ROS 2 install
COPY --from=ros2-builder /opt/ros/${ROS_DISTRO} /opt/ros/${ROS_DISTRO}

# Install navigator-lib via pip
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-pip \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir navigator-lib

# Entrypoint: source ROS 2 and exec
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
