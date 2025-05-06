# syntax=docker/dockerfile:1

########################################
# 1) Builder stage: compile ROS 2 Humble from source
########################################
FROM arm32v7/ubuntu:jammy AS ros2-builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# 1a) Install core tools & enable universe
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      software-properties-common \
      curl gnupg2 dirmngr lsb-release locales \
      python3-pip build-essential cmake git \
 && add-apt-repository universe \
 && locale-gen en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

# 1b) Add the ROS 2 Humble apt key & repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# 1c) Install the ROS build tools
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-vcstool \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# 1d) Fetch & build all of ROS 2 Humble
RUN rosdep init \
 && rosdep update \
 && mkdir -p /root/ros2_ws/src \
 # <-- Fixed: download the repos file and pipe into vcs import
 && curl -sSL https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos \
       | vcs import /root/ros2_ws/src \
 && rosdep install --from-paths /root/ros2_ws/src --ignore-src -y \
 && cd /root/ros2_ws \
 && colcon build --install-base /opt/ros/humble \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf /var/lib/apt/lists/*

########################################
# 2) Runtime stage: bundle ROS 2 + navigator-lib + your BlueOS labels
########################################
FROM arm32v7/ubuntu:jammy
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# ——— BlueOS Extension metadata ———
LABEL permissions='{"NetworkMode":"host","HostConfig":{"Binds":["/dev:/dev:rw"],"Privileged":true,"NetworkMode":"host"}}' \
      version="0.0.1" \
      authors='[{"name":"Max","email":"al415556@uji.es"}]' \
      company='{"name":"CIRTESU","about":"ROS2 Navigator Lib","email":"support@your.org"}' \
      type="tool" \
      tags='["navigation","ros"]'

# Copy in the built ROS 2 install
COPY --from=ros2-builder /opt/ros/humble /opt/ros/humble

# Install navigator-lib via pip
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-pip \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir navigator-lib

# Entrypoint: source ROS & exec
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
