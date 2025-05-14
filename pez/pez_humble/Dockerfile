# File: Dockerfile (with rqt)
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# 1) Install build tools, joystick, and rqt GUI
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      python3-rosdep \
      ros-humble-joy \
      ros-humble-rqt \
      ros-humble-plotjuggler \
      ros-humble-plotjuggler-ros \
      ros-humble-rqt-common-plugins \
      qtbase5-dev qtbase5-dev-tools \
      ros-humble-image-transport-plugins \
      ros-humble-compressed-image-transport \
      ros-humble-compressed-depth-image-transport \
      ros-humble-theora-image-transport

    && rm -rf /var/lib/apt/lists/*

# 2) Re‐init rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update

# 3) Copy & build your workspace
WORKDIR /pez_ws
COPY pez_ws/src src
RUN rosdep install --from-paths src --ignore-src -r -y \
 && . /opt/ros/humble/setup.bash

# 4) Entrypoint: source ROS & your workspace
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

