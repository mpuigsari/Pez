# File: Dockerfile
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# 1) Install core tools, ROS tools, and add-apt-repository
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      software-properties-common \
      python3-colcon-common-extensions \
      python3-pip \
      python3-serial \
      python3-yaml \
      python3-rosdep \
      ros-humble-joy \
      ros-humble-rqt \
      ros-humble-rqt-common-plugins \
      qtbase5-dev qtbase5-dev-tools \
      ros-humble-image-transport-plugins \
      ros-humble-compressed-image-transport \
      ros-humble-compressed-depth-image-transport \
      ros-humble-theora-image-transport \
      ros-humble-plotjuggler-ros \
      socat \
    && rm -rf /var/lib/apt/lists/*

# 2) Re-init rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update

# 3) Enable “universe” and refresh apt **without** immediately clearing lists
RUN add-apt-repository universe \
 && apt-get update

# 4) Build your pez_ws workspace
WORKDIR /pez_ws
COPY pez_ros/pez_humble/pez_ws/src/ ./src/
RUN rosdep install --from-paths src --ignore-src -r -y \
 && source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install

# 6) Entrypoint: source ROS and both workspaces
COPY pez_ros/pez_docker/host/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY pez_ros/pez_docker/host/dev_entrypoint.sh /dev_entrypoint.sh
RUN chmod +x /dev_entrypoint.sh

RUN echo '' >> /root/.bashrc \
 && echo '# >>> ROS setup >>>'                     >> /root/.bashrc \
 && echo 'source /opt/ros/humble/setup.bash'       >> /root/.bashrc \
 && echo '[ -f /pez_ws/install/setup.bash ] && \\' >> /root/.bashrc \
 && echo '  source /pez_ws/install/setup.bash'     >> /root/.bashrc \
 && echo '# <<< ROS setup <<<'                     >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
