# File: Dockerfile (with rqt)
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# 1) Install build tools, joystick, and rqt GUI
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      python3-serial \
      python3-yaml \
      python3-rosdep \
      ros-humble-joy \
      ros-humble-rqt \
      #ros-humble-plotjuggler \
      #ros-humble-plotjuggler-ros  \
      ros-humble-rqt-common-plugins \
      qtbase5-dev qtbase5-dev-tools \
      ros-humble-image-transport-plugins \
      ros-humble-compressed-image-transport \
      ros-humble-compressed-depth-image-transport \
      ros-humble-theora-image-transport \
      socat \
    && rm -rf /var/lib/apt/lists/*

# 2) Re‐init rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update

# 3) Copy & build your workspace
RUN mkdir -p /pez_ws/src
COPY pez_ros/pez_humble/pez_ws/src /pez_ws/src
RUN rosdep install --from-paths /pez_ws/src --ignore-src -r -y \
      && . /opt/ros/humble/setup.bash

RUN source /opt/ros/humble/setup.bash \
 && cd /pez_ws \
 && colcon build --symlink-install

RUN mkdir -p /ws_plotjuggler/src
RUN cd /ws_plotjuggler/src \
 && git clone https://github.com/PlotJuggler/plotjuggler_msgs.git \
 && git clone https://github.com/facontidavide/PlotJuggler.git \
 && git clone https://github.com/PlotJuggler/plotjuggler-ros-plugins.git \
 && cd /ws_plotjuggler
RUN rosdep install --from-paths src --ignore-src -r -y \
      && . /opt/ros/humble/setup.bash

RUN source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install


# 4) Entrypoint: source ROS & your workspace
COPY pez_ros/pez_docker/host/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

