FROM arm32v7/ros:noetic-robot

# 1) Install only Python and ROS build tools (no Rust/Cargo)
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-rosdep \
      python3-catkin-tools \
      build-essential \
      pkg-config \
      libssl-dev \
      python3-dev \
    && rm -rf /var/lib/apt/lists/*

# 2) (Re-)initialize rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update

# 3) Set up your workspace
WORKDIR /root/pez_ws
COPY pez_ws/src/pez src/pez

# 4) Install navigator-lib from PyPI (no build required)
RUN pip3 install --no-cache-dir bluerobotics-navigator

# 5) Resolve any ROS dependencies for your package
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y"

# 6) Build your catkin workspace
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash \
    && catkin_make"

# 7) Entrypoint wiring
COPY docker_pez_ros/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
