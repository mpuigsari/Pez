# File: Dockerfile (no‐GUI, minimal ROS 2 Humble)
FROM ros:humble-ros-base-jammy

# Install just the CLI & build tools, plus the joystick driver
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      python3-rosdep \
      ros-humble-joy \
    && rm -rf /var/lib/apt/lists/*

# Re‐init rosdep so you can rosdep install
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update

# Copy & build your workspace
WORKDIR /pez_ws
COPY pez_ws/src src
RUN rosdep install --from-paths src --ignore-src -r -y
RUN bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# (Optional) navigator-lib if you need it:
# RUN pip3 install --no-cache-dir git+https://github.com/bluerobotics/navigator-lib.git@v0.1.1

# Entrypoint: source ROS and your workspace, then run whatever you pass
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
