FROM arm32v7/ros:noetic-robot

# 1) Install Python, ROS tools, and Rust/Cargo  
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-rosdep python3-catkin-tools build-essential \
      pkg-config libssl-dev python3-dev \
      rustc cargo \
    && rm -rf /var/lib/apt/lists/*

# 2) (Re-)initialize rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update

WORKDIR /root/pez_ws
COPY pez_ws/src/pez src/pez

# 3) Install navigator-lib (source via pip → Rust build)  
RUN pip3 install --no-cache-dir bluerobotics-navigator

# 4) Now that pip has built the extension, you can safely remove Rust  
RUN apt-get purge -y rustc cargo \
 && apt-get autoremove -y \
 && rm -rf /var/lib/apt/lists/*

# 5) Install ROS deps + build your workspace  
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y \
    && catkin_make"

COPY docker_pez_ros/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
