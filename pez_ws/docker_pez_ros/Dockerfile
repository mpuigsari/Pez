FROM arm32v7/ros:noetic-robot

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-rosdep python3-catkin-tools build-essential \
      git curl pkg-config libssl-dev python3-dev rustc cargo \
    && rm -rf /var/lib/apt/lists/*

# re-initialize rosdep, removing existing default list if present
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update

WORKDIR /root/pez_ws

# copy in your catkin workspace src  
WORKDIR /root/pez_ws
COPY src/pez      src/pez

# clone navigator-lib and build its Python extension
RUN git clone https://github.com/bluerobotics/navigator-lib.git src/navigator-lib \
 && pip3 install maturin --no-cache-dir \
 && cd src/navigator-lib \
 && maturin build --release --no-default-features \
 && pip3 install target/wheels/*.whl

# install any rosdeps used by your package
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y"

# build the entire workspace
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash \
    && catkin_make"

# copy in our entrypoint script
COPY docker_pez_ros/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
