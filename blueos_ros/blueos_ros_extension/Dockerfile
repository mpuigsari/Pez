# Use your prebuilt BlueOS+camera image as the base
FROM inesperez03/blueos-ros-usbcam:latest

# ──────────────────────────────────────────────────────────────────────────────
# 1) Remove the ROS apt-source list to skip the expired-keyged error
#    (the base image already has /opt/ros/noetic installed)
# ──────────────────────────────────────────────────────────────────────────────
RUN rm -f /etc/apt/sources.list.d/ros-latest.list

# ──────────────────────────────────────────────────────────────────────────────
# 2) Install Python build tools, lxml/future deps, and Catkin tooling
# ──────────────────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-numpy \
      python3-yaml \
      python3-lxml \
      python3-dev \
      python3-setuptools \
      python3-wheel \
      python3-future \
      cython \
      build-essential \
      python3-catkin-tools \
      python3-catkin-pkg \
    && rm -rf /var/lib/apt/lists/*

# ──────────────────────────────────────────────────────────────────────────────
# 3) Install pymavlink quickly, using system lxml & future
# ──────────────────────────────────────────────────────────────────────────────
RUN pip3 install --no-build-isolation pymavlink

# ──────────────────────────────────────────────────────────────────────────────
# 4) Export ROS env so CMake and catkin_make can find everything
# ──────────────────────────────────────────────────────────────────────────────
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV CMAKE_PREFIX_PATH=$ROS_ROOT:$CMAKE_PREFIX_PATH
ENV PATH=$ROS_ROOT/bin:$PATH
ENV LD_LIBRARY_PATH=$ROS_ROOT/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=$ROS_ROOT/lib/python3/dist-packages:$PYTHONPATH

# ──────────────────────────────────────────────────────────────────────────────
# 5) Create a Catkin workspace and copy in your Python-only packages
# ──────────────────────────────────────────────────────────────────────────────
RUN mkdir -p /root/blueos_ws/src
WORKDIR /root/blueos_ws/src

COPY ./blueos_ros/pez_comms    ./pez_comms
COPY ./blueos_ros/bluerov_server ./bluerov_server

# ──────────────────────────────────────────────────────────────────────────────
# 6) Build the workspace (pure-Python, so fast)
# ──────────────────────────────────────────────────────────────────────────────
WORKDIR /root/blueos_ws
RUN bash -c "catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ──────────────────────────────────────────────────────────────────────────────
# 7) Auto-source ROS on container startup
# ──────────────────────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "source /root/blueos_ws/devel/setup.bash" >> /root/.bashrc

# ──────────────────────────────────────────────────────────────────────────────
# 8) Default entrypoint: an interactive ROS-enabled shell
# ──────────────────────────────────────────────────────────────────────────────
ENTRYPOINT ["/bin/bash","-l"]

