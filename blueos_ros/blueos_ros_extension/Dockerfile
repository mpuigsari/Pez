# Use your prebuilt BlueOS+camera image as the base
FROM inesperez03/blueos-ros-usbcam:latest

# ──────────────────────────────────────────────────────────────────────────────
# 1) Remove the ROS apt source list to avoid EXPKEYSIG errors
#    (ROS is already baked into the base image)
# ──────────────────────────────────────────────────────────────────────────────
RUN rm -f /etc/apt/sources.list.d/ros-latest.list

# ──────────────────────────────────────────────────────────────────────────────
# 2) Install Python build tools, lxml deps, pez_comms/server.py deps,
#    and the catkin build system itself
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
      ros-${ROS_DISTRO}-catkin \
    && rm -rf /var/lib/apt/lists/*

# ──────────────────────────────────────────────────────────────────────────────
# 3) Install pymavlink via pip (uses system-provided lxml/future)
# ──────────────────────────────────────────────────────────────────────────────
RUN pip3 install --no-build-isolation pymavlink

# ──────────────────────────────────────────────────────────────────────────────
# 4) Create a Catkin workspace and copy in your packages
# ──────────────────────────────────────────────────────────────────────────────
RUN mkdir -p /root/blueos_ws/src
WORKDIR /root/blueos_ws/src

COPY ./blueos_ros/pez_comms      ./pez_comms
COPY ./blueos_ros/bluerov_server ./bluerov_server

# ──────────────────────────────────────────────────────────────────────────────
# 5) Build the workspace
# ──────────────────────────────────────────────────────────────────────────────
WORKDIR /root/blueos_ws
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"

# ──────────────────────────────────────────────────────────────────────────────
# 6) Auto-source ROS on container startup
# ──────────────────────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "source /root/blueos_ws/devel/setup.bash" >> /root/.bashrc

# ──────────────────────────────────────────────────────────────────────────────
# 7) Default entrypoint: a ROS-enabled interactive shell
# ──────────────────────────────────────────────────────────────────────────────
ENTRYPOINT ["/bin/bash","-l"]
