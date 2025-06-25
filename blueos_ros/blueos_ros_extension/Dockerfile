# Use your prebuilt BlueOS+camera image as the base
FROM inesperez03/blueos-ros-usbcam:latest

# ──────────────────────────────────────────────────────────────────────────────
# 1) Fix ROS Noetic apt GPG key (resolves EXPKEYSIG errors)
# ──────────────────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg2 apt-transport-https \
    && rm -rf /var/lib/apt/lists/*

# Import the official ROS key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | apt-key add -

# Also fetch directly from Ubuntu keyserver (in case)
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
     --recv-keys F42ED6FBAB17C654 || true

# ──────────────────────────────────────────────────────────────────────────────
# 2) Install pez_comms & server.py Python and ROS deps (minus pymavlink)
# ──────────────────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-numpy \
      python3-yaml \
      ros-${ROS_DISTRO}-rospy \
      ros-${ROS_DISTRO}-geometry-msgs \
      ros-${ROS_DISTRO}-std-msgs \
      build-essential \
      python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# ──────────────────────────────────────────────────────────────────────────────
# 3) Install pymavlink via pip (not available on focal/armhf apt)
# ──────────────────────────────────────────────────────────────────────────────
RUN pip3 install pymavlink

# ──────────────────────────────────────────────────────────────────────────────
# 4) Create a Catkin workspace and copy in your packages
# ──────────────────────────────────────────────────────────────────────────────
RUN mkdir -p /root/blueos_ws/src
WORKDIR /root/blueos_ws/src

COPY ./blueos_ros/pez_comms    ./pez_comms
COPY ./blueos_ros/bluerov_server ./bluerov_server

# ──────────────────────────────────────────────────────────────────────────────
# 5) Build the workspace
# ──────────────────────────────────────────────────────────────────────────────
WORKDIR /root/blueos_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# ──────────────────────────────────────────────────────────────────────────────
# 6) Auto-source ROS on container start
# ──────────────────────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "source /root/blueos_ws/devel/setup.bash" >> /root/.bashrc

# ──────────────────────────────────────────────────────────────────────────────
# 7) Default entrypoint: a ROS-enabled interactive shell
# ──────────────────────────────────────────────────────────────────────────────
ENTRYPOINT ["/bin/bash","-l"]
