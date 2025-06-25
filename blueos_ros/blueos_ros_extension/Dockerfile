# Use your prebuilt BlueOS+camera image as the base
FROM inesperez03/blueos-ros-usbcam:latest

# ──────────────────────────────────────────────────────────────────────────────
# Remove the ROS apt source (packages are already baked into the base image)
# This avoids the EXPKEYSIG errors during apt-get update
# ──────────────────────────────────────────────────────────────────────────────
RUN rm -f /etc/apt/sources.list.d/ros-latest.list

# ──────────────────────────────────────────────────────────────────────────────
# Install pez_comms & server.py Python and ROS deps (minus pymavlink)
# ──────────────────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-numpy \
      python3-yaml \
      build-essential \
      python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# ──────────────────────────────────────────────────────────────────────────────
# Install pymavlink via pip
# ──────────────────────────────────────────────────────────────────────────────
RUN pip3 install pymavlink

# ──────────────────────────────────────────────────────────────────────────────
# Create a Catkin workspace and copy in your packages
# ──────────────────────────────────────────────────────────────────────────────
RUN mkdir -p /root/blueos_ws/src
WORKDIR /root/blueos_ws/src

COPY ./blueos_ros/pez_comms    ./pez_comms
COPY ./blueos_ros/bluerov_server ./bluerov_server

# ──────────────────────────────────────────────────────────────────────────────
# Build the workspace
# ──────────────────────────────────────────────────────────────────────────────
WORKDIR /root/blueos_ws
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"

# ──────────────────────────────────────────────────────────────────────────────
# Auto-source ROS on container startup
# ──────────────────────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "source /root/blueos_ws/devel/setup.bash" >> /root/.bashrc

# ──────────────────────────────────────────────────────────────────────────────
# Default entrypoint: a ROS-enabled interactive shell
# ──────────────────────────────────────────────────────────────────────────────
ENTRYPOINT ["/bin/bash","-l"]
