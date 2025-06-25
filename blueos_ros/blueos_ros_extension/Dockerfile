# Use your prebuilt BlueOS+camera image as the base
FROM inesperez03/blueos-ros-usbcam:latest

# 1) Install pez_comms & server.py Python and ROS deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-pymavlink \
    python3-yaml \
    ros-${ROS_DISTRO}-rospy \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    build-essential \
    python3-catkin-tools \
 && rm -rf /var/lib/apt/lists/*

# 2) Create a Catkin workspace and copy in your packages
RUN mkdir -p /root/blueos_ws/src
WORKDIR /root/blueos_ws/src

# Copy pez_comms & bluerov_server package
COPY ./blueos_ros/pez_comms ./pez_comms
COPY ./blueos_ros/bluerov_server ./bluerov_server


# 3) Build the workspace
WORKDIR /root/blueos_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 4) Ensure ROS environment is sourced on container startup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "source /root/blueos_ws/devel/setup.bash" >> /root/.bashrc

# 5) (Optional) default entrypoint—drop you into a ROS-enabled shell
ENTRYPOINT ["/bin/bash","-l"]
