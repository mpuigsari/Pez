# syntax=docker/dockerfile:1

####################################################
# 1) Base + Navigator install
####################################################
FROM --platform=linux/arm64/v8 ros:humble-ros-base-jammy

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DOMAIN_ID=21

# Install pip, Navigator, Colcon & build-tools
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-colcon-common-extensions \
      build-essential \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir bluerobotics-navigator

####################################################
# 2) Copy & Build your local ROS 2 package
####################################################
# make the workspace
RUN mkdir -p /pez_ws/src

# copy your package into the workspace
COPY pez/pez_humble/pez_ws/src/pez_control /pez_ws/src/pez_control

# build it
RUN . /opt/ros/humble/setup.bash \
 && cd /pez_ws \
 && colcon build --symlink-install \
 && rm -rf /pez_ws/build /pez_ws/install/log

####################################################
# 3) Entrypoint
####################################################
COPY docker_blueos/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
