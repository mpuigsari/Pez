# syntax=docker/dockerfile:1

########################################
# Runtime stage: ROS 2 Humble + Navigator-Lib
########################################
FROM rclex/arm32v7_ros_docker:humble

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# ——— Your BlueOS Extension metadata ———
LABEL permissions='{"NetworkMode":"host","HostConfig":{"Binds":["/dev:/dev:rw"],"Privileged":true,"NetworkMode":"host"}}' \
      version="0.0.1" \
      authors='[{"name":"Max","email":"al415556@uji.es"}]' \
      company='{"name":"CIRTESU","about":"ROS2 Navigator Lib","email":"support@your.org"}' \
      type="tool" \
      tags='["navigation","ros"]'

# Install your Python package
RUN pip3 install --no-cache-dir navigator-lib

# Entrypoint: source ROS 2 & exec user command
COPY docker_blueos/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
