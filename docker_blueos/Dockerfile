# syntax=docker/dockerfile:1
FROM ros:humble-ros-base-jammy

# ——— BlueOS Extension metadata ———
LABEL permissions='{"NetworkMode": "host", "HostConfig": {"Binds": ["/dev:/dev:rw"],"Privileged": true, "NetworkMode": "host"}, "NetworkMode": "host"}' \
      version="0.0.1" \
      authors='[{"name":"Max","email":"al415556@uji.es"}]' \
      company='{"name":"CIRTESU","about":"ROS2 Navigator Lib","email":"support@your.org"}' \
      type="tool" \
      tags='["navigation","ros"]'

# Install pip and your Python package
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-pip \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir navigator-lib

# Entrypoint to source ROS2
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
