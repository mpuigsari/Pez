####################################################
# 1) Base + Navigator install
####################################################
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DOMAIN_ID=21

# Install pip, Navigator, Colcon & build-tools, plus Python 3.10 dev headers
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-dev \                    
      python3.10-dev \                 
      libpython3.10-dev \
      python3-all-dev \              
      python3-pip \
      python3-colcon-common-extensions \
      build-essential \
      cmake \                          
      ros-humble-usb-cam \
      i2c-tools \
      libgpiod2 \
      python3-rpi.gpio \
      python3-serial \
      python3-yaml \
      socat \
      nano \
 && rm -rf /var/lib/apt/lists/*

# Now install Python packages via pip
RUN pip3 install --no-cache-dir \
      bluerobotics-navigator \
      adafruit-blinka \
      smbus2

####################################################
# 2) Copy & Build your local ROS 2 packages
####################################################
RUN mkdir -p /pez_ws/src
COPY pez_ros/pez_humble/pez_ws/src/      /pez_ws/src/

RUN source /opt/ros/humble/setup.bash \
 && cd /pez_ws \
 && colcon build --symlink-install

####################################################
# 3) Entrypoint
####################################################
COPY pez_ros/pez_docker/pez/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

RUN echo '' >> /root/.bashrc \
 && echo '# >>> ROS setup >>>'                     >> /root/.bashrc \
 && echo 'source /opt/ros/humble/setup.bash'       >> /root/.bashrc \
 && echo '[ -f /pez_ws/install/setup.bash ] && \\' >> /root/.bashrc \
 && echo '  source /pez_ws/install/setup.bash'     >> /root/.bashrc \
 && echo '# <<< ROS setup <<<'                     >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
