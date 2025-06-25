# Use your prebuilt BlueOS+camera image as the base
FROM inesperez03/blueos-ros-usbcam:latest

# 1) Remove the broken ROS apt source (Noetic is already in the base image)
RUN rm -f /etc/apt/sources.list.d/ros-latest.list

# 2) Install Python build tools, lxml/future deps, and Catkin tooling
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

# 3) Install pymavlink quickly, leveraging system lxml & future
RUN pip3 install --no-build-isolation pymavlink

# 4) Create a Catkin workspace
RUN mkdir -p /root/blueos_ws/src
WORKDIR /root/blueos_ws/src

# 5) Copy your packages
COPY ./blueos_ros/pez_comms    ./pez_comms
COPY ./blueos_ros/bluerov_server ./bluerov_server

# 6) Build the workspace, *sourcing* ROS so catkin is found
WORKDIR /root/blueos_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release"

# 7) Auto-source ROS and your workspace on container startup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "source /root/blueos_ws/devel/setup.bash" >> /root/.bashrc

# 8) Default to a ROS-enabled shell
ENTRYPOINT ["/bin/bash","-l"]
