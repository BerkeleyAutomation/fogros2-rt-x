ARG UBUNTU_DISTRO=jammy
#rolling is alternative
ARG ROS_DISTRO=humble 
FROM ubuntu:${UBUNTU_DISTRO}

# Set up install, set tzdata
ARG UBUNTU_DISTRO
ARG ROS_DISTRO
ENV TZ=America/Vancouver
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Get ROS key
RUN apt update && apt install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install apt deps 
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_DISTRO} main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
  ros-${ROS_DISTRO}-desktop \
  python3-colcon-common-extensions \
  python3-pip \
  libgmp-dev \
  sqlite3 \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-ament-cmake-nose \
  ros-${ROS_DISTRO}-rosbag2 \
  git

RUN rm -rf /var/lib/apt/lists/*

# Install python deps
RUN python3 -m pip install --no-cache-dir -U imageio tensorflow envlogger[tfds] numpy transforms3d rosbags tensorflow_datasets

# Create FogROS2 worspace and build it
ENV ROS_WS=/home/root/fog_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}/src
RUN git clone https://github.com/Box-Robotics/ros2_numpy

COPY .  ${ROS_WS}/src/
WORKDIR ${ROS_WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
      colcon build --cmake-clean-cache

CMD ["bash"]