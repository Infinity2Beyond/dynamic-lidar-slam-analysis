ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-core as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libcamera-dev \
    ros-${ROS_DISTRO}-camera-ros \
    ros-${ROS_DISTRO}-libcamera \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-ros2bag \
    ros-${ROS_DISTRO}-rosbag2-transport \
    v4l-utils \
    nano git wget curl \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rtabmap-slam \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    python3-matplotlib \
    python3-numpy \
    python3-scipy \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    pcl-tools \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install dufomap dynamic-object-removal pyquaternion

RUN usermod -a -G video root
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "alias cs='colcon build --symlink-install'" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

RUN apt-get update && rosdep init || true && rosdep update

WORKDIR /workspace
CMD ["bash"]