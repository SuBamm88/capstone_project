FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/workspace

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    git \
    iputils-ping \
    nano \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-dev-tools \
    sudo \
    vim \
    wget \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR ${WORKSPACE}

# Keep the entrypoint simple so developers can source ROS and work immediately.
CMD ["/bin/bash"]
