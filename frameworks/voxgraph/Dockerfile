FROM nvidia/cuda:11.4.0-base-ubuntu20.04

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS Noetic and dependencies
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    git \
    sudo \
    python3-catkin-tools \
    cmake \
    build-essential \
    libeigen3-dev \
    libopencv-dev \
    ros-noetic-ros-base \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-message-generation \
    ros-noetic-roslint \
    && rm -rf /var/lib/apt/lists/*

# Add a non-root user (optional, for better file permissions)
RUN useradd -ms /bin/bash admin
USER admin

# Set up Xauthority for display forwarding
RUN mkdir -p /home/admin && touch /home/admin/.Xauthority && chown admin:admin /home/admin/.Xauthority

WORKDIR /home/admin

# Source ROS when container starts
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && bash"]
