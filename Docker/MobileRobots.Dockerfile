############################################################
#####################  Base Image ##########################
############################################################

# Getting Ubuntu 22.04 image
FROM ubuntu:22.04

# Updating and Upgrading
RUN apt-get update -y && apt-get upgrade -y

##########################################################
###################### GENERAL SETUP #####################
##########################################################

ARG DEBIAN_FRONTEND=noninteractive

# Installing needed packages
RUN apt-get install -y apt-utils \
                       git \
                       nano \
                       curl \
                       python3-pip \
                       xz-utils \
                       python3-tk \
                       xterm \
                       libusb-1.0-0-dev \
                       desktop-file-utils \
                       libgtk-3-dev \
                       software-properties-common \
                       evince \
                       net-tools \
                       terminator \
                       iputils-ping

# Updating and Upgrading
RUN apt-get update -y && apt-get upgrade -y

############################################################
##################### ROS2 Humble ##########################
############################################################

# Install locale and UTF-8 support
RUN apt update && apt install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Setup Ubuntu universe repo
RUN apt update -y && apt install -y software-properties-common \
    && add-apt-repository universe

# Install ROS 2 apt source
RUN apt update -y && apt install -y curl gnupg lsb-release

# Download the correct ros2-apt-source .deb for Ubuntu 22.04
RUN export UBUNTU_CODENAME=$( . /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}} ) \
    && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

# Install ROS 2 Humble Packages
RUN apt update -y && apt upgrade -y
RUN apt install -y ros-humble-desktop
RUN apt install -y ros-dev-tools

# Source ROS2 on container startup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Using Cyclone DDS
RUN apt install -y ros-humble-rmw-cyclonedds-cpp
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

# Installing Zenoh 
RUN curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
RUN echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" |  tee -a /etc/apt/sources.list > /dev/null

# Updating and Upgrading
RUN apt-get update -y && apt-get upgrade -y

# Installing Zenoh Executable
RUN apt install -y zenoh-bridge-ros2dds

############################################################
##################### TurtleBot3 Setup #####################
############################################################

# Install TurtleBot3 ROS dependencies
RUN apt update -y && apt install -y \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-colcon-common-extensions \
    git

# Create workspace
RUN mkdir -p /home/turtlebot3_ws/src
WORKDIR /home/turtlebot3_ws/src

# Clone TurtleBot3 packages
RUN git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git  && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git  && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git  && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git


# Build workspace
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    cd /home/turtlebot3_ws && \
    colcon build --symlink-install
WORKDIR /home/turtlebot3_ws

# Automatically source workspace + env configs
RUN echo "export TURTLEBOT3_MODEL=waffle" >> /root/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc
RUN echo "source /home/turtlebot3_ws/install/setup.bash" >> /root/.bashrc

# Clone Main Lab Repo 
WORKDIR /home/
RUN git clone https://github.com/THI-Robotics-Lab/Mobile-Robots-Practical-Lab.git
