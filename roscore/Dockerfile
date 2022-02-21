FROM ubuntu:20.04

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-vcstools \
    curl \
		wget \
    gnupg \
		gnupg2 \
		lsb-release \
		ca-certificates 

# Add ROS Repository Key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    ros-noetic-rosbash \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

    
