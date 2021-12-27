FROM ros:noetic-ros-core-focal
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    unzip \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    wget \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# add stuff to path 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

# install dependencies for librealsense
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    at \
    cmake \
    avahi-daemon 

# download librealsense v2.45.0 (latest supported by ROS)
RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.50.0.zip  
RUN unzip v2.50.0.zip && cd librealsense-2.50.0/

# build librealsense
RUN mkdir -p /librealsense-2.50.0/build && cd /librealsense-2.50.0/build && cmake /librealsense-2.50.0 \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_PYTHON_BINDINGS=true \
    -DBUILD_PYTHON_EXECUTABLE=/usr/bin/python3.8 
 
RUN cd /librealsense-2.50.0/build && make && make install

RUN git clone https://github.com/IntelRealSense/realsense-ros ~/catkin_ws/src/realsense-ros
# build realsense-ros wrappe
RUN cd ~/catkin_ws/src/realsense-ros \
    && git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

# install ros-realsense 
RUN apt install --no-install-recommends -y libeigen3-dev \
    python3-catkin-tools \
    python3-osrf-pycommon \
    ros-noetic-catkin \
    ros-noetic-cv-bridge \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-image-transport \
    ros-noetic-diagnostic-updater \
    ros-noetic-tf \
    udev 

# build ros-realsense
RUN source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws/src/ \
    && catkin_init_workspace  \
    && cd /root/catkin_ws  \
    && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release \
    && catkin_make install

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN apt install --no-install-recommends -y udev
RUN cd /librealsense-2.45.0/ && ./scripts/setup_udev_rules.sh
RUN rm -rf /librealsense-2.45.0/

CMD source /root/catkin_ws/devel/setup.bash && \
    roslaunch realsense2_camera rs_camera.launch depth_width:=424 depth_height:=240 depth_fps:=60 color_width:=320 color_height:=240 color_fps:=60