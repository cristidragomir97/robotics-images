FROM cristidragomir97/roscore AS builder
ARG PACKAGES="libraspberrypi-bin"

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    python3-dev \
    python3 \
    python3-pip \
    python3-catkin-pkg \
    python3-catkin-tools \
    ros-noetic-catkin \
    libboost-dev \

    && rm -rf /var/lib/apt/lists/* 

# clone rosboard
RUN mkdir -p ~/catkin_ws/src/ \
    && cd ~/catkin_ws/src/ \
    && git clone https://github.com/UbiquityRobotics/raspicam_node


RUN mkdir -p /etc/ros/rosdep/sources.list.d/
RUN echo "yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml" > /etc/ros/rosdep/sources.list.d/30-ubiquity.list
# list of packages to be installed in rootfs we will use in next stage

# download all packages and dependencies
RUN apt update && apt-get install -y $(apt-cache depends --recurse --no-recommends --no-suggests --no-conflicts --no-breaks --no-replaces --no-enhances ${PACKAGES} | grep "^\w" | sort -u)
RUN pip3 install -U netifaces defusedxml empy rospkg rosinstall-generator wstool rosdep 

RUN export PATH="/root/.local/bin:$PATH" rosdep init && rosdep update 
RUN wstool init src

RUN . /opt/ros/noetic/setup.sh \
    && export PATH="/root/.local/bin:$PATH" \
    && cd /root/catkin_ws/src/ \
    && catkin_init_workspace  \
    && cd /root/catkin_ws  \
    && wstool init src \
    && rosinstall_generator compressed_image_transport --rosdistro noetic --wet-only --deps --tar > compressed_image_transport-wet.rosinstall \
    && rosinstall_generator camera_info_manager --rosdistro noetic --wet-only --deps --tar > camera_info_manager-wet.rosinstall \ 
    && rosinstall_generator dynamic_reconfigure --rosdistro noetic --wet-only --deps --tar > dynamic_reconfigure-wet.rosinstall \
    && rosinstall_generator diagnostics --rosdistro noetic --wet-only --deps --tar > diagnostics-wet.rosinstall \
    && wstool merge -t src compressed_image_transport-wet.rosinstall \
    && wstool merge -t src camera_info_manager-wet.rosinstall \
    && wstool merge -t src dynamic_reconfigure-wet.rosinstall \
    && wstool merge -t src diagnostics-wet.rosinstall \
    && wstool update -t src \
    && rosdep install --from-paths src --ignore-src --rosdistro=noetic -y \
    && export PATH="/root/.local/bin:$PATH" catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release 

FROM balenalib/raspberrypi4-64-debian as runtime 

RUN apt update && apt-get install -y $(apt-cache depends --recurse --no-recommends --no-suggests --no-conflicts --no-breaks --no-replaces --no-enhances libraspberrypi | grep "^\w" | sort -u)

RUN apt update && apt-get install -y \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-regex-dev \
    libboost-chrono-dev \
    libconsole-bridge-dev \
    liblog4cxx-dev \
    libtinyxml2-dev 

COPY --from=builder /root/.local /root/.local
COPY --from=builder /root/catkin_ws/src/ /root/catkin_ws/src/

RUN export PYTHONPATH="${PYTHONPATH}:/opt/ros/noetic/lib/python3/dist-packages"


CMD . /opt/ros/noetic/setup.sh && roslaunch raspicam_node camerav2_1280x960.launch