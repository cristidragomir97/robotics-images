


FROM cristidragomir97/rospack

RUN apt-get update && apt-get install -y locales && rm -rf /var/lib/apt/lists/* \
    && localedef -i en_US -c -f UTF-8 -A /usr/share/locale/locale.alias en_US.UTF-8
ENV LANG en_US.utf8
ENV LC_ALL en_US.utf8

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y  \
    qtdeclarative5-dev \
    qtbase5-dev \
    build-essential \
    cmake \
    python3-dev \
    python3 \
    python3-pip \
    python3-empy \
    python3-rospkg \
    python3-catkin-pkg \
    liblog4cxx-dev \
    libtinyxml2-dev \
    git \
    unzip \
    libopencv-dev \
    python3-opencv \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-perception \
    unzip \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
    && rm -rf /var/lib/apt/lists/* 

RUN  pip3 install defusedxml netifaces

RUN mkdir -p /orocos \ 
    && git clone https://github.com/orocos/orocos_kinematics_dynamics /orocos \
    && cd /orocos/orocos_kdl \
    && mkdir build && cd build \
    && cmake .. \
    && make && make install 

RUN mkdir -p ~/catkin_ws/src/ \
    && cd ~/catkin_ws/src/ \
    && git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam
    

RUN . /opt/ros/noetic/setup.sh \
    && cd /root/catkin_ws/src/ \
    && catkin_init_workspace  \
    && cd /root/catkin_ws  \
    && . /opt/ros/noetic/setup.sh \
    && catkin_make -j2 -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release  \
    && catkin_make install \
    # REMOVE ROS INSTALLATION AS WE'LL USE THE SHARED BINARIES ON RUNTIME
    && apt-get remove --purge -y build-essential cmake python3-pip git unzip python3-empy \
    && apt-get autoremove -y

