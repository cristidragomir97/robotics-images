FROM ubuntu:20.04

RUN apt-get update && apt-get install -y locales && rm -rf /var/lib/apt/lists/* \
    && localedef -i en_US -c -f UTF-8 -A /usr/share/locale/locale.alias en_US.UTF-8
ENV LANG en_US.utf8
ENV LC_ALL en_US.utf8

ARG DEBIAN_FRONTEND=noninteractive

# install bootstrap tools
COPY --from=cristidragomir97/rospack:latest /opt/ros/ /opt/ros/

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-dev \
    python3 \
    python3-pip \
    python3-empy \
    python3-rospkg \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-regex-dev \
    libboost-chrono-dev \
    libconsole-bridge-dev \ 
    python3-catkin-pkg \
    liblog4cxx-dev \
    libtinyxml2-dev \
    git \
    unzip \
    && rm -rf /var/lib/apt/lists/* 

RUN  pip3 install defusedxml netifaces simplejpeg tornado
# clone thing
RUN mkdir -p ~/catkin_ws/src/ \
    && cd ~/catkin_ws/src/ \
    && git clone https://github.com/dheera/rosboard

# build thing
RUN . /opt/ros/noetic/setup.sh \
    && cd /root/catkin_ws/src/ \
    && catkin_init_workspace  \
    && cd /root/catkin_ws  \
    && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release \
    && catkin_make install \
    # REMOVE ROS INSTALLATION AS WE'LL USE THE SHARED BINARIES ON RUNTIME
    && find /opt/ros -delete \
    && rm -rf /opt/ros \
    && apt-get remove --purge -y build-essential cmake python3-pip git unzip python3-empy \
    && apt-get autoremove -y

# SOURCE ROS ENVIRONMENT FROM SHARED VOLUMEs
# AND RUN THE APPLICATION
CMD . /opt/ros/noetic/setup.sh \
    && . /root/catkin_ws/devel/setup.sh \
    && rosrun rosboard rosboard_node
