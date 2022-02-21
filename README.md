
# Modular robotics with balena 

Last year, Cannonical published an article on [6 reasons ](https://ubuntu.com/blog/ros-docker) why Docker and ROS are not a good fit. To summarize, here are the main points of the article:
* Privileged containers are a security risk 
* Configuring networking is challenging 
* No notifications for updating vulnerable software packages
* No transactional updates for ROS on Docker
* No delta updates for applications
* Fleet management is a pain. 

While the last four would be solved just by running your robotics stack on balena, the first two need some extra attention.  

This repository is not meant to be an extensive library of ROS software packages that were modified to run on docker, but more of a guide on how to write your own. 

However, at the end of this document, you'll find a list of pre-made images for some common hardware like cameras and LIDAR sensors and applications like SLAM and object detection. 


## ROSCore
One instance of this image required to run anything ROS-related on balena devices. Instead of a having full copy of ROS for each service, we define a volume mount share.

It's main functionality however is to run the`roscore`process. This process keeps track of the nodes that are running, how to reach them, and what their message types are. 

Additionally, ROSCore makes it possible that ROS nodes can all to talk to each other even if they are part of different services/containers. For security reasons, there's a separate internal network for the services. However, you can open whatever port you want to the external world. 

#### Intermission: ROS networking

##### 1. Set environment variables for ROSCore container
For multi-machine setups, or multi-container in our case, ROS requires two environment variables to be set, one is `ROS_HOSTNAME`, which is the hostname of the service in question. Since docker engine allows services get resolved by their name, you need to set it to your service name. `ROS_MASTER_URI` is basically the hostname of the machine/service that runs roscore. 
 
##### 2. Add links
After `ROS_MASTER_URI` has been set, services will able to send messages to the core process, but the reverse it's not true, core has no way to know where to send the callback. To solve this issue we need to set links to help resolve them. 

Here is an example of a  `docker-compose.yaml` file for a robotics solution:
```
version:  '2.1'

volumes:
	shered-ros-bin:

services:
 -  roscore:
	   build: cristidragomir97/roscore
	   environment: 
		- ROS_HOSTNAME=roscore
		- ROS_MASTER_URI=http://roscore:11311
	    links: 
	      - SERVICE_0
	      - SERVICE_1
	    volumes:
	      - shered-ros-bin:/opt/ros/noetic
```

##### 2. Set variables for ROSCore container
you need to set  `ROS_HOSTNAME` and `ROS_MASTER_URI=http://roscore:11311` for each service.  You can either define it in the `docker-compose.yml`  or use the dashboard. 

##### 3. (Optional) Expose ports to talk to the outside world

## Creating ROS images for docker

### 1. Create a builder image based on ROSCore  
This will give you a bare-bones ROS installation together with all the tools you need to build packages. From here onward, it's like compiling a package on any regular ROS installation you have used.  

`FROM cristidragomir97/roscore AS builder`

### 2. Create the build environment and  clone the ROS package repository
If you are reading this, you probably want to do one of the following, either containerize your own ROS package, or convert an existing application for easier deployment and fleet management. 

In order to do that you need to create a workspace for catkin, ROS' build system, and clone the repository containing the package in question. 

```bash
mkdir -p ~/catkin_ws/src/ \
&& cd ~/catkin_ws/src/ \`
&& git clone https://github.com/YOUR_REPOSITORY
```

### 2. Install dependencies 
This is probably the most tricky part of this whole process. We want to be able to layer any number of ROS images on top of the ROSCore container and share common binaries to save space. Therefore, we want to bundle all the dependencies together with our ROS package. We'll use `wstool` and `rosinstall_generator` to pull the source of our dependencies and import them into our `catkin_ws` folder.

#### 2.1. Initialize the toolkit: 
```bash
cd /root/catkin_ws/src && catkin_init_workspace 
```
```bash
rosdep init && rosdep update
```
```bash
cd /root/catkin_ws && wstool init src 
```
#### 2.2. Check `package.xml` 
An important file for every ROS package is it's manifest, the `package.xml` file. You can find all the dependencies the package use

Here's a snippet from the `package.xml` file of ... :
```xml
<?xml version="1.0"?>
	....
	<depend>compressed_image_transport</depend>
	<depend>roscpp</depend>
	<depend>std_msgs</depend>
	<depend>std_srvs</depend>
	<depend>sensor_msgs</depend>
	<depend>camera_info_manager</depend>
	<depend>dynamic_reconfigure</depend>
	<depend>diagnostic_updater</depend>
	<depend>libraspberrypi0</depend>
	....
```

Ok, let's go trough these and figure out what's what. `roscpp`, `std_msgs`, `std_srvs`, `sensor_msgs` are all part of the core ROS installation, they are already taken care of. `libraspberrypi0` is an external dependency that needs to be installed from apt.

So, the packages we are concerned are: `camera_info_manager`, `dynamic_reconfigure`, `diagnostic_updater`, `compressed_image_transport`. 
Let's see how we can install them. 

#### 2.3. Add dependencies to source folder 
Move into the root of our workspace:
```bash
cd /root/catkin_ws 
```
At this point we can use `rosinstall_generator` to pull the source of all the required packages and `wstool` to add it to the build list. 


```bash
rosinstall_generator compressed_image_transport --rosdistro noetic --tar > compressed_image_transport.rosinstall 
wstool merge -t src compressed_image_transport.rosinstall
```

```bash
rosinstall_generator camera_info_manager --rosdistro noetic --tar > camera_info_manager.rosinstall 
wstool merge -t src camera_info_manager.rosinstall
```
```bash
rosinstall_generator dynamic_reconfigure --rosdistro noetic --tar > dynamic_reconfigure.rosinstall 
wstool merge -t src dynamic_reconfigure.rosinstall
```
```bash
rosinstall_generator diagnostics --rosdistro noetic --tar > diagnostics.rosinstall
wstool merge -t src diagnostics.rosinstall 
```

```bash
wstool update -t src
```
All these commands prepare catkin to build all the dependencies before building our package.

Not all dependencies are ROS packages,  but we can use rospack to install any other system libraries we might be missing.  
```bash 
rosdep install --from-paths src --ignore-src --rosdistro=noetic --os=ubuntu:focal -y 
```

We are finally ready to build everything 

### 3. Build everything
If you are trying to build a package found on the web, you might want to check their official repository for more accurate build and installation instructions. If you are working on your own code, this is a good place to start.

```bash 
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release 
catkin_make install 
```

### 4.  Create runtime image
**4.1. Start from a base image**
```bash 
FROM balenalib/rpi-raspbian:bullseye as runtime
```

**4.2. Move artifacts from the to runtime image.**
I usually move the whole catkin workspace to the runtime image, but you could get more granular if you want to save more space. However that requires some deeper knowledge about catkin and the package you are trying to build. 

```bash 
COPY --from=builder /root/catkin_ws/src/ /root/catkin_ws/src/
```

### 5. Install runtime dependencies 
First off, you will need the following libraries to be able to use the shared ROS binaries. Every balenified ROS image needs these libraries:   

```bash
install_packages \
	libboost-thread-dev \
	libboost-program-options-dev \
	libboost-filesystem-dev \
	libboost-regex-dev \
	libboost-chrono-dev \
	libconsole-bridge-dev \
	liblog4cxx-dev \
	libtinyxml2-dev 
```



```bash
pip3 install defusedxml netifaces
```


### 6. Set entrypoint 

Congrats, if you followed along with this, you should have a ready to deploy ROS package for balena. 

# Robotics Images

## Hardware Abstraction
These images are meant to simplify working with complex external hardware like LIDARs, cameras, that usually come with a ROS package from their vendors. 

#### Raspberry Pi Camera 
Thhttps://github.com/UbiquityRobotics/raspicam_node

#### RPLidar
Based on SLAMTEC's official [ROS package](https://github.com/Slamtec/rplidar_ros), this image provides plug and play functionality for the RPLidar series of LIDAR sensors. On runtime, you'll see the information from this sensor on the `/scan` topic. 

#### Intel Realsense 

## Applications
#### ROSboard
[ROSboard](https://github.com/dheera/rosboard) is a very useful tool that allows you to graph and visualize ROS information in real time. This provides the same functionality as RViz but it's in a web browser.

**Note:** Streaming video, pointcloud and other similar messages is quite computationally intensive. 

### HectorSLAM


