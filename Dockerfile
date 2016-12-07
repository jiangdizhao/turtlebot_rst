FROM  osrf/ros:kinetic-desktop-full

MAINTAINER Christoph Rösmann <christoph.roesmann@tu-dortmund.de>

ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

### Install packages

# update apt-get because osrf image clears this cache and download deps 
RUN apt-get -qq update && \
    apt-get -qq install -y \
        git-core \
        python-catkin-tools \
        curl && \
    rm -rf /var/lib/apt/lists/*

### Copy current context to container

COPY . turtlebot_rst/

### Download additional packages from source to test the newest versions
### TODO: maybe use wstool here similar to https://hub.docker.com/r/moveit/moveit_docker/~/dockerfile/

RUN git clone -b ${ROS_DISTRO}-devel https://github.com/rst-tu-dortmund/teb_local_planner.git
RUN git clone -b master https://github.com/rst-tu-dortmund/costmap_converter.git

### Install additional ros dependencies

# since we should not call rosdep update with sudo, we create a non-root user
RUN useradd rosuser -m
RUN echo "rosuser:rosuser" | chpasswd
USER rosuser
RUN rosdep update

# switch back to root
USER root
RUN apt-get -qq update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*


### HACK, replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && ln -s /bin/bash /bin/sh


### build repo 
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8 
RUN source /ros_entrypoint.sh && \
    catkin build -i -s --no-status



