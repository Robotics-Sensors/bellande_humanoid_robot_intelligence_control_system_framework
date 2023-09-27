ARG ROS_ARCHITECTURE_VERSION=latest

FROM ubuntu:20.04 as base_build
SHELL [ "/bin/bash" , "-c" ]

ENV DEBIAN_FRONTEND noninteractive
ENV PYTHON_VERSION="3.8"

ARG ROS_ARCHITECTURE_VERSION_GIT_BRANCH=master
ARG ROS_ARCHITECTURE_VERSION_GIT_COMMIT=HEAD

LABEL maintainer=ronaldsonbellande@gmail.com
LABEL ROS_architecture_github_branchtag=${ROS_ARCHITECTURE_VERSION_GIT_BRANCH}
LABEL ROS_architecture_github_commit=${ROS_ARCHITECTURE_VERSION_GIT_COMMIT}

# Ubuntu setup
RUN apt-get update -y
RUN apt-get upgrade -y

# RUN workspace and sourcing
WORKDIR ./
COPY requirements.txt .
COPY system_requirements.txt .
COPY ros_requirements.txt .
COPY ros_repository_requirements.txt .

# Install dependencies for system
RUN apt-get update && apt-get install -y --no-install-recommends <system_requirements.txt \
  && apt-get upgrade -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Install python 3.8 and make primary 
RUN apt-get update && apt-get install -y \
  python3.8 python3.8-dev python3-pip python3.8-venv \
  && update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1

# Pip install update 
RUN pip3 install --upgrade pip

# Install python libraries
RUN pip --no-cache-dir install -r requirements.txt

# Install dependencies for ros system 
RUN apt-get update && apt-get install -y --no-install-recommends <ros_requirements.txt \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
  && <ros_repository_requirements.txt 

# Create local catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Initialize local catkin workspace, install dependencies and build workpsace
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN cd $CATKIN_WS \
  && rosdep init \
  && rosdep update \
  && rosdep update --rosdistro noetic \
  && rosdep fix-permissions \
  && rosdep install -y --from-paths . --ignore-src --rosdistro noetic

# Always source catkin_setup.sh when launching bash 
RUN echo "source /usr/local/bin/catkin_setup.sh" >> /root/.bashrc
COPY catkin_setup.sh /usr/local/bin/catkin_setup.sh
RUN chmod +x /usr/local/bin/catkin_setup.sh

ENTRYPOINT ["/usr/local/bin/catkin_setup.sh"]
CMD ["bash"]
