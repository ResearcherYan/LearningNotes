#-------------------------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License. See https://go.microsoft.com/fwlink/?linkid=2090316 for license information.
#-------------------------------------------------------------------------------------------------------------
FROM ubuntu:bionic

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# This Dockerfile adds a non-root 'vscode' user with sudo access. However, for Linux,
# this user's GID/UID must match your local user UID/GID to avoid permission issues
# with bind mounts. Update USER_UID / USER_GID if yours is not 1000. See
# https://aka.ms/vscode-remote/containers/non-root-user for details.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse \n\
  deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse \n\
  deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse \n\
  deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse \n\
  deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse \n\
  deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse \n\
  deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse \n\
  deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse \n\
  deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse \n\
  deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse" > /etc/apt/sources.list


# Configure apt and install packages
RUN apt-get update \
  && apt-get -y install --no-install-recommends apt-utils dialog 2>&1 \
  #
  # Verify git, process tools, lsb-release (common in install instructions for CLIs) installed
  && apt-get -y install git iproute2 procps lsb-release \
  #
  # Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
  && groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  #
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*


# install Pangolin Dependencies
RUN apt-get update \
  && apt-get -y install cmake \
  gcc g++ \
  libgl1-mesa-dev \  
  #install openGL 
  libglew-dev \     
  #install Glew 
  libpython2.7-dev \
  # Recommended Dependencies
  && apt-get -y install pkg-config \                                          
  # install pkg-config 
  libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols \     
  # install Wayland and EGL 
  # Optional Dependencies for video input
  && apt-get -y install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \  
  #install FFMPEG(For video decoding and image rescaling) 
  libdc1394-22-dev libraw1394-dev \ 
  #install DC1394(For firewire input) 
  libjpeg-dev libtiff5-dev libopenexr-dev 
# libpng12-dev 安装失败
#install xxx (For reading still-image sequences

## install opencv Dependencies
RUN apt-get update \
  && apt-get -y install build-essential \
  cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
  libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev 
# libjasper-dev 安装失败

# install Pangolin opencv eigen3
WORKDIR /extendmodel
ADD extendmodel/ /extendmodel/

RUN cd Pangolin \
  && mkdir build \
  && cd build \
  && cmake .. \
  && cmake --build . \
  && make install

RUN cd eigen-3.3.5 \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make -j$(nproc) \
  && make install

RUN cd opencv-3.2.0 \
  && mkdir build \
  && cd build \
  && cmake -D CMAKE_BUILD_TYPE=Release BUILD_DOCS BUILD_EXAMPLES  .. \
  && make -j$(nproc) \
  && make install

# install RealSense SDK
RUN \
  # Register the server's public key
  sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
  || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
  # install software-properties-common in order to use the command "add-apt-repository"
  && sudo apt-get install software-properties-common -y \
  # Add the server to the list of repositories
  && sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
  # Install the libraries
  && sudo apt-get install -y \
  librealsense2-dkms librealsense2-utils

# install ROS
RUN \
  # enable Universe, Multiverse, Restricted repositories
  sudo add-apt-repository universe \
  && sudo add-apt-repository multiverse \
  && sudo add-apt-repository restricted \
  # 设置 sources.list，使用清华源
  && sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' \
  # 设置密钥
  && sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  # 安装
  && sudo apt-get update \
  && sudo apt-get install ros-melodic-desktop-full -y \
  # 初始化 rosdep
  && sudo rosdep init \
  && rosdep update \
  # 设置环境
  && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
  && source ~/.bashrc \
  # 安装 rosinstall
  && sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
# # 安装 roslaunch
# && sudo apt-get install ros-kinetic-rgbd-launch -y 

# # install ros wrapper for realsense camera
# # Create a catkin workspace Ubuntu
# WORKDIR /catkin_ws/src
# RUN \
#   # git clone the Intel RealSense ROS
#   git clone https://github.com/IntelRealSense/realsense-ros.git \
#   && cd realsense-ros/ \
#   && git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1` \
#   && cd .. \
#   # git clone the ros package ddynamic_reconfigure
#   && git clone https://github.com/pal-robotics/ddynamic_reconfigure.git \
#   # make & install
#   && catkin_init_workspace \
#   && cd .. \
#   && catkin_make clean \
#   && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release \
#   && catkin_make install \
#   # setup environment
#   && echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc \
#   && source ~/.bashrc

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=