FROM ros:melodic-ros-base-bionic

# 中科大源（比清华源快）
RUN echo "deb https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse \n \
  deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse \n \
  deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse \n \
  deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse \n \
  deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse \n \
  deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse \n \
  deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse \n \
  deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse \n \
  deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse \n \
  deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse" > /etc/apt/sources.list

# Configure apt and install packages
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
  && apt-get -y install --no-install-recommends apt-utils dialog 2>&1 \
  # Verify git, process tools, lsb-release (common in install instructions for CLIs) installed
  && apt-get -y install git iproute2 procps lsb-release \
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/* \
  # unset the environment variable DEBIAN_FRONTEND
  && unset DEBIAN_FRONTEND

# install Pangolin Dependencies
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
  && apt-get -y install cmake \
  gcc g++ \
  libgl1-mesa-dev \  
  # install openGL 
  libglew-dev \     
  # install Glew 
  libpython2.7-dev \
  # Recommended Dependencies
  && apt-get -y install pkg-config \                                          
  # install pkg-config 
  libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols \     
  # install Wayland and EGL 
  # Optional Dependencies for video input
  && apt-get -y install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \  
  # install FFMPEG (For video decoding and image rescaling) 
  libdc1394-22-dev libraw1394-dev \ 
  # install DC1394 (For firewire input) 
  libjpeg-dev libtiff5-dev libopenexr-dev \
  # unset the environment variable DEBIAN_FRONTEND
  && unset DEBIAN_FRONTEND

# install opencv Dependencies
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
  && apt-get -y install build-essential \
  cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
  libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev \
  # unset the environment variable DEBIAN_FRONTEND
  && unset DEBIAN_FRONTEND

# install eigen3 Pangolin opencv
WORKDIR /orbslam2-dependencies
COPY /orbslam2-dependencies/ .

# RUN cd eigen-3.3.5 && export DEBIAN_FRONTEND=noninteractive \
#   && mkdir build \
#   && cd build \
#   && cmake .. \
#   && make -j$(nproc) \
#   && make install \
#   # unset the environment variable DEBIAN_FRONTEND
#   && unset DEBIAN_FRONTEND

# RUN cd Pangolin && export DEBIAN_FRONTEND=noninteractive \
#   && mkdir build \
#   && cd build \
#   && cmake .. \
#   && cmake --build . \
#   && make install \
#   # unset the environment variable DEBIAN_FRONTEND
#   && unset DEBIAN_FRONTEND

# RUN cd opencv-3.2.0 && export DEBIAN_FRONTEND=noninteractive \
#   && mkdir build \
#   && cd build \
#   && cmake -D CMAKE_BUILD_TYPE=Release BUILD_DOCS BUILD_EXAMPLES .. \
#   && make -j$(nproc) \
#   && make install \
#   # unset the environment variable DEBIAN_FRONTEND
#   && unset DEBIAN_FRONTEND

# # install RealSense SDK
# RUN export DEBIAN_FRONTEND=noninteractive \
#   # Register the server's public key
#   apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
#   || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
#   # install software-properties-common in order to use the command "add-apt-repository"
#   && apt-get install software-properties-common -y \
#   # Add the server to the list of repositories
#   && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
#   # Install the libraries
#   && apt-get install -y \
#   librealsense2-dkms librealsense2-utils \
#   # unset the environment variable DEBIAN_FRONTEND
#   && unset DEBIAN_FRONTEND

# # install ros wrapper for realsense camera
# # Create a catkin workspace Ubuntu
# WORKDIR /catkin_ws/src
# RUN export DEBIAN_FRONTEND=noninteractive \
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
#   && source ~/.bashrc \
#   # unset the environment variable DEBIAN_FRONTEND
#   && unset DEBIAN_FRONTEND
