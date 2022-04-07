FROM ros:melodic-ros-base-bionic

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# 中科大源（比清华源快）
RUN echo "deb https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse \n\
  deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse \n\
  deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse \n\
  deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse \n\
  deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse \n\
  deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse \n\
  deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse \n\
  deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse \n\
  deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse \n\
  deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse" > /etc/apt/sources.list

# Configure apt and install packages
RUN apt-get update \
  && apt-get -y install --no-install-recommends apt-utils dialog 2>&1 \
  # Verify git, process tools, lsb-release (common in install instructions for CLIs) installed
  && apt-get -y install git iproute2 procps lsb-release \
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*

# install Pangolin Dependencies
RUN apt-get update \
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
  libjpeg-dev libtiff5-dev libopenexr-dev

# install opencv Dependencies
RUN apt-get update \
  && apt-get -y install build-essential \
  cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
  libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

# install dependencies: eigen3 Pangolin opencv
WORKDIR /orbslam2-dependencies
COPY /orbslam2-dependencies/ .

RUN cd eigen-3.3.5 \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make -j$(nproc) \
  && make install

RUN cd Pangolin \
  && mkdir build \
  && cd build \
  && cmake .. \
  && cmake --build . \
  && make install

RUN cd opencv-3.2.0 \
  && mkdir build \
  && cd build \
  && cmake -D CMAKE_BUILD_TYPE=Release BUILD_DOCS BUILD_EXAMPLES .. \
  && make -j$(nproc) \
  && make install

# install RealSense SDK
RUN \
  # Register the server's public key
  apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
  || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
  # install software-properties-common in order to use the command "add-apt-repository"
  && apt-get install software-properties-common -y \
  # Add the server to the list of repositories
  && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
  # Install the libraries
  && apt-get update && apt-get install -y \
  librealsense2-dkms librealsense2-utils

# install ros wrapper for realsense camera
# 1.安装缺失的依赖（cmake报错一个个试出来的）
RUN apt-get update \
  && apt-get install -y \
  ros-melodic-cv-bridge \
  ros-melodic-image-transport \
  ros-melodic-tf \
  ros-melodic-diagnostic-updater \
  librealsense2-dev \
  ros-melodic-rgbd-launch

# 2.正式安装 Intel RealSense ROS
# 默认的 shell 是 /bin/sh，使用 /bin/bash 才能用 source 命令（source 在 /bin/sh 下为 . ）
SHELL ["/bin/bash", "-c"]
# git clone 容易出网络问题（有时候开了代理也不行），所以直接本地下载好再 COPY 到容器里
WORKDIR /root/catkin_ws/src/
COPY realsense-ros ./realsense-ros
COPY ddynamic_reconfigure ./ddynamic_reconfigure

RUN cd realsense-ros/ \
  && git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1` \
  && cd .. \
  # 如果把 source /opt/ros/melodic/setup.bash 写到 .bashrc 里面，然后直接 source /root/.bashrc 是行不通的。因为 source /root/.bashrc 并不会在镜像的 build 过程中马上生效。
  # 所以需要直接 source 源文件，才能使用 catkin_init_workspace 命令。
  && source /opt/ros/melodic/setup.bash \
  #
  && catkin_init_workspace \
  && cd .. \
  && catkin_make clean \
  && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release \
  && catkin_make install

# Build ORB-SLAM2
WORKDIR /root/catkin_ws/ORB_SLAM2
# 复制源代码
COPY /ORB_SLAM2 .
RUN ./build.sh \
  && source /opt/ros/melodic/setup.bash \
  && source /root/catkin_ws/devel/setup.bash \
  && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/catkin_ws/ORB_SLAM2/Examples/ROS \
  # 执行完上面三句后，${ROS_PACKAGE_PATH} 的值应该为 /root/catkin_ws/src:/opt/ros/melodic/share:/root/catkin_ws/ORB_SLAM2/Examples/ROS
  && ./build_ros.sh \
  # 设置 ROS 相关的环境变量，保证每次启动终端 bash 都能找到到 ROS 相关的command
  && echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc \
  && echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc \
  && echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}" >> /root/.bashrc

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=