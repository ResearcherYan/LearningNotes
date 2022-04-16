> 本文主要介绍如何在树莓派上部署 orbslam2 环境

> CONTENT

- [orbslam2 环境部署](#orbslam2-环境部署)
  - [安装 orbslam2 的依赖](#安装-orbslam2-的依赖)
  - [安装 RealSense SDK](#安装-realsense-sdk)
  - [安装 ROS](#安装-ros)
  - [安装 RealSense ROS Package](#安装-realsense-ros-package)
- [参考链接](#参考链接)

## orbslam2 环境部署
### 安装 orbslam2 的依赖
把自己电脑上的三个依赖库传到树莓派上，路径为/home/ubuntu/orbslam2/orbslam2-dependencies/
- 安装 Pangolin 的依赖
  ```bash
  sudo apt-get update && sudo apt-get -y install cmake gcc g++ libgl1-mesa-dev libglew-dev libpython2.7-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libdc1394-22-dev libraw1394-dev libjpeg-dev libtiff5-dev libopenexr-dev
  ```
- 安装 opencv3 的依赖
  ```bash
  sudo apt-get update && sudo apt-get -y install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
  ```
  报错：
  ```bash
  The following packages have unmet dependencies:
  libgtk2.0-dev : Depends: libglib2.0-dev (>= 2.27.3) but it is not going to be installed
                  Depends: libgdk-pixbuf2.0-dev (>= 2.21.0) but it is not going to be installed
                  Depends: libpango1.0-dev (>= 1.20) but it is not going to be installed
                  Depends: libatk1.0-dev (>= 1.29.2) but it is not going to be installed
                  Depends: libcairo2-dev (>= 1.6.4-6.1) but it is not going to be installed
  E: Unable to correct problems, you have held broken packages.
  ```
  执行 `sudo apt-get install libglib2.0-dev`，报错：
  ```bash
  The following packages have unmet dependencies:
  libglib2.0-dev : Depends: libglib2.0-0 (= 2.64.6-1~ubuntu20.04.3) but 2.64.6-1~ubuntu20.04.4 is to be installed
                    Depends: libglib2.0-bin (= 2.64.6-1~ubuntu20.04.3) but 2.64.6-1~ubuntu20.04.4 is to be installed
  E: Unable to correct problems, you have held broken packages.
  ```
  执行 `sudo apt-get install libglib2.0-0=2.64.6-1~ubuntu20.04.3`，然后再安装 libglib2.0-dev：`sudo apt-get install libglib2.0-dev`，然后再安装所有 opencv3 的依赖：
  ```bash
  sudo apt-get update && sudo apt-get -y install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
  ```
- 安装 eigen 3.3.5
  ```bash
  cd /home/ubuntu/orbslam2/orbslam2-dependencies/eigen-3.3.5 && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install
  ```
- 安装 Pangolin
  ```bash
  cd /home/ubuntu/orbslam2/orbslam2-dependencies/Pangolin && mkdir build && cd build && cmake .. && cmake --build . && sudo make install
  ```
- 安装 opencv 3.2
  ```bash
  cd /home/ubuntu/orbslam2/orbslam2-dependencies/opencv-3.2.0 && mkdir build && cd build && cmake -D CMAKE_BUILD_TYPE=Release BUILD_DOCS BUILD_EXAMPLES .. && make -j$(nproc) && sudo make install
  ```

### 安装 RealSense SDK
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
&& sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
&& sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

### 安装 ROS


### 安装 RealSense ROS Package


## 参考链接
1. 本仓库下的 [Dockerfile](.devcontainer/Dockerfile)
2. [RealSense SDK Installation](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md)
3. [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
4. [RealSense ROS Package Installation](https://github.com/IntelRealSense/realsense-ros)