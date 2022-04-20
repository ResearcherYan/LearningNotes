> 本文主要介绍如何在树莓派上部署 orbslam2 环境

> CONTENT

- [试错 1](#试错-1)
  - [安装 orbslam2 的依赖](#安装-orbslam2-的依赖)
  - [安装 RealSense SDK](#安装-realsense-sdk)
- [试错 2](#试错-2)
  - [安装全部的依赖](#安装全部的依赖)
    - [安装 Pangolin 的依赖](#安装-pangolin-的依赖)
    - [安装 opencv3 的依赖](#安装-opencv3-的依赖)
    - [安装 RealSense SDK](#安装-realsense-sdk-1)
    - [安装 ROS](#安装-ros)
    - [安装 rslidar_sdk 的依赖](#安装-rslidar_sdk-的依赖)
    - [安装 ubuntu-desktop<br>](#安装-ubuntu-desktop)
  - [完成剩余的安装步骤](#完成剩余的安装步骤)
    - [安装 RealSense ROS Package](#安装-realsense-ros-package)
    - [安装 orbslam2](#安装-orbslam2)
    - [完成 rslidar_sdk 的安装](#完成-rslidar_sdk-的安装)
- [参考链接](#参考链接)

## 试错 1
### 安装 orbslam2 的依赖
把自己电脑上的三个依赖库传到树莓派上，路径为/home/ubuntu/orbslam2/orbslam2-dependencies/
- 安装 Pangolin 的依赖
  ```bash
  sudo apt-get install cmake gcc g++ libgl1-mesa-dev libglew-dev libpython2.7-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libdc1394-22-dev libraw1394-dev libjpeg-dev libtiff5-dev libopenexr-dev
  ```
- 安装 opencv3 的依赖
  ```bash
  sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
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
  执行 `sudo apt-get install libglib2.0-0=2.64.6-1~ubuntu20.04.3`，发现这个过程中会卸载掉一些现有的库，包括 gnome的一些库。然后再安装 libglib2.0-dev：`sudo apt-get install libglib2.0-dev`，然后再安装所有 opencv3 的依赖：
  ```bash
  sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
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
  报错：
  ```bash
  opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp:1541:21: error: ‘CODEC_FLAG_GLOBAL_HEADER’ was not declared in this scope; did you mean ‘AV_CODEC_FLAG_GLOBAL_HEADER’?
  ```
  参考[博客](https://blog.csdn.net/guo_lei_lamant/article/details/81568346#commentBox)，在 opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp 最顶端加入 3 行代码，然后重新编译，成功。

### 安装 RealSense SDK
参考 [Intel® RealSense™ D400 cameras with Raspberry Pi](https://github.com/IntelRealSense/librealsense/blob/master/doc/RaspberryPi3.md)，Intel 的关于树莓派的官方教程给的是 Ubuntu Mate 的例子，因为树莓派是 ARM 架构，只能用 Build from Source 的方法安装。
- Make Ubuntu Up-to-date（后面发现其实最好不要执行这一步，upgrade 之后内核会很新，很容易引发一些依赖冲突）
  ```bash
  sudo apt-get update && sudo apt-get upgrade
  ```
  发现用 upgradable 的包，但在终端并没有 upgrade，于是打开 software & update 手动 upgrade，发现有linux核更新，还有树莓派的核更新，更新完之后重启，发现无法连上 vnc，但可以连上 ssh。<br>
  首先想到的是，可能是因为更新了树莓派的核，导致 config.txt 文件被重写，所以没有 hdmi 负载的情况下树莓派不会启动桌面，结果 `sudo vim /boot/firmware/config.txt` 发现 config.txt 并没有重写。<br>
  于是就想到了前面在装 opencv 依赖的时候，执行 `sudo apt-get install libglib2.0-0=2.64.6-1~ubuntu20.04.3`后终端提示需要卸载掉一些现有的库，包括 gnome的一些库，所以很可能是这个导致 ubuntu-desktop 无法正常运行。<br>
  于是需要重新安装桌面：`sudo apt install ubuntu-desktop`，发现有依赖错误，逐步排查（这个过程需要 downgrade 一些软件包），最终成功安装，再重启 `sudo systemctl reboot` 之后就可以连接 vnc 了。
- Download/Clone librealsense github repository<br>
  ```bash
  mkdir -p /home/ubuntu/realsense && cd /home/ubuntu/realsense && git clone https://github.com/IntelRealSense/librealsense.git
  ```
- Prepare Linux Backend and the Dev. Environment
  ```bash
  cd /home/ubuntu/realsense \
  && sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
  ```
  报错：
  ```bash
  The following packages have unmet dependencies:
  libgtk-3-dev : Depends: gir1.2-gtk-3.0 (= 3.24.18-1ubuntu1) but 3.24.20-0ubuntu1.1 is to be installed
                  Depends: libatk-bridge2.0-dev but it is not going to be installed
                  Depends: libgtk-3-0 (= 3.24.18-1ubuntu1) but 3.24.20-0ubuntu1.1 is to be installed
  libudev-dev : Depends: libudev1 (= 245.4-4ubuntu3.15) but 245.4-4ubuntu3.16 is to be installed
  E: Unable to correct problems, you have held broken packages.
  ```
  根据依赖错误对软件包进行降级：
  ```bash
  sudo apt-get install gir1.2-gtk-3.0=3.24.18-1ubuntu1 libgtk-3-0=3.24.18-1ubuntu1 libudev1=245.4-4ubuntu3.15
  ```
  提示：
  ```bash
  The following packages were automatically installed and are no longer required:
  apg apturl-common bluez-obexd busybox-initramfs colord-data cryptsetup
  cryptsetup-bin cryptsetup-run devio dns-root-data dnsmasq-base finalrd
  fprintd gir1.2-goa-1.0 gir1.2-gst-plugins-base-1.0
  gir1.2-javascriptcoregtk-4.0 gir1.2-rb-3.0 gir1.2-snapd-1 gir1.2-webkit2-4.0
  gnome-control-center-faces gnome-online-accounts gnome-session-common
  gvfs-libs hplip-data initramfs-tools-bin klibc-utils libatasmart4
  libblockdev-crypto2 libblockdev-fs2 libblockdev-loop2 libblockdev-part-err2
  libblockdev-part2 libblockdev-swap2 libblockdev-utils2 libblockdev2
  libbluetooth3 libcolord-gtk1 libcolorhug2 libcue2 libdmapsharing-3.0-2
  libfprint-2-2 libfprint-2-tod1 libgif7 libgpod-common libgpod4 libgsf-1-114
  libgsf-1-common libgsound0 libgssdp-1.2-0 libgupnp-1.2-0 libgupnp-av-1.0-2
  libgupnp-dlna-2.0-3 libhpmud0 libieee1284-3 libimagequant0 libimobiledevice6
  libisns0 libjansson4 libklibc libldb2 liblirc-client0 libmtp-common
  libmtp-runtime libmtp9 libmysqlclient21 libndp0 libnfs13 libpam-fprintd
  libparted-fs-resize0 libpkcs11-helper1 libplist3 libplymouth5
  librhythmbox-core10 librygel-core-2.6-2 librygel-db-2.6-2
  librygel-renderer-2.6-2 librygel-server-2.6-2 libsane-common libsane-hpaio
  libsbc1 libsgutils2-2 libsmbclient libsnmp-base libsnmp35 libtalloc2
  libteamdctl0 libtevent0 libtracker-control-2.0-0 libtracker-miner-2.0-0
  libudisks2-0 liburcu6 libusbmuxd6 libvolume-key1 libwbclient0
  linux-sound-base lz4 mobile-broadband-provider-info mtd-utils mysql-common
  nautilus-data openvpn policykit-1-gnome ppp pptp-linux printer-driver-hpcups
  printer-driver-postscript-hp python3-dateutil python3-ldb
  python3-macaroonbakery python3-mako python3-olefile python3-pil
  python3-protobuf python3-renderpm python3-reportlab python3-reportlab-accel
  python3-rfc3339 python3-talloc python3-tz python3-xkit read-edid
  rhythmbox-data rygel samba-libs sg3-utils tracker tracker-extract
  tracker-miner-fs usbmuxd x11-apps x11-session-utils xbitmaps xfonts-scalable
  xinit xinput xwayland
  Use 'sudo apt autoremove' to remove them.
  Suggested packages:
    gvfs
  Recommended packages:
    libgtk-3-bin
  The following packages will be REMOVED:
    alsa-base apturl bluez brltty cloud-initramfs-copymods
    cloud-initramfs-dyn-netconf colord cryptsetup-initramfs flash-kernel gdm3
    gnome-bluetooth gnome-control-center gnome-disk-utility gnome-power-manager
    gnome-session-bin gnome-shell-extension-desktop-icons
    gnome-startup-applications gvfs gvfs-backends gvfs-daemons gvfs-fuse hplip
    i2c-tools initramfs-tools initramfs-tools-core kpartx libgtk-3-bin libsane
    libu2f-udev mdadm media-player-info multipath-tools nautilus nautilus-share
    network-manager network-manager-config-connectivity-ubuntu
    network-manager-gnome network-manager-openvpn network-manager-openvpn-gnome
    network-manager-pptp network-manager-pptp-gnome open-iscsi overlayroot
    plymouth plymouth-label plymouth-theme-spinner plymouth-theme-ubuntu-text
    pulseaudio-module-bluetooth rhythmbox rhythmbox-plugin-alternative-toolbar
    rhythmbox-plugins sane-utils sg3-utils-udev simple-scan snapd
    software-properties-gtk ubuntu-desktop ubuntu-desktop-minimal
    ubuntu-drivers-common ubuntu-minimal ubuntu-release-upgrader-gtk
    ubuntu-session udev udisks2 update-manager update-notifier upower xorg
    xserver-xorg xserver-xorg-core xserver-xorg-input-all
    xserver-xorg-input-libinput xserver-xorg-input-wacom xserver-xorg-video-all
    xserver-xorg-video-amdgpu xserver-xorg-video-ati xserver-xorg-video-fbdev
    xserver-xorg-video-nouveau xserver-xorg-video-radeon xserver-xorg-video-vesa
  The following packages will be DOWNGRADED:
    gir1.2-gtk-3.0 libgtk-3-0 libudev1
  0 upgraded, 0 newly installed, 3 downgraded, 80 to remove and 0 not upgraded.
  Need to get 2691 kB of archives.
  After this operation, 222 MB disk space will be freed.
  Do you want to continue? [Y/n]
  ```
  这里看到要卸载的软件包包括 gnome 和 network-manager，估计选择卸载的话可能桌面又会被卸掉（甚至可能 wifi 都不一定会自动连上？）。决定还是尝试一下，如果说只是桌面卸载了，到时候就把所有的包装完了之后再把桌面装回来。<br><br>
  果然，发现当把这些包卸载之后 wifi 都直接断掉了。看来只能换成 18 的系统试试了。<br>
  尝试安装 18 的系统，发现 Ubuntu 18 的树莓派系统都没法进登录界面。<br>
  尝试 21.10（本身就带桌面），发现安装之后经常卡死。<br>

---

## 试错 2
兜兜转转还是决定安装 Ubuntu 20.04 Server，但不马上装桌面，先把所有要用的依赖库装好（还要把 RealSense SDK 编译完，因为从源代码安装 RealSense SDK 可能会涉及到内核的一些东西）之后再装桌面。

### 安装全部的依赖
先安装好所有的依赖，是为了看看依赖包之间有无冲突。（尤其是与 gnome 桌面的依赖包有无冲突）

#### 安装 Pangolin 的依赖
  ```bash
  sudo apt-get install cmake gcc g++ libgl1-mesa-dev libglew-dev libpython2.7-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libdc1394-22-dev libraw1394-dev libjpeg-dev libtiff5-dev libopenexr-dev
  ```

#### 安装 opencv3 的依赖
  ```bash
  sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
  ```

#### 安装 RealSense SDK
- 安装 RealSense SDK 的依赖
  - Download/Clone librealsense github repository<br>
    ```bash
    mkdir -p /home/ubuntu/realsense && cd /home/ubuntu/realsense && git clone https://github.com/IntelRealSense/librealsense.git
    ```
  - Prepare Linux Backend and the Dev. Environment
    - Navigate to librealsense root directory
      ```bash
      cd /home/ubuntu/realsense/librealsense
      ```
    - Install the core packages required to build librealsense binaries and the affected kernel modules
      ```bash
      sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
      ```
    - Run Intel Realsense permissions script
      ```bash
      ./scripts/setup_udev_rules.sh
      ```
    <!-- - Build and apply patched kernel modules
      ```bash
      ./scripts/patch-realsense-ubuntu-lts.sh
      ``` -->
- Build RealSense SDK
  ```bash
  cd /home/ubuntu/realsense/librealsense && mkdir build && cd build \
  && cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true \
  && sudo make uninstall && make clean && make -j$(nproc) && sudo make install
  ```

#### 安装 ROS
- Configure your Ubuntu repositories<br>
  首先检查一下 Ubuntu 软件仓库设置有没有允许来自 "restricted", "universe", "multiverse" 这三种仓库的软件安装：`sudo vim /etc/apt/sources.list`，装好 Ubuntu 20.04 Server 之后应该是默认允许这三个仓库的软件安装。
- Setup your sources.list（用中科大的源）<br>
  ```bash
  sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
  ```
- Set up your keys<br>
  ```bash
  sudo apt install curl \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  ```
- Installation
  ```bash
  sudo apt update && sudo apt install ros-noetic-desktop-full
  ```
  安装 ros-noetic-desktop-full 之后发现它的依赖包括很多 gnome 的东西，心一悬，感觉后面装桌面的时候可能会有依赖冲突（到后来发现其实并没有冲突）。<br>
  在下载完成后大概安装到 70% 80% 的时候，ssh 突然断开，并且无法重连上，然后在路由器管理页面可以看到树莓派的 wifi 也断开了。估计是刚刚带桌面的 ROS 的依赖包 gnome 搞的鬼。<br>
  等了大概 30 min 后（确保后面百分之 20% 30% 安装完成），手动重启树莓派，在 hdmi 显示器端看到果然自动给装了 gnome 桌面，只不过是个简易版的，什么应用都没有，也没有 screen sharing。
- Environment setup
  ```bash
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
  && source ~/.bashrc
  ```
- Install Dependencies for building packages
  ```bash
  sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
  ```
- Test your installation（查看 ROS 的环境变量）
  ```bash
  printenv | grep ROS
  ```

#### 安装 rslidar_sdk 的依赖
- git clone<br>
  ```bash
  mkdir -p /home/ubuntu/rslidar && cd /home/ubuntu/rslidar \
  && git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git \
  && cd rslidar_sdk \
  && git submodule init \
  && git submodule update
  ```
- 安装除 ROS 以外的依赖<br>
  ```bash
  sudo apt-get install libyaml-cpp-dev libpcap-dev libprotobuf-dev protobuf-compiler
  ```

#### 安装 ubuntu-desktop<br>
前面在安装 ROS 的时候已经自动安装了一个简易版的 gnome 桌面，这里为了启动 screen sharing，决定还是装一个完整版的 gnome 桌面。<br>
```bash
sudo apt install ubuntu-desktop
```
安装完桌面后重启树莓派，并测试 realsense-viewer：在终端输入 `realsense-viewer`，SDK 正常运行。

### 完成剩余的安装步骤
#### 安装 RealSense ROS Package
这里参考 [Intel® RealSense™ D400 cameras with Raspberry Pi](https://github.com/IntelRealSense/librealsense/blob/master/doc/RaspberryPi3.md)，在 Perrequisites 这一节，只执行 kernel patches instructions 前面的步骤，因此应该是没有对系统的内核进行修改的。
- Create a catkin workspace
  ```bash
  mkdir -p ~/catkin_ws/src \
  && cd ~/catkin_ws/src/
  ```
- git clone
  ```bash
  git clone https://github.com/IntelRealSense/realsense-ros.git \
  && cd realsense-ros/ \
  && git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1` \
  && cd ..
  ```
- Install ros package *ddynamic_reconfigure*
  ```bash
  sudo apt install ros-noetic-ddynamic-reconfigure
  ```
- Build
  ```bash
  catkin_init_workspace
  cd ..
  catkin_make clean
  catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
  catkin_make install
  ```
- Setup environment variables
  ```bash
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc \
  && source ~/.bashrc
  ```

#### 安装 orbslam2
- 准备好源代码文件
  - 创建目录
    ```bash
    mkdir -p /home/ubuntu/orbslam2/orbslam2-dependencies/
    ```
  - 复制源代码<br>
    把自己电脑上的 eigen pangolin opencv 的源代码传输到文件夹 */home/ubuntu/orbslam2/orbslam2-dependencies/*，把 ORB_SLAM2 的源代码传输到文件夹 */home/ubuntu/orbslam2/*。
  - 修改 opencv 源代码<br>
    参考[博客](https://blog.csdn.net/guo_lei_lamant/article/details/81568346#commentBox)，在 opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp 最顶端加入下面 3 行代码
    ```hpp
    #define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
    #define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
    #define AVFMT_RAWPICTURE 0x0020
    ```
- 安装 eigen 3.3.5
  ```bash
  cd /home/ubuntu/orbslam2/orbslam2-dependencies/eigen-3.3.5 && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install
  ```
- 安装 Pangolin
  ```bash
  cd /home/ubuntu/orbslam2/orbslam2-dependencies/Pangolin && mkdir build && cd build && cmake .. && cmake --build . && sudo make install
  ```
- 安装 opencv 3.2<br>
  ```bash
  cd /home/ubuntu/orbslam2/orbslam2-dependencies/opencv-3.2.0 && mkdir build && cd build && cmake -D CMAKE_BUILD_TYPE=Release BUILD_DOCS BUILD_EXAMPLES .. && make -j$(nproc) && sudo make install
  ```
  - 报错 1
    ```
    [ 24%] Generating precomp.hpp.gch/opencv_viz_Release.gch
    In file included from /usr/include/c++/9/ext/string_conversions.h:41,
                    from /usr/include/c++/9/bits/basic_string.h:6496,
                    from /usr/include/c++/9/string:55,
                    from /usr/include/c++/9/stdexcept:39,
                    from /usr/include/c++/9/array:39,
                    from /usr/include/c++/9/tuple:39,
                    from /usr/include/c++/9/bits/stl_map.h:63,
                    from /usr/include/c++/9/map:61,
                    from /home/ubuntu/orbslam2/orbslam2-dependencies/opencv-3.2.0/build/modules/viz/precomp.hpp:49:
    /usr/include/c++/9/cstdlib:75:15: fatal error: stdlib.h: No such file or directory
      75 | #include_next <stdlib.h>
          |               ^~~~~~~~~~
    compilation terminated.
    make[2]: *** [modules/viz/CMakeFiles/pch_Generate_opencv_viz.dir/build.make:64: modules/viz/precomp.hpp.gch/opencv_viz_Release.gch] Error 1
    make[1]: *** [CMakeFiles/Makefile2:3419: modules/viz/CMakeFiles/pch_Generate_opencv_viz.dir/all] Error 2
    make[1]: *** Waiting for unfinished jobs....
    ```
    参考 [stackoverflow](https://stackoverflow.com/questions/40262928/error-compiling-opencv-fatal-error-stdlib-h-no-such-file-or-directory)，在编译时加入 `-DENABLE_PRECOMPILED_HEADERS=OFF`，解决。
    ```bash
    cd /home/ubuntu/orbslam2/orbslam2-dependencies/opencv-3.2.0 && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_PRECOMPILED_HEADERS=OFF BUILD_DOCS BUILD_EXAMPLES .. && make -j$(nproc) && sudo make install
    ```
  - 报错 2
    ```bash
    [100%] Built target opencv_test_calib3d
    /opencv-3.2.0/modules/python/src2/cv2.cpp:730:34: error: invalid conversion from 'const char*' to 'char*' [-fpermissive]
    ```
    参考 [stackoverflow](https://stackoverflow.com/questions/57289402/opencv-error-while-building-on-raspberry-pi)，修改 */opencv-3.2.0/modules/python/src2/cv2.cpp*，把 `char* str = PyString_AsString(obj);` 改为 `const char* str = PyString_AsString(obj);`。重新编译，成功。

- Build orbslam2
  - Build ORB_SLAM2 library and examples
    ```bash
    cd /home/ubuntu/orbslam2/ORB_SLAM2 \
    && chmod +x build.sh \
    && ./build.sh
    ```
    报错
    ```bash
    In file included from /usr/include/c++/9/map:61,
                    from /home/ubuntu/orbslam2/ORB_SLAM2/Thirdparty/DBoW2/DBoW2/BowVector.h:14,
                    from /home/ubuntu/orbslam2/ORB_SLAM2/include/Frame.h:27,
                    from /home/ubuntu/orbslam2/ORB_SLAM2/include/MapPoint.h:25,
                    from /home/ubuntu/orbslam2/ORB_SLAM2/include/KeyFrame.h:24,
                    from /home/ubuntu/orbslam2/ORB_SLAM2/include/LoopClosing.h:24,
                    from /home/ubuntu/orbslam2/ORB_SLAM2/src/LoopClosing.cc:21:
    /usr/include/c++/9/bits/stl_map.h: In instantiation of ‘class std::map<ORB_SLAM2::KeyFrame*, g2o::Sim3, std::less<ORB_SLAM2::KeyFrame*>, Eigen::aligned_allocator<std::pair<const ORB_SLAM2::KeyFrame*, g2o::Sim3> > >’:
    /home/ubuntu/orbslam2/ORB_SLAM2/src/LoopClosing.cc:438:21:   required from here
    /usr/include/c++/9/bits/stl_map.h:122:71: error: static assertion failed: std::map must have the same value_type as its allocator
      122 |       static_assert(is_same<typename _Alloc::value_type, value_type>::value,
          |                                                                       ^~~~~
    ```
    参考[博客](https://blog.csdn.net/lixujie666/article/details/90023059)，修改文件 */home/ubuntu/orbslam2/ORB_SLAM2/include/LoopClosing.h*。
    ```h
    // 修改前的代码
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
            Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
    // 修改后的代码
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
            Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > > KeyFrameAndPose;
    ```

  - Build ROS nodes
    ```bash
    cd /home/ubuntu/orbslam2/ORB_SLAM2 \
    && echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/ubuntu/orbslam2/ORB_SLAM2/Examples/ROS" >> ~/.bashrc \
    && source ~/.bashrc \
    && chmod +x build_ros.sh \
    && ./build_ros.sh
    ```
    - 报错 1
      ```bash
      [rosbuild] Building package ORB_SLAM2
      Failed to invoke /opt/ros/noetic/bin/rospack deps-manifests ORB_SLAM2
      [rospack] Error: the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'


      CMake Error at /opt/ros/noetic/share/ros/core/rosbuild/public.cmake:129 (message):
        

        Failed to invoke rospack to get compile flags for package 'ORB_SLAM2'.
        Look above for errors from rospack itself.  Aborting.  Please fix the
        broken dependency!

      Call Stack (most recent call first):
        /opt/ros/noetic/share/ros/core/rosbuild/public.cmake:207 (rosbuild_invoke_rospack)
        CMakeLists.txt:4 (rosbuild_init)


      -- Configuring incomplete, errors occurred!
      See also "/home/ubuntu/orbslam2/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/CMakeFiles/CMakeOutput.log".
      make: *** No targets specified and no makefile found.  Stop.
      ```
      根据提示，应该是装 ros 的时候没有 rosdep init。
      ```bash
      sudo rosdep init
      rosdep update
      ```
    - 报错 2<br>
      执行 `rosdep update` 的时候老是报错 read time out 的错误。<br>
      尝试 [博客 1](https://blog.csdn.net/qq_38649880/article/details/87903654) 里的做法，把 `DOWNLOAD_TIMEOUT` 调大一点，从 15 调到了 60，还是会报同样的错。<br>
      尝试 [博客 2](https://blog.csdn.net/weixin_41010198/article/details/109495305) 的做法，先 `apt update` 再 `sudo rosdep init` 和 `rosdep update`，也是一样报错。<br>
      观察到每次在哪一步 read time out 其实是有随机性的，所以也不是说某个网站就一定无法链接上，决定碰运气多试几次，再继续尝试两三次后终于全部链接上，成功。<br>
      然后重新 build ros node
      ```bash
      cd /home/ubuntu/orbslam2/ORB_SLAM2 \
      && chmod +x build_ros.sh \
      && ./build_ros.sh
      ```
    - 报错 3
      ```bash
      /usr/bin/ld: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.2.0: error adding symbols: DSO missing from command line
      ```
      看到 [ORB_SLAM3 Github Issue](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/279) 里有人也遇到了这个问题，是因为装 ROS 的时候没有从源代码安装，所以没有指定 ROS 使用的 opencv 的版本，默认安装的时候 ROS 用的是 opencv 4.2，而 ORB_SLAM2 的 *CMakeLists.txt* 里面写的是找 opencv 3，因此需要修改 *CMakeLists.txt* 来用 opencv 4.2 编译。<br>
      注意 *ORB_SLAM2* 源代码里面一共有**四个** *CMakeLists.txt*，分别是<br>
      *ORB_SLAM2/CMakeLists.txt*<br>
      *ORB_SLAM2/Examples/ROS/ORB_SLAM2/CMakeLists.txt*<br>
      *ORB_SLAM2/Thirdparty/DBoW2/CMakeLists.txt*<br>
      *ORB_SLAM2/Thirdparty/g2o/CMakeLists.txt<br>
      其中前三个是要用到 opencv 的，把这三个文件里的 `find_package(OpenCV 3.0 QUIET)` 改为 `find_package(OpenCV 4 QUIET)`。<br>
      前面执行 build.sh 的时候可能也到了 opencv，因此先重新执行一下 build.sh
      ```bash
      cd /home/ubuntu/orbslam2/ORB_SLAM2 \
      && chmod +x build.sh \
      && ./build.sh
      ```
    - 报错 4<br>
      因为编译时 opencv 版本从 3 变成了 4，在重新 build 的时候报了新的错误
      ```bash
      /home/ubuntu/orbslam2/ORB_SLAM2/Examples/Monocular/mono_kitti.cc:68:48: error: ‘CV_LOAD_IMAGE_UNCHANGED’ was not declared in this scope
        68 |         im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
            |                                                ^~~~~~~~~~~~~~~~~~~~~~~
      ```
      参考 [ORB_SLAM2 Github Issue](https://github.com/raulmur/ORB_SLAM2/issues/451#issuecomment-955154787) 的这个回答，先找到 ORB_SLAM2 文件夹下所有含有 CV_LOAD_IMAGE_UNCHANGED 的 .cc 或 .h 文件：`find -type f -name "*.cc" -or -name "*.h" | xargs -i grep -l "CV_LOAD_IMAGE_UNCHANGED" {}`，然后在这些文件里加入头文件 `#include <opencv2/imgcodecs/imgcodecs_c.h>`，发现还是不行。<br>
      参考这个人前面的人的回答：在刚刚找到的这些文件里加上 `#define CV_LOAD_IMAGE_UNCHANGED -1`。尝试后发现 build 成功。<br>
      完成 build 之后，重新 build ros node，成功。

- 测试 ORB_SLAM2 的 RGBD demo（使用 realsense D455）<br>
  打开三个终端
  - `roscore`
  - `roslaunch realsense2_camera rs_camera.launch`
  - `rosrun ORB_SLAM2 RGBD /home/ubuntu/orbslam2/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/ubuntu/orbslam2/ORB_SLAM2/Examples/ROS/ORB_SLAM2/AsusD455.yaml`

#### 完成 rslidar_sdk 的安装
- 打开工程内的 *CMakeLists.txt* 文件，将文件顶部的 `set(COMPILE_METHOD ORIGINAL)` 改为 `set(COMPILE_METHOD CATKIN)`。
- 将 rslidar_sdk 工程目录下的 *package_ros1.xml* 文件复制到 *package.xml*。
- 将 rslidar_sdk 工程放入 *~/catkin/src* 文件夹内。
- 返回工作空间目录，用 catkin 编译 rslidar_sdk 源代码。
  ```bash
  catkin_make \
  && source devel/setup.bash \
  ```
- 运行 rslidar_sdk
  ```bash
  roslaunch rslidar_sdk start.launch
  ```

## 参考链接
1. 本仓库下的 [Dockerfile](.devcontainer/Dockerfile)
2. [Intel® RealSense™ D400 cameras with Raspberry Pi](https://github.com/IntelRealSense/librealsense/blob/master/doc/RaspberryPi3.md)
3. [ReanSense SDK Installation -- Built from Source](https://github.com/IntelRealSense/librealsense/blob/development/doc/installation.md)
4. [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
5. [RealSense ROS Package Installation](https://github.com/IntelRealSense/realsense-ros)
6. [rslidar_sdk 安装](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/README_CN.md#rslidar_sdk)