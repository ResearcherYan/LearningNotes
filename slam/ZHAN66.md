> 本文用于记录 ZHAN 66 SLAM 开发环境的安装过程

- [Ubuntu 22.04 LTS](#ubuntu-2204-lts)
  - [VLP-16](#vlp-16)
  - [尝试安装 ROS Noetic](#尝试安装-ros-noetic)
    - [下载源代码 + 安装依赖](#下载源代码--安装依赖)
    - [编译](#编译)
    - [收尾工作](#收尾工作)
- [Ubuntu 20.04 LTS](#ubuntu-2004-lts)
  - [升级内核](#升级内核)
  - [安装软件包](#安装软件包)
  - [尝试用电脑同时连接小车底盘和相机](#尝试用电脑同时连接小车底盘和相机)
  - [尝试同时使用相机和激光雷达](#尝试同时使用相机和激光雷达)
  - [卸载 Ubuntu 22.04](#卸载-ubuntu-2204)

# Ubuntu 22.04 LTS
首先，2022年的这款 ZHAN 66，用的是高通的一个比较新的无线网卡，Ubuntu 20.04 最新的固件版本也不支持它，因此被迫换到 Ubuntu 22.04，硬件适配上一切都好，但唯一不好的一点就是 Ubuntu 22 只能支持 ROS2，而目前 SLAM 开源社区用的基本上都是 ROS1，这就得自己把源代码 ROS1 的东西迁移到 ROS2。

## VLP-16
- **安装驱动**<br>
  首先，截止今天（2022.7.4），ROS2 社区还没有把 humble 版本的 velodyne 驱动以包的形式发布出来，只有 github 上的[驱动源码](https://github.com/ros-drivers/velodyne/tree/ros2)，于是只能 git clone 到本地然后手动编译。
  ```bash
  cd ~/dev_ws/src && git clone -b ros2 https://github.com/ros-drivers/velodyne.git
  cd .. && rosdep install --from-paths src --ignore-src --rosdistro humble -y -r
  colcon build --packages-select velodyne velodyne_laserscan velodyne_msgs velodyne_driver velodyne_pointcloud
  ```
  注意如果在用 rosdep 安装完依赖之后，直接用 `colcon build --packages-select velodyne`，会报依赖错误，因为 `velodyne` 文件夹下面还有个 `velodyne`，如果用前面的那个命令，系统默认识别要编译的是那个子文件夹，而那个子文件夹又依赖于其他四个包 `velodyne_laserscan velodyne_msgs velodyne_driver velodyne_pointcloud`，因此这里应该同时把这个五个子文件夹一起编译。
- **配置 IP**<br>
  参考 [Getting Started with the Velodyne VLP16 步骤 1.1](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16#Configure_your_computer.2BIBk-s_IP_address_through_the_Gnome_interface)，将电脑有线网卡的 IP 固定为 192.168.1.102（为了与速腾的雷达也兼容）
- **可视化雷达数据**<br>
  ROS2 的 velodyne 包跟 ROS1 差别还不小，因此 ROS1 官网的教程用不了。参考 [CSDN](https://blog.csdn.net/qq_45701501/article/details/119275134)，得到以下 ROS2 命令来可视化雷达数据
  ```bash
  ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
  ros2 topic echo /velodyne_points # 第二个终端
  ros2 run rviz2 rviz2 -f velodyne # 第三个终端
  ```
  然后在 rviz 里选择 Add -> By Topic -> /velodyne_points/pointcloud2，即可看到雷达扫描到的数据。（如果用的是 `ros2 run rviz2 rviz2` 打开 rviz，还需要手动修改 Fixed Frame 为 velodyne，才能看到点云）
- **录制**<br>
  ```bash
  ros2 bag record -o out /velodyne_points
  ```
  与 ROS1 的 `rosbag record` 不同，ROS2 的上述命令会在 `./out` 目录下生成一个 `metadata.yaml` 和一个 `out_0.db3` 文件，不再是 `.bag` 文件了。

## 尝试安装 ROS Noetic
> 参考链接: [Installing from source](http://wiki.ros.org/noetic/Installation/Source)

考虑到 SLAM 开源社区基本都是用的 ROS1，并且实验室其他两人都用的是 ROS1，如果用 ROS2 后面工作交接起来也比较麻烦，还是决定尝试装一下 ROS1。

### 下载源代码 + 安装依赖
```bash
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
rosdep update
mkdir ~/catkin_ws && cd ~/catkin_ws
rosinstall_generator desktop_full --rosdistro noetic --deps --tar > noetic-desktop-full.rosinstall
mkdir ./src
vcs import --input noetic-desktop-full.rosinstall ./src
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
```
这一步遇到了一个依赖上的问题
```
WARNING: given --rosdistro noetic but ROS_DISTRO is "humble". Ignoring environment.
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
diagnostic_common_diagnostics: [hddtemp] defined as "not available" for OS version [*]
```
尝试 `sudo apt insstall hddtemp` 发现找不到这个包，然后在 [ask ubuntu](https://askubuntu.com/questions/1403900/how-to-install-exfat-utils-and-hddtemp-on-ubuntu-22-04) 上找到一个安装方法，但安装后仍会报上面的错误。顺着 [ask ubuntu](https://askubuntu.com/questions/1403900/how-to-install-exfat-utils-and-hddtemp-on-ubuntu-22-04) 里给的 [hddtemp](https://manpages.ubuntu.com/manpages/jammy/man8/hddtemp.8.html) 包的链接，发现原来这个包的作用是检测机械硬盘温度的，但 ZHAN 66 本身就只有个固态，所以也难怪这个包找不到。<br>
果断决定卸载手动装的 `hddtemp`，并忽略这个依赖错误，继续装上其他的包
```bash
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y -r
```

### 编译
```bash
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
```
直接执行上面这条命令会报错
```
ROS_DISTRO was set to 'humble' before. Please make sure that the environment does not mix paths from different distributions.
```
将 `~/.bashrc` 里关于设置 ROS2 环境变量的语句注释掉，然后 `source ~/.bashrc`，重启一个终端
```
cd ~/catkin_ws
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
```
报错
```
error: cannot convert ‘ros::console::impl::ROSConsoleStdioAppender*’ to ‘log4cxx::AppenderPtr’ {aka ‘std::shared_ptr<log4cxx::Appender>’}
```
参考 [reddit](https://www.reddit.com/r/ROS/comments/p62vs6/ros_on_arch_linux_not_downloading/) 上的回答，尝试安装了 gcc/g++-9 gcc/g++-5，均无果，仍会在这里报错。<br>
其中安装 gcc-9 可以直接用 `sudo apt install gcc-9`，但 gcc-5 不在 ubuntu 22 的软件源里，要通过手动添加 ubuntu 16 的软件源：`sudo gedit /etc/apt/sources.list`，然后加入下面两行
```
deb http://mirrors.ustc.edu.cn/ubuntu/ xenial main
deb http://mirrors.ustc.edu.cn/ubuntu/ xenial universe
```
随后
```bash
# 添加公钥
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 40976EAF437D05B5
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 3B4FE6ACC0B21F32
# 更新软件源并安装 gcc-5
sudo apt update
sudo apt install gcc-5
```
使用非默认的 gcc 版本时，cmake 只能找到 python2 的解释器，用 `-DCMAKE_PREFIX_PATH=/usr/bin/python3` 和 `-DPythonInterp_FIND_VERSION=3` 都没用，只能通过指定解释器路径让 cmake 找到 python3 的解释器：`-DPYTHON_EXECUTABLE=/usr/bin/python3`。

参考 [reddit](https://www.reddit.com/r/ROS/comments/umrquk/ros_on_ubuntu_2204/) 的另一个 post，有人说只要不用 log4cxx 作为 rosconsole 的后端，就可以正常使用在 ubuntu 22 上编译的 noetic。于是决定忽略这个编译错误，还是用默认的 gcc-11 完成剩下的编译。<br>
但实际操作时发现有困难，参考 [how to ignore errors/keep going using cmake](https://cmake.org/pipermail/cmake/2011-January/041730.html)，只有 make 能够忽略编译错误继续执行编译，cmake 没有提供这种选项，只有在源代码里用的是 *SEND_ERROR* 而非 *FATAL_ERROR* 的时候，才可以实现有编译错误继续执行。

---

只能放弃从源代码编译的思路。在 [ROS answers](https://answers.ros.org/question/399664/will-ros-noetic-support-ubuntu-2204/) 和 [reddit](https://www.reddit.com/r/ROS/comments/umrquk/ros_on_ubuntu_2204/) 都看到有人说可以用 apt 安装 debian 系列的 ROS1（看 ID 好像是一个人回答的），两个回答都提到了可能还需要将部分包从源代码编译（在 [https://github.com/ros-o](https://github.com/ros-o) 和一些官方 github 仓库上获取源代码），这就很麻烦，而且从前面的经验可以看到基于源代码编译很容易崩。<br><br>
所以最终，选择放弃安装 ROS1，直接上 ROS2（起码是 offically supported）

### 收尾工作
- 卸载 gcc/g++-5 和 gcc/g++-9
```
sudo apt remove gcc-5 gcc-9 g++-5 g++-9
sudo update-alternatives --remove gcc /usr/bin/gcc-5
sudo update-alternatives --remove gcc /usr/bin/gcc-9
sudo update-alternatives --remove g++ /usr/bin/g++-5
sudo update-alternatives --remove g++ /usr/bin/g++-9
```
- 删除 ubuntu 16 的公钥
```
sudo apt-key del 40976EAF437D05B5
sudo apt-key del 3B4FE6ACC0B21F32
```
- 删除 ubuntu 16 的软件源：`sudo gedit /etc/apt/sources.list`，删掉 xenial 相关的源。
- 删除 catkin_ws：`rm -rf ~/catkin_ws`
- 无法还原的操作：前面通过 rosdep 安装的包不知道有哪些了，因此无法卸载他们，不过应该没什么影响。

# Ubuntu 20.04 LTS
经过详细和项目组其他成员反复讨论之后，决定还是统一用 ROS1，而进一步尝试在 ubuntu 22 上安装 ROS1 会有很大风险，没有官方支持。因此尝试再装一个 ubuntu 20，从 win11 下面划 200G，最小化安装。

## 升级内核
ubuntu 20 默认内核是 5.13，把内核升级到 5.15 之后，可以解决无线网卡驱动问题。<br>
首先，查看已安装的 5.13 内核有哪些库文件
```bash
dpkg --get-selections | grep linux
```
发现与 5.13 内核相关的库有
```bash
linux-headers-5.13.0-52-generic # 核心文件
linux-hwe-5.13-headers-5.13.0-52
linux-hwe-5.13-tools-5.13.0-52
linux-image-5.13.0-52-generic # 核心文件？
linux-modules-5.13.0-52-generic # 核心文件
linux-modules-extra-5.13.0-52-generic # 网卡驱动关键文件
linux-tools-5.13.0-30-generic
```
最后一个 5.13.0-30 是最初的内核版本，upgrade 之后到 5.13.0-52，但 linux-tools 这个包没更新。<br>
照着安装 5.15 内核（33 是通过 tab 试出来的）
```bash
sudo apt install linux-headers-5.15.0-33-generic \
linux-hwe-5.15-headers-5.15.0-33 \
linux-hwe-5.15-tools-5.15.0-33 \
linux-image-5.15.0-33-generic \
linux-modules-5.15.0-33-generic \
linux-modules-extra-5.15.0-33-generic \
linux-tools-5.15.0-33-generic
```

## 安装软件包
**换源**：在 Software & Updates 里换成中科大的源<br><br>
**安装常用软件**
- electron-ssr: 先要 `sudo apt install python`，用 `dpkg` 装完之后再用 `sudo apt -f install` 来补装需要的依赖
- 搜狗输入法: 进入 [搜狗输入法 linux](https://shurufa.sogou.com/linux) 下载 deb 包，用 `dpkg` 装完之后再用 `sudo apt -f install` 来补装需要的依赖，再参考 [搜狗输入法 linux 安装指南](https://shurufa.sogou.com/linux/guide)，将语言选择为 fcitx，重启，输入法 -> configure -> add -> 搜索 sogou -> 把搜狗输入法放到首位，进入搜狗输入法悬浮框里的设置，默认为英文并隐藏悬浮框。（如果有什么问题可以手动安装一下安装指南里的依赖再试试）
- ROS Noetic（也用中科大源）
- chrome（firefox 走代理有点小问题，懒得深究，直接用 chrome）
- brave
  - 由于 brave 站点在外网，用 `curl` 和 `sudo apt update` 的时候都要求超级用户走代理，参考自己之前写的 [proxy](https://github.com/ResearcherYan/LearningNotes/blob/master/general/proxy.md) 笔记设置普通用户和超级用户代理
  - 安装完后，取消掉超级用户代理，不然 `sudo apt install` 用中科大的源都还要走代理
  - 安装完后，删掉 brave 相关的源: `sudo rm /etc/apt/sources.list.d/brave*`，否则每次 `sudo apt update` 的时候都会卡在 brave 那里
- 卸载 firefox, fluid
- vim
- vscode: 登录 github 账户，同步所有设置
- gnome-tweak-tool: `sudo apt install gnome-tweak-tool`。然后按 super 搜索 tweak，使显示电源百分比，隐藏桌面的主文件夹和回收站图标。
- deepin-wine: 先安装[新 deepin-wine](https://github.com/zq1997/deepin-wine)，然后安装[微信](https://github.com/ResearcherYan/LearningNotes/blob/master/general/linux.md#%E5%AE%89%E8%A3%85%E5%BE%AE%E4%BF%A1)，然后去 5.4 内核下安装[老 deepin-wine](https://github.com/wszqkzqk/deepin-wine-ubuntu)，并安装[旧版 TIM](https://gitee.com/wszqkzqk/deepin-wine-containers-for-ubuntu/raw/master/deepin.com.qq.office_2.0.0deepin4_i386.deb)（在 5.15 内核下安装 TIM 会无法打开），切回到 5.15 内核后可正常使用 TIM（甚至可以输密码自动登录）
- [toplcons plus 插件](https://extensions.gnome.org/extension/1031/topicons/)
- wps: 参考[安装 wps](https://github.com/ResearcherYan/LearningNotes/blob/master/general/linux.md#%E5%AE%89%E8%A3%85-wps-%E6%9C%80%E6%96%B0%E7%89%88)，安装好之后会提示缺失字体，`sudo apt install symbol-fonts_1.2_all.deb` 安装缺失的字体（这个包在自己的 U 盘里有，[备份网盘链接](https://http://pan.baidu.com/s/1IzxdT_7iEjTq_2uDRofLMQ)，密码: 23d3）

**安装 ORB-SLAM2 相关的包**
- Pangolin: 参考 [视觉 SLAM 14 讲第三方库安装](https://github.com/ResearcherYan/LearningNotes/blob/master/slam/slambook2-3rdparty.md#pangolin-ch3)，路径为 `~/cmake_ws/Pangolin`
- Realsense SDK && realsense-ros: Realsense SDK 翻来覆去最后还是在 5.4 内核下从源代码安装了，路径为 `~/cmake_ws/librealsense`
- ORB_SLAM2_MAP, dashgo_driver, dashgo_tools, yocs_velocity_smoother（从别人电脑里直接拷到 `~/catkin_ws/src` 下然后编译），并补安装一些缺失库：
  ```bash
  python3 -m pip install pyserial
  sudo usermod -aG dialout yan # 为用户 yan 提供串口读写权限，重启生效
  sudo apt install ros-noetic-serial ros-noetic-ecl-threads ros-noetic-teleop-twist-keyboard
  ```
- 针对上面这些包，修改一些源文件
  - ORB_SLAM2_MAP: `ORB_SLAM2_MAP/Examples/ROS/ORB_SLAM2_MAP/slam.launch`，路径改成自己电脑下的
  - dashgo_driver
    - `dashgo_driver/config/my_dashgo_params.yaml`: 第一句串口号改为 `/dev/ttyUSB0`
    - `dashgo_driver/nodes/dashgo_driver.py`: 第一句 python 解释器改为 `#!/usr/bin/python3`，确保用的是 python3
  - dashgo_tools
    - `dashgo_tools/scripts/check_action.py` & `dashgo_tools/scripts/teleop_twist_keyboard.py`: 第一句 python 解释器改为 `#!/usr/bin/python3`，确保用的是 python3
- 编译安装上述包
  ```
  catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
  catkin_make install
  ```

**安装 LeGO-LOAM 相关的包**
- VLP-16 驱动: 参考 [VLP-16](https://github.com/ResearcherYan/LearningNotes/blob/master/slam/lego-loam/lego-loam.md#vlp-16)（新电脑按 super 键找不到 Networks Connections，只能从终端启动 `nm-connection-editor`）
- gtsam: 参考 [LeGO-LOAM Dependency](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM#dependency)，路径为 `~/cmake_ws/gtsam-4.0.0-alpha2`（这个库 make 有点慢，用所有线程一起 make: `sudo make -j$(nproc) install`）
- LeGO-LOAM: 参考 [LeGO-LOAM 安装](https://github.com/ResearcherYan/LearningNotes/blob/master/slam/lego-loam/lego-loam.md#lego-loam)，修改 LeGO-LOAM 源码之后再编译


## 尝试用电脑同时连接小车底盘和相机
- 同时启动 ORB_SLAM2_MAP 和底盘控制程序
  ```
  roslaunch ORB_SLAM2_MAP slam.launch
  roslaunch dashgo_driver driver.launch
  rosrun dashgo_tools teleop_twist_keyboard.py
  ```
  控制小车走一会之后控制失灵，无法继续控制
- 串口调试
  - `sudo apt install cutecom`
  - 同时连接小车底盘和相机，在 cutecom 里面给小车底盘收发数据：`m -3 3`, `m 3 -3`（左转右转）, `e`（返回编码器数据）
  - 有两种情况
    - 正常发送接收几条数据之后，再发送数据，没有数据反馈，点击 close 会卡死，如果此时 ctrl + c 退出 cutecom，`ls /dev | grep ttyUSB` 找不到串口，重新插拔也没用，同时电脑蓝牙也会没了，并且关机会卡住。
    - 给底盘发送一条数据后，底盘串口从 `/dev/ttyUSB0` 跳到 `/dev/ttyUSB1`，然后就需要手动打开 ttyUSB1 串口，再发送一条，又跳到 ttyUSB0，循环往复。
- 原因总结：自己的旧电脑和另一位同学的电脑（都是 intel 的 cpu）都没问题，而实验室的两台 AMD 的电脑都出现这个问题。在网上查了之后，发现 Intel Realsense 相机就是对 AMD 支持比较差，所以才会出现串口紊乱的情况。

## 尝试同时使用相机和激光雷达
> 经验证，在 AMD CPU 上，Intel Realsense 相机与 VLP-16 激光雷达不会产生冲突（主要因为激光雷达数据是通过网口传输，realsense 主要影响 USB 设备）

将“启动相机”、“运行 ORB_SLAM2_MAP”、“启动激光雷达”、“录制激光雷达点云”一起写到 `ORB_SLAM2_MAP` 的 `slam.launch` 文件里，免得开多个终端。
- 相机录制的图片路径: `/home/yan/v_slam/1/`（多次录制需要手动更改此路径）
- 激光雷达录制的点云路径: `/home/yan/lidar_slam/velodyne.bag`（多次录制需要手动更改 bag 的名称）

## 卸载 Ubuntu 22.04
事实证明，ZHAN 66 在多内核的加持下能够丝滑使用 Ubuntu 20.04（5.15 用于适配硬件驱动，5.4 用于适配软件兼容性，5.13 为安装时的初始内核），于是决定卸掉 Ubuntu 22.04，把多余的空间腾给主力系统。<br><br>
**操作步骤**
- 准备好 Ubuntu 启动盘
- Try Ubuntu
- 打开 Gparted
- 删除掉 Ubuntu 22 的三个分区：swap, efi, 主分区。由于 Ubuntu 20 的空间是从 win 11 划过去的，所以 Ubuntu 22 的空间正好是在 Ubuntu 20 主分区扇区的后面，就可以直接合并。
- 重启进 bios：把 U 盘启动放在 Ubuntu 后面
- 进入 Ubuntu 20，这次开机很慢，卡在加载页面，按 esc 看到后台进程显示
  ```
  A start job is running for /dev/disk/by-uuid/...
  ```
  查阅资料之后，发现是因为 Ubuntu 20 启动时会加载 Ubuntu 22 的 swap 分区（因为 Ubuntu 22 的 swap 分区就在 Ubuntu 20 的主分区后面，所以可能 Ubuntu 20 认为给自己分了两个 swap 分区）<br>
  先用 `sudo blkid` 查看当前系统 swap 分区对应的 uuid，然后再 `sudo gedit /etc/fstab`，把该文件里另一个 swap 分区的 uuid 删掉
- 更新 grub: `sudo update-grub2`
- 重启，发现 Ubuntu 22 的启动项消失，开机速度正常，Ubuntu 20 成功扩容
