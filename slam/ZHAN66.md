> 本文用于记录 ZHAN 66 SLAM 开发环境的安装过程

- [VLP-16](#vlp-16)
- [尝试从源代码安装 ROS Noetic](#尝试从源代码安装-ros-noetic)
  - [下载源代码 + 安装依赖](#下载源代码--安装依赖)
  - [编译](#编译)

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

## 尝试从源代码安装 ROS Noetic
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
所以最终，选择放弃安装 ROS1，直接上 ROS2（起码是 offically supported）。