> CONTENT
- [Installation](#installation)
  - [Step #1 安装 RealSense SDK](#step-1-安装-realsense-sdk)
  - [Step #2 安装 ROS](#step-2-安装-ros)
  - [Step #3 安装 RealSense 的 ROS 包](#step-3-安装-realsense-的-ros-包)
  - [转化为 Dockerfile](#转化为-dockerfile)

## Installation
要基于 ROS 调用 RealSense camera，需要分三步安装软件包。第一步安装 RealSense SDK，第二步安装 ROS，第三步安装 RealSense 的 ROS 包。<br>
总的流程参考博客：[使用Realsense D435相机在ROS Kinetic中跑通ORB-SLAM2](https://blog.csdn.net/Carminljm/article/details/86353775)。

### Step #1 安装 RealSense SDK
直接安装 pre-build packages 就够了，不用从源代码安装。参考 [distribution_linux.md](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md)

### Step #2 安装 ROS
参考 [ROS 官网](http://wiki.ros.org/cn/melodic/Installation/Ubuntu)

### Step #3 安装 RealSense 的 ROS 包
参考 [ROS Wrapper for Intel® RealSense™ Devices](https://github.com/IntelRealSense/realsense-ros#step-2-install-intel-realsense-ros-from-sources)，直接看链接定向到的 Step 2 就好，前面的 Step 1 已经做了。

### 转化为 Dockerfile
因为 orbslam2 整个环境是在 docker 下面的，所以调用 RealSense camera 的软件包也需要部署在 docker 下。上面三步对应的 *Dockerfile* 命令可以在 [Dockerfile](Dockerfile) 里面看到。