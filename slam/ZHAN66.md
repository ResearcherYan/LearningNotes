> 本文用于记录 ZHAN 66 SLAM 开发环境的安装过程

首先，2022年的这款 ZHAN 66，用的是高通的一个比较新的无线网卡，Ubuntu 20.04 最新的固件版本也不支持它，因此被迫换到 Ubuntu 22.04，硬件适配上一切都好，但唯一不好的一点就是 Ubuntu 22 只能支持 ROS2，而目前 SLAM 开源社区用的基本上都是 ROS1，这就得自己把源代码 ROS1 的东西迁移到 ROS2。

## Velodyne 雷达
首先，截止今天（2022.7.4），ROS2 社区还没有把 humble 版本的 velodyne 驱动以包的形式发布出来，只有 github 上的[驱动源码](https://github.com/ros-drivers/velodyne/tree/ros2)，于是只能 git clone 到本地然后手动编译。
```
cd ~/dev_ws/src && git clone -b ros2 https://github.com/ros-drivers/velodyne.git
cd .. && rosdep install --from-paths src --ignore-src --rosdistro humble -y -r
colcon build --packages-select velodyne velodyne_laserscan velodyne_msgs velodyne_driver velodyne_pointcloud
```
注意如果在用 rosdep 安装完依赖之后，直接用 `colcon build --packages-select velodyne`，会报依赖错误，因为 `velodyne` 文件夹下面还有个 `velodyne`，如果用前面的那个命令，系统默认识别要编译的是那个子文件夹，而那个子文件夹又依赖于其他四个包 `velodyne_laserscan velodyne_msgs velodyne_driver velodyne_pointcloud`，因此这里应该同时把这个五个子文件夹一起编译。