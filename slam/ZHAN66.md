> 本文用于记录 ZHAN 66 SLAM 开发环境的安装过程

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