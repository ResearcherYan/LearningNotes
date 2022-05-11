> 本文主要记录 LeGO-LOAM 的安装过程

> CONTENT
- [Raspberry Pi 4B](#raspberry-pi-4b)
- [PC](#pc)
  - [ROS Noetic](#ros-noetic)
  - [VLP-16](#vlp-16)
  - [LeGO-LOAM](#lego-loam)
  - [Debug LeGO-LOAM](#debug-lego-loam)

## Raspberry Pi 4B
本来想直接在树莓派 4B 上部署 LeGO-LOAM，尝试了将近一周后，未果。主要原因可能还是 PCL/Boost/FLANN 库的版本问题。以下记录安装过程中的关键错误。
- ROS Noetic 默认安装的是 PCL 1.10，这会直接导致 LeGO-LOAM 的 ImageProjection 线程崩溃。<br>
- 转到 PCL 1.9 版本后，看起来 LeGO-LOAM 可以正常运行（虽然偶尔会出现 MapOptimization 线程崩溃的情况），rviz 窗口也能看到激光雷达扫到的点云，但参考[博客](https://blog.csdn.net/sinat_38792591/article/details/117064027)尝试把 LeGO-LOAM 得到的全局地图点云导出时，发现没有任何点云信息。回头看终端的输出信息，发现会有一段红字：*[pcl::KdTreeFLANN::setInputCloud] Cannot create a KDTree with an empty input cloud!*，感觉原因是 PCL 库没有接收到来自雷达的点云。
- 在克服了重重困难，在树莓派上编译好 PCL 1.8 版本后，发现跟 PCL 1.9 一样，还是无法导出点云信息。

暂时放弃在树莓派上部署的想法，先考虑在自己电脑上跑通 LeGO-LOAM（因为这样编译起来快很多，不用受到 VNC 连接经常卡顿的折磨，网速更快并且可以走代理）。以后如果再复现什么开源算法，先在自己电脑上跑通、debug 完了之后再考虑部署。

## PC
做好心理准备，尝试在自己电脑上跑通 LeGO-LOAM（因为自己电脑上的各种库环境要比树莓派上复杂）。
### ROS Noetic
参考 [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)，步骤 1.2 选择用中科大的源。
- `sudo apt install ros-noetic-desktop-full`，报依赖错误，一层层往下挖，直到挖到了 `sudo apt install libopencv-dev`，报错：
  ```bash
  下列软件包有未满足的依赖关系：
  libopencv-dev : 依赖: libopencv-calib3d-dev (= 4.2.0+dfsg-5) 但是它将不会被安装
                  依赖: libopencv-contrib-dev (= 4.2.0+dfsg-5) 但是它将不会被安装
                  依赖: libopencv-features2d-dev (= 4.2.0+dfsg-5) 但是它将不会被安装
                  依赖: libopencv-highgui-dev (= 4.2.0+dfsg-5) 但是它将不会被安装
                  依赖: libopencv-objdetect-dev (= 4.2.0+dfsg-5) 但是它将不会被安装
                  依赖: libopencv-stitching-dev (= 4.2.0+dfsg-5) 但是它将不会被安装
                  依赖: libopencv-superres-dev (= 4.2.0+dfsg-5) 但是它将不会被安装
                  依赖: libopencv-videoio-dev (= 4.2.0+dfsg-5) 但是它将不会被安装
                  依赖: libopencv-videostab-dev (= 4.2.0+dfsg-5) 但是它将不会被安装
  ```
  发现 ROS Noetic 应该是依赖于特定的一个 opencv 版本（4.2.0），但之前在装《视觉 SLAM 14 讲》的库的时候，已经装了 opencv 4.5，感觉可能是 opencv 版本冲突了，于是按照 [slambook2-3rdparty.md#opencv](../slambook2-3rdparty.md#opencv-ch5) 里的要求卸载了 opencv（但并未删除 */home/ubuntu/software/opencv-4.5.5*）。<br>
  卸载完之后，发现 `sudo apt install ros-noetic-desktop-full` 还是会报错。
- 尝试用 aptitude 寻找错误的依赖：`sudo aptitude install ros-noetic-desktop-full`。aptitude 提示需要修改几个看起来无关紧要的库的版本，选择同意之后，仍然无法安装 ros-noetic-desktop-full。
- 选择顺着 `sudo apt install ros-noetic-desktop-full` 报的依赖错误一步步往下找，发现降级一个软件包即可：`sudo apt install libgphoto2-6=2.5.24-1`。降级完之后再安装 ros：`sudo apt install ros-noetic-desktop-full`。
- 继续完成 [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 里的其余步骤，完成 ros 的安装。
- 参考[安装和配置ROS环境](http://wiki.ros.org/cn/ROS/Tutorials/InstallingandConfiguringROSEnvironment)，步骤 3 直接使用 *catkin_make* 会报错，要按照注脚里的 *catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3* 才能够成功初始化 catkin 工作空间。最后还要 `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc`。

### VLP-16
参考 [Getting Started with the Velodyne VLP16](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)，配置好 VLP-16 雷达。
- 步骤 1.1 里把 ip 设置为 192.168.1.102（为了与速腾的雷达也兼容）
- 步骤 1.2 不用执行

### LeGO-LOAM
- 参考 [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)，编译 gtsam，并且 git clone LeGO-LOAM 源代码。（先不要急着编译）
- 参考 [LeGO-LOAM：Ubuntu20.04下的编译与运行](https://blog.csdn.net/weixin_44156680/article/details/118070387?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1.pc_relevant_default&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1.pc_relevant_default&utm_relevant_index=1) 中的第六条，修改源代码并安装缺失的库。博客里提到的 rviz 问题，需要去掉 "/" 的 frame_id 是 "/camera" 和 "/camera_init"。
- `cd ~/catkin_ws && catkin_make -j1`
- 成功编译一次后，之后再编译 LeGO-LOAM 的时候可以直接 `catkin_make -DCATKIN_WHITELIST_PACKAGES="lego_loam;cloud_msgs"`

### Debug LeGO-LOAM
- 修改 LeGO-LOAM 的 launch 文件：`rosed lego_loam run.launch`，把 */use_sim_time* 由 true 改为 false，表示用的是实际采集的雷达数据，而非来自于 rosbag 的回放。
- 启动雷达的 launch 文件：`roslaunch velodyne_pointcloud VLP16_points.launch`
- 启动 LeGO-LOAM 的 launch 文件：`roslaunch lego_loam run.launch`。在 rviz 中看到雷达采到的点云后，使用 ctrl + c 停止 LeGO-LOAM 运行。发现报错：
  ```bash
  terminate called after throwing an instance of 'pcl::IOException'
  what():  : [pcl::PCDWriter::writeASCII] Input point cloud has no data!
  ```
  想起来在树莓派上也会报这个错，这个错误会导致无法导出全局地图的点云。
- 参考 [LeGO-LOAM初探：原理，安装和测试](https://blog.csdn.net/learning_tortosie/article/details/86527542)，发现其实 LeGO-SLAM 自带了保存点云地图的功能，只需要在 rviz 中勾选 Map Cloud，然后再 ctrl + c，这样就会在 /tmp 目录下应该会生成 4 个 pcd 文件，其中 finalCloud.pcd 应该是最终的点云地图。
- 安装 pcl-tools：`sudo apt install pcl-tools`<br>
  查看点云地图：`pcl_viewer /tmp/finalCloud.pcd`