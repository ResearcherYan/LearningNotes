> 本文主要介绍如何使用 realsense D455 相机作为 orbslam2 的 RGBD 数据输入。

> CONTENT
- [Procedures](#procedures)
  - [Step #1 - Build ORB-SLAM2](#step-1---build-orb-slam2)
  - [Step #2 - 获取相机信息](#step-2---获取相机信息)
  - [Step #3 - Build ORB-SLAM2 with ROS](#step-3---build-orb-slam2-with-ros)
  - [Step #4 - Run ORB_SLAM2 in RGBD mode with Realsense D455](#step-4---run-orb_slam2-in-rgbd-mode-with-realsense-d455)
- [Trouble Shooting](#trouble-shooting)
  - [Problem #1: error: ‘usleep’ was not declared in this scope](#problem-1-error-usleep-was-not-declared-in-this-scope)
  - [Problem #2: fatal error: ../../config.h: No such file or directory](#problem-2-fatal-error-configh-no-such-file-or-directory)
  - [Problem #3: undefined reference to symbol '_ZN5boost6system15system_categoryEv'](#problem-3-undefined-reference-to-symbol-_zn5boost6system15system_categoryev)
  - [Problem #4: Re-run cmake with a different source directory.](#problem-4-re-run-cmake-with-a-different-source-directory)
  - [Problem #5: [rospack] Error: package 'ORB_SLAM2' not found](#problem-5-rospack-error-package-orb_slam2-not-found)
- [备用链接](#备用链接)

## Procedures
总的流程参考博客：[使用Realsense D435相机在ROS Kinetic中跑通ORB-SLAM2](https://blog.csdn.net/Carminljm/article/details/86353775)。<br>
部署好 orbslam2 的 docker 环境后，要使用 realsense D455 相机跑 orbslam2 需要 3 步。

### Step #1 - Build ORB-SLAM2
1. 把 *ORB_SLAM2* 文件夹复制到 *catkin_ws* 文件夹中：`cp -r /workspaces/ORB_SLAM2 /root/catkin_ws/`
2. 修改源代码：在 */include/System.h* 中加上 `#include <unistd.h>`（否则会导致 [Problem #1](#problem-1-error-usleep-was-not-declared-in-this-scope)）
3. 运行 *build.sh*: `./build.sh`

### Step #2 - 获取相机信息
1. 获取相机内参矩阵。在 *catkin* 目录下打开三个终端：
   - `roscore`
   - `roslaunch realsense2_camera rs_rgbd.launch`
   - `rostopic echo /camera/color/camera_info`<br>
   看到终端里的内参矩阵 K，对比 `K = [fx 0 cx 0 fy cy 0 0 1 ]` 得到 fx, fy, cx, cy 的值。
2. 查 D455 的 datasheet，可以知道 D455 的 baseline 长度为 95mm，根据 `bf = baseline (in meters) * fx` 算出 bf 的值。
3. 在 */Examples/ROS/ORB_SLAM2* 下新建一个文件 *AsusD455.yaml*，把 *Asus.yaml* 的内容复制进来，把前面两步得到的 5 个参数改掉。
4. 更改源代码中 camera data 和 depth data 的位置：在 *Examples/ROS/ORB_SLAM2/src/ros_rgbd.cc* 将 rgb_sub 的路径和 depth_sub 的路径分别换成 `/camera/color/image_raw` 和 `/camera/depth/image_rect_raw`。

### Step #3 - Build ORB-SLAM2 with ROS
1. 设置环境变量：`export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/catkin_ws/ORB_SLAM2/Examples/ROS`（也可以写到 *.bashrc* 中）
2. 修改 CMakeLists.txt：具体修改方法见 [Problem #2](#problem-2-fatal-error-configh-no-such-file-or-directory)
3. 运行 *build_ros.sh*: `./build_ros.sh`

### Step #4 - Run ORB_SLAM2 in RGBD mode with Realsense D455
打开 3 个终端
- `roscore`
- `roslaunch realsense2_camera rs_rgbd.launch`
- `rosrun ORB_SLAM2 RGBD /root/catkin_ws/ORB_SLAM2/Vocabulary/ORBvoc.txt /root/catkin_ws/ORB_SLAM2/Examples/ROS/ORB_SLAM2/AsusD455.yaml`


## Trouble Shooting
### Problem #1: error: ‘usleep’ was not declared in this scope
- 报错描述：运行 `./build.sh` 报错
- 原因：猜测可能是作者使用过的c++版本和自己编译时的不一样
- 解决方法：参考 [github issue](https://github.com/raulmur/ORB_SLAM2/issues/778)。在 */include/System.h* 中加上 `#include <unistd.h>`

### Problem #2: fatal error: ../../config.h: No such file or directory
- 报错描述：运行 `./build_ros.sh` 报错
- 原因：在 `./build_ros.sh` 之前没有先 `./build.sh`

### Problem #3: undefined reference to symbol '_ZN5boost6system15system_categoryEv'
- 报错描述：运行 `./build_ros.sh` 报错
- 原因：未知
- 解决方法：参考 [github issue](https://github.com/raulmur/ORB_SLAM2/issues/494#issuecomment-354346674)，需要修改 */Examples/ROS/ORB_SLAM2/CMakeLists.txt*，主要是加了一行 `-lboost_system`。
  ```cmake
  set(LIBS 
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
  ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
  -lboost_system
  )
  ```

### Problem #4: Re-run cmake with a different source directory.
- 报错描述：先在 */workspaces/ORB_SLAM2* 下运行了 *build.sh*，然后把 *ORB_SLAM2* 复制到 *catkin* 下面之后又运行了一遍 *build.sh*
- 原因：cmake 检测到了两次运行 *build.sh* 的路径不同，没法用CMakeCache了
- 解决方法：删除 */catkin/ORB_SLAM2* 下的 *build* 文件夹和 */catkin/ORB_SLAM2/Examples/ROS/ORB_SLAM2* 下的 *build* 文件夹，然后再重新运行 *build.sh*

### Problem #5: [rospack] Error: package 'ORB_SLAM2' not found
- 报错描述：运行 `rosrun ORB_SLAM2 RGBD /root/catkin_ws/ORB_SLAM2/Vocabulary/ORBvoc.txt /root/catkin_ws/ORB_SLAM2/Examples/ROS/ORB_SLAM2/AsusD455.yaml` 报错
- 原因：没有设置 ORB_SLAM2 ROS 的环境变量
- 解决方法：参考[博客](https://www.cnblogs.com/1228073191Blog/p/10635691.html)。先 `source /root/catkin_ws/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/setup.bash`，然后再执行上面的 rosrun 指令

## 备用链接
- [RealSense ROS Package](https://github.com/IntelRealSense/realsense-ros)