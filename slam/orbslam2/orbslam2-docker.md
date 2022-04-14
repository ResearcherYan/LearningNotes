> 本文主要介绍：如何构建 orbslam2 的 docker 环境（带有 ROS 和 realsense ROS package），以及如何使用 realsense D455 相机作为 orbslam2 的 RGBD 数据输入。

> CONTENT
- [Image Building](#image-building)
  - [Procedures](#procedures)
    - [1. 准备工作目录](#1-准备工作目录)
    - [2. git clone & 完善工作目录](#2-git-clone--完善工作目录)
    - [3. 修改 ORB_SLAM2 源代码](#3-修改-orb_slam2-源代码)
    - [4. 使用 VSCode Remote Container 构建镜像](#4-使用-vscode-remote-container-构建镜像)
    - [5. 运行 ORB_SLAM2 的 RGBD demo](#5-运行-orb_slam2-的-rgbd-demo)
  - [devcontainer.json 和 Dockerfile 的改动（相较于 gitee 仓库）](#devcontainerjson-和-dockerfile-的改动相较于-gitee-仓库)
  - [如何获取相机信息](#如何获取相机信息)
- [Trouble shooting](#trouble-shooting)
  - [Problem #1: Certificate verification failed: The certificate is NOT trusted.](#problem-1-certificate-verification-failed-the-certificate-is-not-trusted)
  - [Problem #2: Client network socket disconnected before secure TLS connection was established 或 XHR failed](#problem-2-client-network-socket-disconnected-before-secure-tls-connection-was-established-或-xhr-failed)
  - [Problem #3: Pangolin X11: Failed to open X display](#problem-3-pangolin-x11-failed-to-open-x-display)
  - [Problem #4: No protocol specified.](#problem-4-no-protocol-specified)
  - [Problem #5: error checking context: 'no permission to read from '.../ORB_SLAM2/core](#problem-5-error-checking-context-no-permission-to-read-from-orb_slam2core)
  - [Problem #6: apt-utils : Depends: apt (= 1.6.1) but 1.6.14 is to be installed](#problem-6-apt-utils--depends-apt--161-but-1614-is-to-be-installed)
  - [Problem #7: error: ‘usleep’ was not declared in this scope](#problem-7-error-usleep-was-not-declared-in-this-scope)
  - [Problem #8: fatal error: ../../config.h: No such file or directory](#problem-8-fatal-error-configh-no-such-file-or-directory)
  - [Problem #9: undefined reference to symbol '_ZN5boost6system15system_categoryEv'](#problem-9-undefined-reference-to-symbol-_zn5boost6system15system_categoryev)
  - [Problem #10: Re-run cmake with a different source directory.](#problem-10-re-run-cmake-with-a-different-source-directory)
  - [Problem #11: [rospack] Error: package 'ORB_SLAM2' not found](#problem-11-rospack-error-package-orb_slam2-not-found)
  - [Problem #12: Failed to connect to 127.0.0.1 port 12333: Connection refused](#problem-12-failed-to-connect-to-127001-port-12333-connection-refused)
  - [Problem #13 standard_init_linux.go:228: exec user process caused: exec format error](#problem-13-standard_init_linuxgo228-exec-user-process-caused-exec-format-error)
  - [Warnings](#warnings)
- [核心参考链接](#核心参考链接)


## Image Building
考虑到开发和部署环境的一致性需求，选择用 docker 的方式安装 orbslam2。安装方法参考了 [ORB-SLAM2 Docker 容器 (gitee)](https://gitee.com/wycan/orbslam2_runin_docker)，直接使用 vscode 的 remote container 插件实现容器的创建非常方便。<br>
但是这个仓库只配置了 orbslam2 的环境，没有配置 ROS，RealSense 相机软件包，以及激光雷达软件包等，并且 orbslam2 的依赖包的位置不是很合理，因此以下的步骤也只是部分参考这个仓库。

### Procedures
请在同一个终端执行 *Procedures* 这一节里的所有命令。
#### 1. 准备工作目录
```bash
# 创建工作目录，如 /home/yan/Learning/slam/orbslam2
mkdir -p /home/yan/Learning/slam/orbslam2
cd /home/yan/Learning/slam/orbslam2
# 创建一个用于 vscode 打开的目录（后面会用 vscode 打开这个目录，然后连接到 remote container）
mkdir vscode-folder
```
接下来需要手动操作一步：把本仓库的 [.devcontainer 文件夹](./.devcontainer) 复制到 vscode-folder 里。
> 为什么要单独设立一个 vscode-folder，并且里面只装 .devcontainer 文件夹？
> - 不把依赖包放在 vscode-folder 下的原因：在构建容器的过程中就已经通过 COPY 操作把依赖包复制到了容器内（因为需要在构建容器的过程中 build 这些依赖包），而 remote container 在构建完容器后，还会把主机 vscode 打开的文件夹（即 vscode-folder）复制到容器内，如果像 [gitee 仓库](https://gitee.com/wycan/orbslam2_runin_docker) 那样把依赖包放在 vscode-folder 下，相当于第二次把这些依赖复制到容器内，造成不必要的空间浪费（接近0.5G）。
> - 不把源代码文件放在 vscode-folder 下的原因：同理，由于需要在构建容器的过程中 build ORB_SLAM2 源代码，就需要通过 COPY 操作把源代码复制到容器内，如果像 [gitee 仓库](https://gitee.com/wycan/orbslam2_runin_docker) 那样把源代码放在 vscode-folder 下，相当于第二次把源代码复制到容器内，源代码位置太多不好管理。

#### 2. git clone & 完善工作目录
```bash
# orbslam2 docker 环境（主要用到里面的 3 个依赖包），这个仓库 copy 到 orbslam2 文件夹外面
cd .. && git clone https://gitee.com/wycan/orbslam2_runin_docker.git
# 把这个仓库里的 3 个依赖包 copy 到相应文件夹内
mkdir orbslam2/orbslam2-dependencies && cp -r orbslam2_runin_docker-master/.devcontainer/extendmodel/* orbslam2/orbslam2-dependencies
# ORB_SLAM2 源码
cd orbslam2 && git clone https://github.com/raulmur/ORB_SLAM2.git
# RealSense ROS Package
git clone https://github.com/IntelRealSense/realsense-ros.git
# RealSense ROS Package 的依赖包
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
```
#### 3. 修改 ORB_SLAM2 源代码
- 删除 *.git* 文件夹和 *.gitigore* 文件（以免后面用 VSCode 编辑的时候 git 扩展总会显示一堆更改）
```bash
cd ORB_SLAM2 && rm .git .gitignore
```
- 修改源码，以避免报错
  - 在 */include/System.h* 中加上 `#include <unistd.h>`。否则会导致 [Problem #7](#problem-7-error-usleep-was-not-declared-in-this-scope)。
  - 修改 */Examples/ROS/ORB_SLAM2/CMakeLists.txt*，修改方法参考[Problem #9](#problem-9-undefined-reference-to-symbol-_zn5boost6system15system_categoryev)。
  - 用本仓库的 [build.sh](src/build.sh) 和 [build_ros.sh](src/build_ros.sh)，以替代源码里的这两个文件。否则后面在 build ORB_SLAM2 源码的过程中可能会遇到 [Problem #8](#problem-8-fatal-error-configh-no-such-file-or-directory) 或 [Problem #10](#problem-10-re-run-cmake-with-a-different-source-directory)。
- 将相机信息写入源码中（获取相机信息的方法参考 [如何获取相机信息](#如何获取相机信息) ）
  - 把本仓库下的 [AsusD455.yaml](src/AsusD455.yaml) 复制到 */Examples/ROS/ORB_SLAM2* 下。
  - 把本仓库下的 [ros_rgbd.cc](src/ros_rgbd.cc) 复制到 */Examples/ROS/ORB_SLAM2/src* 下，替换掉同名文件。

#### 4. 使用 VSCode Remote Container 构建镜像
- 在主机终端输入 `xhost +`，许可所有用户都可访问 xserver（此设置在主机重启后会失效）。
- 用 VSCode 打开文件夹 *vscode-folder*，然后在 command palatte 中输入 `Reopen in Container` 即会开始构建镜像。

#### 5. 运行 ORB_SLAM2 的 RGBD demo
打开三个终端
- `roscore`
- `roslaunch realsense2_camera rs_rgbd.launch`
- `rosrun ORB_SLAM2 RGBD /root/catkin_ws/ORB_SLAM2/Vocabulary/ORBvoc.txt /root/catkin_ws/ORB_SLAM2/Examples/ROS/ORB_SLAM2/AsusD455.yaml`

### devcontainer.json 和 Dockerfile 的改动（相较于 gitee 仓库）
- [devcontainer.json](./.devcontainer/devcontainer.json)
  - Build context: 把 build context 设置为了安装步骤里的 *orbslam2* 文件夹，使得 *Dockerfile* 里的 COPY 指令能够在 build context 下找到需要的文件。
  - Mount Point: 把 */dev* 文件夹全部挂载上了，因为不知道 realsense camera 用的是 */dev* 里的哪个设备号。
  - User: 去掉了 runArgs 里的 `-u` 参数，即没有设置 non-root user 了，直接用的 root user。
  - Network: 在 runArgs 里加上了 `"--network=host"`，让容器和主机共享了 network namespace，在容器直接用 127.0.0.1:12333 就可以访问主机的代理了。
  - Proxy: 如果要在容器构建好后之后马上设置好 proxy 的环境变量，可以在设置 containerEnv。（其实建议除了 git 之外，其他能用镜像就不要用代理，直接用镜像下载还是要快一些）
- [Dockerfile](./.devcontainer/Dockerfile)
  - Base image：不用 ubuntu:bionic，直接用装好 ROS Melodic 的 ubuntu bionic 镜像 ros:melodic-ros-base-bionic。但是要注意这样安装的 ROS 是不能直接在命令行调用 ROS command 的，需要在 `~/.bashrc` 中添加 `source /opt/ros/melodic/setup.bash`，才能保证每次开启终端的时候会同时得到 ROS command 的 access（参考 [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)），这个已经写到了 *Dockerfile* 里面，所以不用担心。
  - User: 删除了几个关于 user 的 `ARG` 指令，直接用 root user。
  - Downloading source: 下载源换成了中科大的源，比清华源快很多。
  - DEBIAN_FRONTEND:
    - 在尝试构建镜像的过程中，不通过 `ENV` 的方式来设置环境变量 DEBIAN_FRONTEND，改为即用即销，即哪一层需要用到 DEBIAN_FRONTEND 就在哪一层设置，并且用完后立即 unset。
    - 完整跑通一遍镜像构建后，可以再把 DEBIAN_FRONTEND 改为全局的 `ENV`，到最后面再通过 `ENV` 改回来。
  - More packages installation
    - RealSense SDK & RealSense ROS Package
    - Future: Lidar dependencies & Lidar ROS Package
    - ...

### 如何获取相机信息
1. 获取相机内参矩阵。在 *catkin* 目录下打开三个终端：
 - `roscore`
 - `roslaunch realsense2_camera rs_rgbd.launch`
 - `rostopic echo /camera/color/camera_info`<br>
 看到终端里的内参矩阵 K，对比 `K = [fx 0 cx 0 fy cy 0 0 1 ]` 得到 fx, fy, cx, cy 的值。
2. 查 D455 的 datasheet，可以知道 D455 的 baseline 长度为 95mm，根据 `bf = baseline (in meters) * fx` 算出 bf 的值。
3. 在 */Examples/ROS/ORB_SLAM2* 下新建一个文件 *AsusD455.yaml*，把 *Asus.yaml* 的内容复制进来，把前面两步得到的 5 个参数改掉。
4. 更改源代码中 camera data 和 depth data 的位置：在 *Examples/ROS/ORB_SLAM2/src/ros_rgbd.cc* 将 rgb_sub 的路径和 depth_sub 的路径分别换成 `/camera/color/image_raw` 和 `/camera/depth/image_rect_raw`。

## Trouble shooting
- Problem #1, 2, 3, 12 通过修改 *devcontainer.json* 解决。
- Problem #7, 9 通过修改 ORB_SLAM2 源码解决。
- Problem #6, 8, 10, 11 通过修改 *Dockerfile* 解决。
### Problem #1: Certificate verification failed: The certificate is NOT trusted.
- 报错描述：镜像构建过程中执行到 `apt-get update` 时报错。
- 报错原因：*Dockerfile* 里面的下载源用的是 https，但又没有在 *.devcontainer.json* 中对代理进行设置。
- 解决办法：
  - 先尝试了网上说的重装 ca-certificates，无果（通过 apt install 或手动下载 deb 包安装都没用）。
  - 后面参考 [sudo apt update --> Certificate verification failed](https://blog.csdn.net/qlexcel/article/details/120642914) 直接把 *Dockerfile* 里面清华源的 https 换成 http，成功了。
  - 最后，发现其实是因为 https 协议在传输过程中用了 SSL/TLS 数据加密，导致 http 代理无法正常接收数据，具体原因参考 [http-可用-https-不可用](../../general/proxy.md#http-可用-https-不可用)，因此要通过设置 `"http.proxyStrictSSL": false`，才能正常从 https 网站上下载。

### Problem #2: Client network socket disconnected before secure TLS connection was established 或 XHR failed
- 报错描述：镜像构建过程中执行到 apt-get update 时报错。
- 报错原因 & 解决办法：
  - 首先查的log文件的最底部的报错信息，就是 *XHR failed*。看到 [stackoverflow](https://stackoverflow.com/questions/70177216/visual-studio-code-error-while-fetching-extensions-xhr-failed) 上面有个回答说是因为 vscode 和 PC 设置的 proxy 冲突了。
  - 随后就把log文件往上看了看，发现确实是因为网络问题，还有个报错信息就是 *Client network socket disconnected before secure TLS connection was established*，于是就知道应该是小飞机的问题，应该是镜像构建过程中没有接上主机的代理。通过一个 [github issue](https://github.com/microsoft/vscode-remote-release/issues/986) 找到了镜像构建过程中设置代理的方法，即在 *devcontainer.json* 的 settings 里设置 `http.proxy` 和 `https.proxy`（建议都设置成 `http://127.0.0.1:port`，不要用 https）。

### Problem #3: Pangolin X11: Failed to open X display
- 报错描述：用 orbslam2 单目 demo 运行 TUM 数据集的时候报错。
- 报错原因 & 解决办法：
  - 参考 [askubuntu](https://askubuntu.com/questions/432255/what-is-the-display-environment-variable) ，在既有集显又有独显的电脑上，`DISPLAY=:0` 应该表示显示设备为集显，而 `DISPLAY=:1` 则表示显示设备为独显。在终端查看 `echo $DISPLAY` 发现是 `:1`，说明现在用的是独显，因此在 *devcontainer.json* 里面也应该把 pangolin 的窗口输出到独显上，即设置 `DISPLAY=:1`。
  - 出现这个问题也可能是因为重启主机之后没有 `xhost +`。

### Problem #4: No protocol specified.
- 报错描述：重启主机后，启动镜像后运行 `realsense-viewer` 报错。
- 报错原因 & 解决办法：重启主机后 `xhost +` 命令会失效，需要重新输入一次。

### Problem #5: error checking context: 'no permission to read from '.../ORB_SLAM2/core
- 报错描述：在 container 里成功运行 orbslam2 demo 之后，用 vscode 重新打开 remote container 的时候报错。
- 报错原因：因为在 container 里是用 root 用户的身份去 build 的，所以回到本地环境再次用 remote container 打开 *ORB_SLAM2* 文件夹的时候会报权限错误。
- 解决方法：此时只需要重置一下当前用户对 *ORB_SLAM2* 文件夹的权限 `sudo chown -R $USER: /home/yan/Learning/slam/orbslam2/ORB_SLAM2`。

### Problem #6: apt-utils : Depends: apt (= 1.6.1) but 1.6.14 is to be installed
- 报错描述：把 `SHELL ["/bin/bash", "-c"]` 移到 *Dockerfile* 最前面，镜像构建过程中执行到 `apt-get -y install --no-install-recommends apt-utils dialog 2>&1` 报错。
- 报错原因：在镜像构建的过程中默认用的 shell 是 /bin/sh，改成 /bin/bash 的话可能 apt 和 apt-utils 的版本就不一致了。
- 解决方法：把 `SHELL ["/bin/bash", "-c"]` 移到后面，因为要用 /bin/bash 主要是因为要用 source 命令。
- 
### Problem #7: error: ‘usleep’ was not declared in this scope
- 报错描述：运行 `./build.sh` 报错
- 报错原因：猜测可能是作者使用过的c++版本和自己编译时的不一样
- 解决方法：参考 [github issue](https://github.com/raulmur/ORB_SLAM2/issues/778)。在 */include/System.h* 中加上 `#include <unistd.h>`

### Problem #8: fatal error: ../../config.h: No such file or directory
- 报错描述：运行 `./build_ros.sh` 报错
- 报错原因 && 解决方法：在 `./build_ros.sh` 之前没有先 `./build.sh`

### Problem #9: undefined reference to symbol '_ZN5boost6system15system_categoryEv'
- 报错描述：运行 `./build_ros.sh` 报错
- 报错原因：未知
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

### Problem #10: Re-run cmake with a different source directory.
- 报错描述：先在 */workspaces/ORB_SLAM2* 下运行了 *build.sh*，然后把 *ORB_SLAM2* 复制到 *catkin* 下面之后又运行了一遍 *build.sh*
- 报错原因：cmake 检测到了两次运行 *build.sh* 的路径不同，没法用 CMakeCache 了
- 解决方法：删除 */catkin/ORB_SLAM2* 下的 *build* 文件夹和 */catkin/ORB_SLAM2/Examples/ROS/ORB_SLAM2* 下的 *build* 文件夹，然后再重新运行 *build.sh*。（这个操作已植入到本仓库下的 [build.sh](build.sh) 和 [build_ros.sh](build_ros.sh)，不用再每次都手动删除了）

### Problem #11: [rospack] Error: package 'ORB_SLAM2' not found
- 报错描述：运行 `rosrun ORB_SLAM2 RGBD /root/catkin_ws/ORB_SLAM2/Vocabulary/ORBvoc.txt /root/catkin_ws/ORB_SLAM2/Examples/ROS/ORB_SLAM2/AsusD455.yaml` 报错
- 报错原因：没有设置 ORB_SLAM2 ROS 的环境变量
- 解决方法：参考[博客](https://www.cnblogs.com/1228073191Blog/p/10635691.html)。先 `source /root/catkin_ws/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/setup.bash`，然后再执行上面的 rosrun 指令。（ROS 环境变量的设置都已经植入到 *Dockerfile* 里了，一启动 remote container 就配置好了）

### Problem #12: Failed to connect to 127.0.0.1 port 12333: Connection refused
- 报错描述：启动 orbslam2 docker 容器后运行 `git clone` 命令报错。
- 排错过程：
  - git
    - `git config --global --get http.proxy && git config --global --get https.proxy` 输出的是 `http://127.0.0.1:12333` 和 `https://127.0.0.1:12333`。
    - 点击 Reopen in Container 后，在镜像构建的终端信息中发现了这一句 *Start: Run in container: # Copy /home/yan/.gitconfig to /root/.gitconfig*，这就解释了为什么主机 proxy 环境变量全是 http（是通过 ~/.bashrc 设置的），而镜像的 git proxy 一个是 http，一个是 https，因为它是直接 copy 的主机的 .gitconfig 文件，而在主机上的 .gitconfig 文件里面，proxy 的设置就是一个 http，一个 https。把主机上的 .gitconfig 全改为 http 代理后，重新构建镜像之后，再用 `git config --global --get http.proxy && git config --global --get https.proxy` 查看发现果然都变成了 http 代理。
    - `git config --global --unset http.proxy && git config --global --unset https.proxy` 取消 git proxy 代理后，发现 `git clone` 可以正常使用。说明 git 不走代理可以，走代理不行。
  - apt
    - `apt-get update` 正常链接到镜像源网站
    - `env | grep -i proxy` 没有任何输出，说明容器没有设置任何代理的环境变量。可是明明在 *devcontainer.json* 文件的 *settings* 里设置了 proxy，为什么容器里的 proxy 环境变量还是没有被设置呢？
    - `export http_proxy=http://127.0.0.1:12333 && apt-get update` 也会报 proxy 的错，说明 apt 没有走代理是好的，走代理就不行了。
    - 参考 [github issue](https://github.com/microsoft/vscode-remote-release/issues/6464)，在 *devcontainer.json* 里设置 *containerEnv*。Rebuild 之后 `env | grep -i proxy`，发现容器的 proxy 环境变量设置成功。
    - 综上几条可以发现，*devcontainer.json* 文件的 *settings* 应该跟 vscode 自己的 `/home/yan/.config/Code/User/settings.json` 配置文件有点像，也就说明在  *devcontainer.json* 文件的 *settings* 里面设置的 proxy 只会在 vscode 进行镜像构建的过程中生效，并不会延伸到容器里，只有通过设置 *containerEnv* 才能设置容器内的 proxy 环境变量。

- 报错原因：从上面的 debug 过程可以发现，这个报错的根本原因还是容器无法通过 127.0.0.1:12333 连接到主机的代理。之前遇到的代理报错，都是在镜像构建过程中报的错（如 [Problem #1](#problem-1-certificate-verification-failed-the-certificate-is-not-trusted) 和 [Problem #2](#problem-2-client-network-socket-disconnected-before-secure-tls-connection-was-established-或-xhr-failed)），本质上是主机连接代理的问题，跟容器没关系，这次是容器无法连接主机的代理。
- 解决方法：让容器和主机共享 network namespace
    - 参考博客 [Connection refused? Docker networking and how it impacts your image](https://pythonspeed.com/articles/docker-connection-refused/)，发现原来 container 和 host 使用的是两个 network namespace，也就是说在 container 里面如果设置 proxy 为 127.0.0.1:12333，那么连接的是 container 自己的 127.0.0.1 接口，与主机的 127.0.0.1 接口完全无关。
    - 参考 [stackoverflow](https://stackoverflow.com/questions/24319662/from-inside-of-a-docker-container-how-do-i-connect-to-the-localhost-of-the-mach) 和 [docker docs](https://docs.docker.com/network/host/)，通过在 *devcontainer.json* 的 runArgs 里加上 `"--network=host"`，可以让容器和主机共享同一个 network namespace，这样一来在 container 里面设置 proxy 为 127.0.0.1:12333 就可以直接连接上主机的代理了。

### Problem #13 standard_init_linux.go:228: exec user process caused: exec format error
- 报错描述：将自己电脑上的容器导出到树莓派上的时候报错。
- 报错原因：树莓派是 ARM 架构，而自己的电脑是 x86_64 (也就是 AMD) 架构，不同硬件架构上构建的 docker 镜像一般是不互通的。
- 解决方法
  - 选择1：只把 docker 作为开发环境，即仅仅把它作为一个 "clean workspace" 来用。要部署到树莓派上的时候，软件库还是手动一个个去装。
  - 选择2：在树莓派上也用 docker，用相同的 *Dockerfile* 再重新在树莓派上构建一个 docker 镜像。


### Warnings
  - 如果在 *devcontainer.json* 文件中没有指明 device 的文件位置的话，就会报下面的这些 warning（虽然写的是 error，但不影响程序运行）<br>
  ```shell
  error: XDG_RUNTIME_DIR not set in the environment.
  libGL error: MESA-LOADER: failed to retrieve device information
  libGL error: Version 4 or later of flush extension not found
  libGL error: failed to load driver: i915
  libGL error: failed to open /dev/dri/card0: No such file or directory
  libGL error: failed to load driver: iris
  Framebuffer with requested attributes not available. Using available framebuffer. You may see visual artifacts.
  ```
  参考 [gitee](https://gitee.com/feisonzl/orbslam2_runin_docker/commit/93a6d266c4ddc2c067a3a6cd786f13e88cb10470) 上对原 devcontainer.json 的修改，可以通过指定 device 来解决大部分的 warning（除了第一句）。具体见 [devcontainer.json](devcontainer.json)。


## 核心参考链接
1. [ORB-SLAM2 官方 github](https://github.com/raulmur/ORB_SLAM2)
2. [ORB-SLAM2 Docker 容器 (gitee)](https://gitee.com/wycan/orbslam2_runin_docker)
3. [使用Realsense D435相机在ROS Kinetic中跑通ORB-SLAM2](https://blog.csdn.net/Carminljm/article/details/86353775)
4. [RealSense ROS Package Installation](https://github.com/IntelRealSense/realsense-ros)

> 备用链接
> 1. [带 GPU 驱动 (libnvidia-container) 的安装方法](https://blog.csdn.net/WEINILUO/article/details/118659410)
> 2. [ORB-SLAM2 Docker 容器 (docker hub)](https://hub.docker.com/r/celinachild/orbslam2)
> 3. [ORB-SLAM3 Docker 容器 (github)](https://github.com/jahaniam/orbslam3_docker)