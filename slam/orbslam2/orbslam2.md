## Installation
> 参考链接
> 1. [ORB-SLAM2 Docker 容器 (gitee)](https://gitee.com/wycan/orbslam2_runin_docker)
> 2. [sudo apt update --> Certificate verification failed](https://blog.csdn.net/qlexcel/article/details/120642914)
> 3. [vscode --> XHR failed](https://stackoverflow.com/questions/70177216/visual-studio-code-error-while-fetching-extensions-xhr-failed)
> 4. [vscode --> Client network socket disconnected](https://github.com/microsoft/vscode-remote-release/issues/986)
> 
> 备用链接
> 1. [带 GPU 驱动 (libnvidia-container) 的安装方法](https://blog.csdn.net/WEINILUO/article/details/118659410)
> 2. [ORB-SLAM2 Docker 容器 (docker hub)](https://hub.docker.com/r/celinachild/orbslam2)
> 3. [ORB-SLAM3 Docker 容器 (github)](https://github.com/jahaniam/orbslam3_docker)

### How to install
考虑到开发和部署环境的一致性需求，选择用 docker 的方式安装 orbslam2。总的安装过程参考了 [ORB-SLAM2 Docker 容器 (gitee)](https://gitee.com/wycan/orbslam2_runin_docker)，直接使用 vscode 的 remote container 插件实现容器的创建非常方便。<br>
但是这个仓库只配置了 orbslam2 的环境，没有配置 ROS，RealSense 相机软件包，以及激光雷达软件包等，并且 orbslam2 的依赖包的位置不是很合理，因此需要做一些调整。<br>

主要包括以下几个改动
- Dependencies：把 orbslam2 的依赖包从 .devcontainer 文件夹中移出，移到源代码文件夹外面。<br>
  好处：这样可以减少容器的大小。在构建容器的过程中就已经把依赖复制到了容器内（因为需要在容器内 build），而 remote container 构建完容器后，还会把主机 vscode 打开的文件夹（即源代码文件夹）复制到容器内，如果把依赖包放在源代码文件夹下，相当于第二次把这些依赖复制到容器内，造成不必要的空间浪费。
- Dockerfile
  - Base image：不用 ubuntu:bionic，直接用装好 ROS Melodic 的 ubuntu bionic 镜像 ros:melodic-ros-base-bionic。但是要注意这样安装的 ROS 是不能直接在命令行调用 ROS command 的，需要在 `~/.bashrc` 中添加 `source /opt/ros/melodic/setup.bash`，才能保证每次开启终端的时候会同时得到 ROS command 的 access.（参考 [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)）
  - Later installation
    - RealSense SDK & RealSense ROS Package
    - Lidar dependencies & Lidar ROS Package
    - ...


**注意 [devcontainer.json](devcontainer.json) 和 [Dockerfile](Dockerfile) 文件相比于 gitee 上的有所改动。**

### Trouble shooting
- Problem #1: Certificate verification failed: The certificate is NOT trusted.
  - 先尝试了网上说的重装 ca-certificates，无果（通过 apt install 或手动下载 deb 包安装都没用）。
  - 后面参考 [sudo apt update --> Certificate verification failed](https://blog.csdn.net/qlexcel/article/details/120642914) 直接把 *Dockerfile* 里面清华源的 https 换成 http，成功了。
  - 最后，发现其实是因为 https 协议在传输过程中用了 SSL/TLS 数据加密，导致 http 代理无法正常接收数据，具体原因参考 [http-可用-https-不可用](../../general/proxy.md#http-可用-https-不可用)，因此要通过设置 `"http.proxyStrictSSL": false`，才能正常从 https 网站上下载。
- Problem #2: Client network socket disconnected before secure TLS connection was established 或 XHR failed
  - 首先查的log文件的最底部的报错信息，就是 *XHR failed*。看到 [stackoverflow](https://stackoverflow.com/questions/70177216/visual-studio-code-error-while-fetching-extensions-xhr-failed) 上面有个回答说是因为 vscode 和 PC 设置的 proxy 冲突了。
  - 随后就把log文件往上看了看，发现确实是因为网络问题，还有个报错信息就是 *Client network socket disconnected before secure TLS connection was established*，于是就知道应该是小飞机的问题，应该是容器没有接上主机的代理。通过一个 [github issue](https://github.com/microsoft/vscode-remote-release/issues/986) 找到了给容器设置代理的方法，即在 *devcontainer.json* 的 settings 里设置 `http.proxy`（`https.proxy` 也可以加上，但加上的话一定还要加一句 `"http.proxyStrictSSL": false`）。
- Problem #3: Pangolin X11: Failed to open X display
  - 参考 [askubuntu](https://askubuntu.com/questions/432255/what-is-the-display-environment-variable) ，在既有集显又有独显的电脑上，`DISPLAY=:0` 应该表示显示设备为集显，而 `DISPLAY=:1` 则表示显示设备为独显。在终端查看 `echo $DISPLAY` 发现是 `:1`，说明现在用的是独显，因此在 devcontainer.json 里面也应该把 pangolin 的窗口输出到独显上，即设置 `DISPLAY=:1`。
- Warnings
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


  