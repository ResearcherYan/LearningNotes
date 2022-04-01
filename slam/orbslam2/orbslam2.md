> CONTENT
- [Installation](#installation)
  - [Install method: Docker & VSCode Remote Container](#install-method-docker--vscode-remote-container)
  - [Install Procedures](#install-procedures)
  - [devcontainer.json 和 Dockerfile 的改动（相较于 gitee 仓库）](#devcontainerjson-和-dockerfile-的改动相较于-gitee-仓库)
  - [Trouble shooting](#trouble-shooting)
    - [Problem #1: Certificate verification failed: The certificate is NOT trusted.](#problem-1-certificate-verification-failed-the-certificate-is-not-trusted)
    - [Problem #2: Client network socket disconnected before secure TLS connection was established 或 XHR failed](#problem-2-client-network-socket-disconnected-before-secure-tls-connection-was-established-或-xhr-failed)
    - [Problem #3: Pangolin X11: Failed to open X display](#problem-3-pangolin-x11-failed-to-open-x-display)
    - [Problem #4: No protocol specified.](#problem-4-no-protocol-specified)
    - [Warnings](#warnings)
- [参考链接](#参考链接)
## Installation
### Install method: Docker & VSCode Remote Container
考虑到开发和部署环境的一致性需求，选择用 docker 的方式安装 orbslam2。具体安装方法参考了 [ORB-SLAM2 Docker 容器 (gitee)](https://gitee.com/wycan/orbslam2_runin_docker)，直接使用 vscode 的 remote container 插件实现容器的创建非常方便。<br>
但是这个仓库只配置了 orbslam2 的环境，没有配置 ROS，RealSense 相机软件包，以及激光雷达软件包等，并且 orbslam2 的依赖包的位置不是很合理，因此需要做一些调整。

### Install Procedures
1. 把两个远程仓库复制到本地：[gitee](https://gitee.com/wycan/orbslam2_runin_docker) 上的 orbslam2 docker 环境和 [github](https://github.com/raulmur/ORB_SLAM2) 上的 orbslam2 源码。
   ```bash
   git clone https://gitee.com/wycan/orbslam2_runin_docker.git
   git clone https://github.com/raulmur/ORB_SLAM2.git
   ```
2. 在 orbslam2 源代码文件夹（名称为 *ORB_SLAM2*）下：
   - 删除 *.git* 文件夹和 *.gitigore* 文件（免得后面用 VSCode 编辑的时候 git 扩展总会显示一堆更改）
   - 把本仓库的 [.devcontainer 文件夹](./.devcontainer) 复制到 *ORB_SLAM2* 里面。
3. 重新组织文件结构，以使得 orbslam2 源码和其依赖包是分开的，这样构建的 docker 镜像会小一些。
   - 创建一个新的文件夹 *orbslam2*。
   - 把 *ORB_SLAM2* 移到 *orbslam2* 里面。
   - 在 *orbslam2* 下创建一个新的文件夹 *orbslam2-dependencies*，把 gitee 仓库下 */.devcontainer/extendmodel* 里的 3 个依赖包复制到 *orbslam2-dependencies* 下<br>
    > 将源码和依赖包分开的原因：在构建容器的过程中就已经通过 COPY 操作把依赖包复制到了容器内（因为需要在容器内 build），而 remote container 在构建完容器后，还会把主机 vscode 打开的文件夹（即源代码文件夹）复制到容器内，如果把依赖包放在源代码文件夹下，相当于第二次把这些依赖复制到容器内，造成不必要的空间浪费。
4. 使用 VSCode Remote Container 构建镜像
   - 在主机终端输入 `xhost +`，许可所有用户都可访问 xserver（此设置在主机重启后会失效）。
   - 用 VSCode 打开文件夹 *ORB_SLAM2*（注意是源代码文件夹 *ORB_SLAM2*，不是 *orbslam2*），然后在 command palatte 中输入 `Reopen in Container` 即会开始构建镜像。

### devcontainer.json 和 Dockerfile 的改动（相较于 gitee 仓库）
- [devcontainer.json](./.devcontainer/devcontainer.json)
  - Build context: 把 build context 设置为了安装步骤里的 *orbslam2* 文件夹，使得 *Dockerfile* 里的 COPY 指令能够在 build context 下找到依赖包。
  - Mount Point: 把 */dev* 文件夹全部挂载上了，因为不知道 realsense camera 用的是 */dev* 里的哪个设备号。
  - User: 去掉了 runArgs 里的 `-u` 参数，即没有设置 non-root user 了，直接用的 root user。
- [Dockerfile](./.devcontainer/Dockerfile)
  - Base image：不用 ubuntu:bionic，直接用装好 ROS Melodic 的 ubuntu bionic 镜像 ros:melodic-ros-base-bionic。但是要注意这样安装的 ROS 是不能直接在命令行调用 ROS command 的，需要在 `~/.bashrc` 中添加 `source /opt/ros/melodic/setup.bash`，才能保证每次开启终端的时候会同时得到 ROS command 的 access.（参考 [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)）
  - User: 删除了几个关于 user 的 `ARG` 指令，直接用 root user。
  - Downloading source: 下载源换成了中科大的源，比清华源快很多。
  - DEBIAN_FRONTEND: 不再通过 `ENV` 的方式来设置环境变量 DEBIAN_FRONTEND，改为即用即销，即哪一层需要用到 DEBIAN_FRONTEND 就在哪一层设置，并且用完后立即 unset。
  - More packages installation
    - RealSense SDK & RealSense ROS Package
    - Lidar dependencies & Lidar ROS Package
    - ...

### Trouble shooting
#### Problem #1: Certificate verification failed: The certificate is NOT trusted.
- 先尝试了网上说的重装 ca-certificates，无果（通过 apt install 或手动下载 deb 包安装都没用）。
- 后面参考 [sudo apt update --> Certificate verification failed](https://blog.csdn.net/qlexcel/article/details/120642914) 直接把 *Dockerfile* 里面清华源的 https 换成 http，成功了。
- 最后，发现其实是因为 https 协议在传输过程中用了 SSL/TLS 数据加密，导致 http 代理无法正常接收数据，具体原因参考 [http-可用-https-不可用](../../general/proxy.md#http-可用-https-不可用)，因此要通过设置 `"http.proxyStrictSSL": false`，才能正常从 https 网站上下载。

#### Problem #2: Client network socket disconnected before secure TLS connection was established 或 XHR failed
- 首先查的log文件的最底部的报错信息，就是 *XHR failed*。看到 [stackoverflow](https://stackoverflow.com/questions/70177216/visual-studio-code-error-while-fetching-extensions-xhr-failed) 上面有个回答说是因为 vscode 和 PC 设置的 proxy 冲突了。
- 随后就把log文件往上看了看，发现确实是因为网络问题，还有个报错信息就是 *Client network socket disconnected before secure TLS connection was established*，于是就知道应该是小飞机的问题，应该是容器没有接上主机的代理。通过一个 [github issue](https://github.com/microsoft/vscode-remote-release/issues/986) 找到了给容器设置代理的方法，即在 *devcontainer.json* 的 settings 里设置 `http.proxy`（`https.proxy` 也可以加上，但加上的话一定还要加一句 `"http.proxyStrictSSL": false`）。

#### Problem #3: Pangolin X11: Failed to open X display
- 参考 [askubuntu](https://askubuntu.com/questions/432255/what-is-the-display-environment-variable) ，在既有集显又有独显的电脑上，`DISPLAY=:0` 应该表示显示设备为集显，而 `DISPLAY=:1` 则表示显示设备为独显。在终端查看 `echo $DISPLAY` 发现是 `:1`，说明现在用的是独显，因此在 devcontainer.json 里面也应该把 pangolin 的窗口输出到独显上，即设置 `DISPLAY=:1`。

#### Problem #4: No protocol specified.
- 如果重启了主机，但没有用 `xhost +` 来许可所有用户都可访问 xserver，启动镜像后运行 `realsense-viewer` 就会报这个错误。

#### Warnings
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


## 参考链接
1. [ORB-SLAM2 Docker 容器 (gitee)](https://gitee.com/wycan/orbslam2_runin_docker)
2. [sudo apt update --> Certificate verification failed](https://blog.csdn.net/qlexcel/article/details/120642914)
3. [vscode --> XHR failed](https://stackoverflow.com/questions/70177216/visual-studio-code-error-while-fetching-extensions-xhr-failed)
4. [vscode --> Client network socket disconnected](https://github.com/microsoft/vscode-remote-release/issues/986)

> 备用链接
> 1. [带 GPU 驱动 (libnvidia-container) 的安装方法](https://blog.csdn.net/WEINILUO/article/details/118659410)
> 2. [ORB-SLAM2 Docker 容器 (docker hub)](https://hub.docker.com/r/celinachild/orbslam2)
> 3. [ORB-SLAM3 Docker 容器 (github)](https://github.com/jahaniam/orbslam3_docker)