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
考虑到开发和部署环境的一致性需求，选择用 docker 的方式安装 orbslam2。在 gitee 上发现有人把 orbslam2 docker 环境打包好了，可以直接使用 vscode 的 remote container 插件实现容器的创建。总的安装过程参考 [ORB-SLAM2 Docker 容器 (gitee)](https://gitee.com/wycan/orbslam2_runin_docker)。

### Trouble shooting
- Problem #1: Certificate verification failed: The certificate is NOT trusted.<br>
  - 先尝试了网上说的重装 ca-certificates，无果（通过 apt install 或手动下载 deb 包安装都没用）。
  - 后面参考 [sudo apt update --> Certificate verification failed](https://blog.csdn.net/qlexcel/article/details/120642914) 直接把 *Dockerfile* 里面清华源的 https 换成 http，成功。（可能也可以通过设置代理的方式？）
- Problem #2: Client network socket disconnected before secure TLS connection was established 或 XHR failed<br>
  - 首先查的log文件的最底部的报错信息，就是 *XHR failed*。看到 [stackoverflow](https://stackoverflow.com/questions/70177216/visual-studio-code-error-while-fetching-extensions-xhr-failed) 上面有个回答说是因为 vscode 和 PC 设置的 proxy 冲突了。
  - 随后就把log文件往上看了看，发现确实是因为网络问题，还有个报错信息就是 *Client network socket disconnected before secure TLS connection was established*，于是就知道应该是小飞机的问题，应该是容器没有接上主机的代理。通过一个 [github issue](https://github.com/microsoft/vscode-remote-release/issues/986) 找到了给容器设置代理的方法，即在 *devcontainer.json* 的 settings 里设置 http.proxy（也可以加上 https.proxy）。
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


  