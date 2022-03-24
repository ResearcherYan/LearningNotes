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
  