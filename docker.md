# Docker Learning Notes
> 参考网站：[Docker - 从入门到实践](https://yeasy.gitbook.io/docker_practice/)

## Docker简介
Docker 属于操作系统层面的虚拟化技术。
- 虚拟机与 Docker 的区别
  - 虚拟机：虚拟出一套硬件后，在其上运行一个完整操作系统。
  - Docker：应用进程直接运行于宿主的内核，容器内没有自己的内核，而且也没有进行硬件虚拟。
- Docker 的优势
  - 系统资源利用更高效
  - 启动速度更快
  - 通过 Dockerfile 保证了运行环境的一致性，容易移植
- [Docker 的安装](https://yeasy.gitbook.io/docker_practice/install)

## Docker的三个基本概念
### 镜像 Image
- 镜像的功能：操作系统分为内核和用户空间。对于 Linux 而言，内核启动后，会挂载 root 文件系统为其提供用户空间支持。而 Docker 镜像，就相当于是一个 root 文件系统。
- 镜像的存储：分层存储。镜像构建时，会一层层构建，前一层是后一层的基础。每一层构建完就不会再发生改变，后一层上的任何改变只发生在自己这一层。
### 容器 Container
- 镜像与容器的关系：镜像是静态的定义，容器是镜像运行时的实体。（就像类与实例一样）
- 容器的特性：容器的实质是进程，这个进程运行在一个隔离的环境里。容器可以拥有自己的 root 文件系统、自己的网络配置、自己的进程空间，甚至自己的用户 ID 空间。
- 容器的构建：每一个容器运行时，是以镜像为基础层，在其上创建一个当前容器的存储层。官方推荐，所有的文件写入操作，都应该使用数据卷（Volume）或绑定宿主目录，因为这样可以保证容器删除之后数据不会丢失。
### 仓库 Repository
- 公共 Docker Registry
  - 官方：[Docker Hub](https://hub.docker.com/)，这也是默认的 Registry。（国内访问可能比较慢）
  - 国内的镜像服务（Registry Mirror）,也称为加速器。如[阿里云加速器](https://www.aliyun.com/product/acr?source=5176.11533457&userCode=8lx5zmtu)、[DaoCloud加速器](https://www.daocloud.io/mirror#accelerator-doc)、[网易云镜像服务](https://c.163.com/hub#/m/library/)。
- 私有 Docker Registry

## 使用镜像
### 获取镜像
- 格式：`docker pull [选项] [Docker Registry 地址[:端口号]/]仓库名[:标签]`
- 举例
```shell
$ docker pull ubuntu:18.04
18.04: Pulling from library/ubuntu
92dc2a97ff99: Pull complete
be13a9d27eb8: Pull complete
c8299583700a: Pull complete
Digest: sha256:4bc3ae6596938cb0d9e5ac51a1152ec9dcac2a1c50829c74abd9c4361e321b26
Status: Downloaded newer image for ubuntu:18.04
docker.io/library/ubuntu:18.04
```
