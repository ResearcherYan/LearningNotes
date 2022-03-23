> CONTENT
- [Docker Basics](#docker-basics)
  - [Docker简介](#docker简介)
  - [Docker的三个基本概念](#docker的三个基本概念)
    - [镜像 Image](#镜像-image)
    - [容器 Container](#容器-container)
    - [仓库 Repository](#仓库-repository)
  - [使用镜像](#使用镜像)
    - [获取镜像](#获取镜像)
    - [运行镜像](#运行镜像)
    - [列出镜像](#列出镜像)
    - [删除镜像](#删除镜像)
  - [操作容器](#操作容器)
    - [启动容器](#启动容器)
    - [守护态运行](#守护态运行)
    - [终止容器](#终止容器)
    - [进入容器](#进入容器)
    - [导出和导入](#导出和导入)
    - [删除容器](#删除容器)
- [Docker in VSCode](#docker-in-vscode)
  - [Tutorial #1: Create and share a Docker app with Visual Studio Code](#tutorial-1-create-and-share-a-docker-app-with-visual-studio-code)
  - [Tutorial #2: Persist data in a container app using volumes in VS Code](#tutorial-2-persist-data-in-a-container-app-using-volumes-in-vs-code)
# Docker Basics
> 参考链接：[Docker - 从入门到实践](https://yeasy.gitbook.io/docker_practice/)

## Docker简介
> Docker 属于操作系统层面的虚拟化技术，用Go语言开发。
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
上面的 pull 过程展示了镜像的分层储存，分了 3 层下载 ubuntu:18.04 镜像。
### 运行镜像
- 举例：`docker run -it --rm ubuntu:18.04 bash`
  - `-it`：这是两个参数，一个是 `-i` 交互式操作，一个是 `-t` 终端。合一起就是提供一个交互式终端环境。
  - `--rm`：容器退出后随之将其删除。
  - `ubuntu:18.04`：用 `ubuntu:18.04` 镜像为基础来启动容器。
  - `bash`：放在镜像名后的是**命令**，这里希望有个交互式 Shell，因此用的是 bash。
### 列出镜像
- `docker image ls`：查看所有已下载顶层镜像
  - 注意：镜像 ID（IMAGE ID）和摘要（DIGEST）是镜像的唯一标识，一个镜像可以对应多个标签（TAG）。
- `docker system df`：查看镜像所占空间
- `docker image ls [选项]`：列出部分镜像
  - `docker image ls [repository]`：指定仓库名
  - `docker image ls [repository:tag]`：指定仓库名和标签
  - `docker image ls -f since=[repository:tag]`：列出在某个镜像之后建立的镜像
  - `docker image ls -f label=[your-label]`：列出某个具有某个 label 的镜像
### 删除镜像
- `docker image rm [选项] <镜像1> [<镜像2> ...]`
  - 用镜像 ID 删除：不用是全 ID ，用前面一部分短 ID 就行了
  - 用镜像名删除：`<仓库名>:<标签>`
- `docker image rm $(docker image ls -q redis)`：批量删除名为redis的镜像
- `docker image prune`：删除虚悬镜像。虚悬镜像的仓库和ID均为 `<none>`，一般是因为镜像维护发布了新版本，旧版本的名称就被取消了，这种镜像没有价值，可以随意删除。

## 操作容器
### 启动容器
- 新建并启动
  - `docker run ubuntu:18.04 /bin/echo 'Hello world'`：输出一个 “Hello World”，之后终止容器
  - `docker run -it ubuntu:18.04 /bin/bash`：启动一个 bash 终端，并保持打开
  - `docker run` 背后的后台操作
    - 检查本地是否存在指定的镜像，不存在就从 registry 下载
    - 利用镜像创建并启动一个容器
    - 分配一个文件系统，并在只读的镜像层外面挂载一层可读写层
    - 从宿主主机配置的网桥接口中桥接一个虚拟接口到容器中去
    - 从地址池配置一个 ip 地址给容器
    - 执行用户指定的应用程序
    - 执行完毕后容器被终止
- 启动已终止容器
  - `docker container start`：将一个已经终止（exited）的容器启动运行
- 查看容器信息
  - `docker container ls`：查看正处于激活状态的容器。如果加上 `-a`表示查看所有容器（包括终止的容器）。
  - `ps` 或 `top`：查看某容器内部的进程信息（**在该容器的终端运行**）
### 守护态运行
`-d` 参数可以让docker在后台运行
- 不用 `-d`：容器会把输出的结果打印到宿主机上
```shell
$ docker run ubuntu:18.04 /bin/sh -c "while true; do echo hello world; sleep 1; done"
hello world
hello world
hello world
hello world
```
- 使用 `-d`：容器会在后台运行，输出结果可用 `docker logs` 查看
```shell
$ docker run -d ubuntu:18.04 /bin/sh -c "while true; do echo hello world; sleep 1; done"
77b2dc01fe0f3f1265df143181e7b9af5e05279a884f4776ee75350ea9d8017a
$ docker container ls
CONTAINER ID  IMAGE         COMMAND               CREATED        STATUS       PORTS NAMES
77b2dc01fe0f  ubuntu:18.04  /bin/sh -c 'while tr  2 minutes ago  Up 1 minute        agitated_wright
$ docker container logs [container ID or NAMES]
hello world
hello world
hello world
. . .
```
### 终止容器
- 手动终止：`docker container stop`
- 自动终止：当 Docker 容器中指定的应用终结时，容器将自动终止
- 重启：`docker container restart`
### 进入容器
`-d` 使容器进入后台运行，而 `attach` 或 `exec` 命令可以进入正在后台运行的容器。
- `attach` 命令（不建议用）：执行 `exit` 会导致容器停止
```shell
$ docker run -dit ubuntu
243c32535da7d142fb0e6df616a3c3ada0b8ab417937c853a9e1c251f499f550

$ docker container ls
CONTAINER ID        IMAGE               COMMAND             CREATED             STATUS              PORTS               NAMES
243c32535da7        ubuntu:latest       "/bin/bash"         18 seconds ago      Up 17 seconds                           nostalgic_hypatia

$ docker attach 243c
root@243c32535da7:/#
```
- `exec` 命令（推荐使用）：执行 `exit` 不会导致容器停止。后边可以跟多个参数，如 `-i`, `-t` 等
```shell
$ docker run -dit ubuntu
69d137adef7a8a689cbcb059e94da5489d3cddd240ff675c640c8d96e84fe1f6

$ docker container ls
CONTAINER ID        IMAGE               COMMAND             CREATED             STATUS              PORTS               NAMES
69d137adef7a        ubuntu:latest       "/bin/bash"         18 seconds ago      Up 17 seconds                           zealous_swirles

$ docker exec -it 69d1 bash
root@69d137adef7a:/#
```
### 导出和导入
- `docker export`：导出容器快照
```shell
$ docker container ls -a
CONTAINER ID        IMAGE               COMMAND             CREATED             STATUS                    PORTS               NAMES
7691a814370e        ubuntu:18.04        "/bin/bash"         36 hours ago        Exited (0) 21 hours ago                       test
$ docker export 7691a814370e > ubuntu.tar
```
- `docker import`：导入容器快照
  - 通过镜像名导入
  ```shell
  $ cat ubuntu.tar | docker import - test/ubuntu:v1.0
  $ docker image ls
  REPOSITORY          TAG                 IMAGE ID            CREATED              VIRTUAL SIZE
  test/ubuntu         v1.0                9d37a6082e97        About a minute ago   171.3 MB
  ```
  - 通过 URL 或目录导入
  ```shell
  $ docker import http://example.com/exampleimage.tgz example/imagerepo
  ```
- `docker load`：导入镜像储存文件。镜像储存文件与容器快照的区别：
  - 容器快照文件将丢弃所有的历史记录和元数据信息（即仅保存容器当时的快照状态），并且从容器快照文件导入时可以重新指定标签等元数据信息。
  - 镜像存储文件将保存完整记录，体积也更大一些。
### 删除容器
- `docker container rm [container-name]`：删除一个处于终止状态的容器。如果加上 `-f` 参数，会进行强制删除，可用于删除运行中的容器。
- `docker container prune`：清理所有处于终止状态的容器

---

# Docker in VSCode
> 参考链接（备查）<br>
> 1. [Docker tutorial for VS Code users](https://docs.microsoft.com/en-us/visualstudio/docker/tutorials/docker-tutorial)
> 2. [Docker in Visual Studio Code](https://code.visualstudio.com/docs/containers/overview)
> 3. [Developing inside a Container](https://code.visualstudio.com/docs/remote/containers)

## Tutorial #1: Create and share a Docker app with Visual Studio Code
> Link - [Tutorial: Create and share a Docker app with Visual Studio Code](https://docs.microsoft.com/en-us/visualstudio/docker/tutorials/docker-tutorial)

- General Procedures
  - Make your program prepared.
  - Write a *Dockerfile*. Which may include:
    - Base Image
    - Working directory
    - Folders to be copied
    - Dependencies to be installed
    - Command lines to be executed when container excited
  - Build a container image according to the *Dockerfile*.
  - Start your container and test the program.
  - Share your image on a registry.
  - Run the image on a new instance.

## Tutorial #2: Persist data in a container app using volumes in VS Code
> Link - [Tutorial: Persist data in a container app using volumes in VS Code](https://docs.microsoft.com/en-us/visualstudio/docker/tutorials/tutorial-persist-data-layer-docker-app-with-vscode)

- Pesist data using named volumes (*Recommend*)
  - Create a volume: `docker volume create todo-db`<br>
  docker 会在硬盘上创建一个 volume ，这个 volume 的位置可以通过 `docker volume inspect todo-db` 来查看，发现 todo.db 文件就储存在 MountPoint 指向的位置。
  - Run a container and mount the volume to a container's folder: `docker run -dp 3000:3000 -v todo-db:/etc/todos getting-started`<br>
  docker 先开启一个容器，然后把刚刚创建的 volume（在主机的硬盘上）挂载到**该容器文件系统下**的 /etc/todos 位置。先执行 `docker exec -it [container-ID]` 进入容器的shell环境，然后 `ls /etc/todos` 就可以看到 todo.db 文件。

- Pesist data using bind mount<br>
  - Run a container and mount a host folder to a container's folder: `docker run -dp 3000:3000 -w /app -v ${PWD}:/app node:12-alpine sh -c "yarn install && yarn run dev"`
    - `-w /app` Working directory inside the container.
    - `-v ${PWD}:/app` Bind mount the current directory from the host in the container into the `/app` directory.
    - `sh -c "yarn install && yarn run dev"` A shell command executed in the container.
  - (Optional) Watch the logs: `docker logs -f <container-id>`. When the following messages show, it indicates that the app is running.
  ```shell
  $ nodemon src/index.js
  [nodemon] 1.19.2
  [nodemon] to restart at any time, enter `rs`
  [nodemon] watching dir(s): *.*
  [nodemon] starting `node src/index.js`
  Using sqlite database at /etc/todos/todo.db
  Listening on port 3000
  ```
  这里会发现 `yarn install` 这一步会花很多时间，如果以后要用的docker环境需要安装很多依赖（这很可能），每次都要重新 run 一个新的container，效率就会很低。对于常用的开发环境，可以选择使用完 container 之后不要 remove，只用 stop 掉就好了。下次启动的时候使用 `docker container start [OPTIONS] CONTAINER]` 重新启动该容器即可。<br>
  但其实 `docker container start` 命令的 options 其实是很少的，所以很多东西在第一次创建container的时候就已经定下来了，后面没法改，比如这个例子里面的端口映射。

- The difference between Named Volumes and Bind Mount
<img src=img/docker_1.png>