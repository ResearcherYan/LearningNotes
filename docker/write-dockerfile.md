> 参考链接：[Best practices for writing Dockerfiles](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/)

> CONTENT
- [General guidelines and recommendations](#general-guidelines-and-recommendations)
  - [Container building process](#container-building-process)
  - [Understand build context](#understand-build-context)
  - [Minimize the number of layers](#minimize-the-number-of-layers)
  - [Leverage build cache](#leverage-build-cache)
  - [* Create ephemeral containers](#-create-ephemeral-containers)
  - [* Pipe Dockerfile through `stdin`](#-pipe-dockerfile-through-stdin)
  - [* Exclude with .dockerignore](#-exclude-with-dockerignore)
  - [* Use multi-stage builds](#-use-multi-stage-builds)
  - [* Don’t install unnecessary packages](#-dont-install-unnecessary-packages)
  - [* Decouple applications](#-decouple-applications)
  - [* Sort multi-line arguments](#-sort-multi-line-arguments)
- [Dockerfile instructions](#dockerfile-instructions)
  - [FROM](#from)
  - [RUN](#run)
    - [apt-get](#apt-get)
    - [* Using pipes](#-using-pipes)
  - [ADD or COPY](#add-or-copy)
  - [CMD](#cmd)
  - [ENV](#env)
  - [* EXPOSE](#-expose)
  - [* LABEL](#-label)

## General guidelines and recommendations
### Container building process
- Image building: 首先需要根据 *Dockerfile* 来 build image。Image 是由多个 read-only layer 组成的，而每一个 read-only layer 都是通过 *Dockerfile* 里面的一条指令所生成的，*Dockerfile* 后面的指令所生成的 layer 是堆砌在前面的 layer 上方的。
- Container building: 制作好 image 之后，每次运行 image 的时候会添加一个 writable layer(the "container layer")，这一层是位于最上方的。

### Understand build context
当运行 `docker build` 命令时，当前的工作目录就被称为 *build context*。默认 *Dockerfile* 的位置就是在 build context 下，可以通过 `-f` 来指定 *Dockerfile* 的位置，但不论 *Dockerfile* 在哪，所有当前目录的子目录和子文件都会被作为 build context 发送给 Docker daemon。

### Minimize the number of layers
尽可能减少镜像的层数。事实上，Docker 内部为了减少镜像层数，也做了一些努力：
- Only the instructions `RUN`, `COPY`, `ADD` create layers. Other instructions create temporary intermediate images, and do not increase the size of the build.
- Support multi-stage builds.

### Leverage build cache
在根据 *Dockerfile* 创建新的 layer 的时候，Docker 会先检查有没有现有的 image cache 可以复用的，如果有的话就可以直接使用 cache 创建这一层了。Docker 使用 build cache 的规则大体如下：
- 首先从 cache 里的 parent image 出发，然后和这个 parent image 衍生出的所有的 child image 作比对，看看有没有相同的 build instruction，如果没有，则这一步无法使用 cache。
- 一般比对 *Dockerfile* 里的 build instruction 就够了，但个别命令还需要进一步的比对。`ADD` 和 `COPY` 指令还需要比较指定的文件夹或者文件内容有无变动，如果有变动，则这一步无法使用 cache。
- 一旦到了某一层无法使用 cache，后面的每一层都会直接 rebuild，Docker 不会再考虑 cache 复用了。

---

### * Create ephemeral containers
你写的 *Dockerfile* 要尽可能保证你的 container 重建起来很快

### * Pipe Dockerfile through `stdin`
当需要用到 one-off build（不把 *Dockerfile* 写入磁盘的）的时候，需要通过终端命令的方式构建 *Dockerfile*，具体方式查看文章首部贴出的的参考链接。

### * Exclude with .dockerignore
可以通过 `.dockerignore` 文件指定排除一些 build context 下与 build 无关的文件，不把他们发送给 Docker daemon。这主要是为了防止 `ADD` 或 `COPY` 可能不小心把某些很大的文件或者比较 sentitive 的文件也加入到了 image 中。

### * Use multi-stage builds
这个在 [docker.md](docker.md#multi-stage-builds) 里面讲过了

### * Don’t install unnecessary packages
不要安装用不上的包

### * Decouple applications
Keep your containers clean and modular.<br>
一个 container 最好只有一个主要用途，并且最好只运行一个进程（不是硬性规定）。


### * Sort multi-line arguments
为了防止包的重复安装、提高可读性，建议 multi-line arguments 按照字母顺序来排列，每行的结尾为 **空格+右划线 ( \ )**。示例如下所示：
```Dockerfile
RUN apt-get update && apt-get install -y \
  bzr \
  cvs \
  git \
  mercurial \
  subversion \
  && rm -rf /var/lib/apt/lists/*
```

## Dockerfile instructions
### FROM
用于选择 base image.
### RUN
#### apt-get
- Always combine `RUN apt-get update` with `apt-get install` in the same RUN statement. 如果不同时使用的话，可能会导致 caching issues。
- 可以通过删除 `/var/lib/apt/lists/` 来减少 image 的大小。官方的 Debian 和 Ubuntu 镜像会自动运行 `apt-get clean`, 所以就不用再显式去删除了。
```Dockerfile
RUN apt-get update && apt-get install -y \
    aufs-tools \
    automake \
    build-essential \
    curl \
    dpkg-sig \
    libcap-dev \
    libsqlite3-dev \
    mercurial \
    reprepro \
    ruby1.9.1 \
    ruby1.9.1-dev \
    s3cmd=1.1.* \
 && rm -rf /var/lib/apt/lists/*
```
#### * Using pipes
Pipelines ( | ) 是 linux 命令的一种特殊符号，作用是连结上个指令的标准输出，做为下个指令的标准输入。比如下面这条命令，就是从某网站上下载某文件，然后将终端输出的行数写入到 /number 文件里。
```dockerfile
RUN wget -O - https://some.site | wc -l > /number
```
在 docker 里，只要 pipe 里的最后一条指令成功了，就不会报错。在上面这个例子里，即使 wget 失败了，只要 wc -l 成功了 docker 就不会报错。<br>
如果要求只要 pipe 里的任一条指令失败，docker 就会报错，需要加上 `set -o pipefail &&`：
```dockerfile
RUN set -o pipefail && wget -O - https://some.site | wc -l > /number
```
PS: 不是所有的 shell 都支持 `-o pipefail` 的。比如在基于 Debian 的镜像里的 `dash` shell，这时就需要使用 the *exec* form of `RUN` 来明确说明要用哪个 shell。示例如下
```dockerfile
RUN ["/bin/bash", "-c", "set -o pipefail && wget -O - https://some.site | wc -l > /number"]
```

### ADD or COPY
`ADD` 和 `COPY` 的功能差不多，但一般更倾向于使用 `COPY`，因为它的操作更透明一些，`COPY` 只支持把本地文件复制到容器中，但 `ADD` 会有一些其他不那么透明的用法。
- `ADD`
  - 当需要把 tar 文件自动提取到容器中时，往往会用到 `ADD`，如 `ADD rootfs.tar.xz /`。
  - 不要用 `ADD` 从远程 URL 下载东西，使用 curl 或 wget。
- `COPY`
  - 如果 *Dockerfile* 里有多个步骤用到了不同的文件，建议将他们分开 `COPY`，这样可以更好的利用 build cache，如果一起全 `COPY` 的话会导致只要有一个文件有改动就要重新 `COPY` 所有的。

### CMD
一个 *Dockerfile* 里只能有一个 `CMD` 指令，如果使用多个 `CMD` 的话，只有最后一个会被执行。
- 运行 executable：`CMD ["executable", "param1", "param2"…]`
- 运行 service：`CMD ["service","-DFOREGROUND"]`
- 运行 interative shell: 如 `CMD sh`, `CMD ["python"]`

### ENV
- `ENV` 用于一般指定环境变量，如 `ENV PATH=/usr/local/nginx/bin:$PATH`。
- `ENV` 还可以用来设置一些常用的软件版本号，如下
```dockerfile
ENV PG_MAJOR=9.3
ENV PG_VERSION=9.3.4
RUN curl -SL https://example.com/postgres-$PG_VERSION.tar.xz | tar -xJC /usr/src/postgres && …
ENV PATH=/usr/local/postgres-$PG_MAJOR/bin:$PATH
```
值得注意的是，`ENV` 会创建一个中间层，因此其设置的环境变量会留在这个中间层，即使在后面的层把该环境变量的值改了，中间层的环境变量仍还是原值。如果想把环境变量即用即销的话，可以使用 `RUN`：
```dockerfile
# syntax=docker/dockerfile:1
FROM alpine
RUN export ADMIN_USER="mark" \
    && echo $ADMIN_USER > ./mark \
    && unset ADMIN_USER
CMD sh
```


### * EXPOSE
`EXPOSE` 指令用于指定容器所监听的端口。

### * LABEL
为 image 添加 label（可添加多个 label ），需要注意 label 的语法。

