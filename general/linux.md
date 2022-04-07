> 本文主要记录与 Linux 系统相关的学习笔记

> CONTENT
- [Ubuntu 安装](#ubuntu-安装)
- [Ubuntu 扩容](#ubuntu-扩容)
- [Linux 环境变量文件](#linux-环境变量文件)
## Ubuntu 安装
> 参考官网：[Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)<br>
PS：如果还没买电脑，可以先参考 [Ubuntu certified hardware](https://ubuntu.com/certified?q=&limit=20&category=Desktop&category=Laptop)，看看想买的电脑型号对 ubuntu 的支持性如何。

## Ubuntu 扩容
1. 在ubuntu下制作启动盘。参考 [Create a bootable USB stick on Ubuntu](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview)。ubuntu 镜像下载地址：[阿里云](http://mirrors.aliyun.com/ubuntu-releases/20.04/)。<br>
   如果使用 win 键找到不到 Startup Disk Creator，试试换个名字搜索：usb-creator-gtk，如果发现没安装，通过 `sudo apt install usb-creator-gtk` 安装。
2. 使用GParted进行扩容。参考 [Ubuntu无损扩展分区(目录)容量的方法](https://blog.csdn.net/jwq2011/article/details/54599744)。

## Linux 环境变量文件
参考 [profile、bashrc、~/.bash_profile、~/.bashrc、~/.bash_profile之间的区别和联系以及执行顺序](https://blog.csdn.net/gatieme/article/details/45064705)
- `/etc/profile`：为系统的每个用户设置环境信息，当用户第一次登录时，该文件被执行。
- `/etc/bashrc`：为每一个运行 bash shell 的用户执行此文件。
- `~/.bash_profile` 或 `~/.profile`：作用域仅限当前用户，可使用该文件输入专用于当前用户使用的 shell 信息，用户登录时，该文件仅仅执行一次。在 Debian 中使用 .profile 文件代替 .bash_profile 文件以兼容不同的 shell。
- `~/.bashrc`：作用域仅限当前用户，当用户登录时以及**每次打开新的 shell 时**，该文件被读取。

执行顺序见下图<br>
<img src=../img/linux_1.png><br>