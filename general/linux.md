> 本文主要记录与 Linux 系统相关的学习笔记

> CONTENT
- [Basics](#basics)
  - [Ubuntu 安装](#ubuntu-安装)
  - [Ubuntu 扩容](#ubuntu-扩容)
  - [Linux 环境变量文件](#linux-环境变量文件)
  - [Ubuntu 格式化U盘](#ubuntu-格式化u盘)
- [Troubleshooting](#troubleshooting)
  - [curl: (6) Could not resolve host: ...](#curl-6-could-not-resolve-host-)
  - [E: 无法获得锁 /var/lib/dpkg/lock - open (11: Resource temporarily unavailable)](#e-无法获得锁-varlibdpkglock---open-11-resource-temporarily-unavailable)

## Basics
### Ubuntu 安装
> 参考官网：[Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)<br>
PS：如果还没买电脑，可以先参考 [Ubuntu certified hardware](https://ubuntu.com/certified?q=&limit=20&category=Desktop&category=Laptop)，看看想买的电脑型号对 ubuntu 的支持性如何。

### Ubuntu 扩容
1. 在ubuntu下制作启动盘。参考 [Create a bootable USB stick on Ubuntu](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview)。ubuntu 镜像下载地址：[阿里云](http://mirrors.aliyun.com/ubuntu-releases/20.04/)。<br>
   如果使用 win 键找到不到 Startup Disk Creator，试试换个名字搜索：usb-creator-gtk，如果发现没安装，通过 `sudo apt install usb-creator-gtk` 安装。
2. 使用GParted进行扩容。参考 [Ubuntu无损扩展分区(目录)容量的方法](https://blog.csdn.net/jwq2011/article/details/54599744)。

### Linux 环境变量文件
参考 [profile、bashrc、~/.bash_profile、~/.bashrc、~/.bash_profile之间的区别和联系以及执行顺序](https://blog.csdn.net/gatieme/article/details/45064705)
- `/etc/profile`：为系统的每个用户设置环境信息，当用户第一次登录时，该文件被执行。
- `/etc/bashrc`：为每一个运行 bash shell 的用户执行此文件。
- `~/.bash_profile` 或 `~/.profile`：作用域仅限当前用户，可使用该文件输入专用于当前用户使用的 shell 信息，用户登录时，该文件仅仅执行一次。在 Debian 中使用 .profile 文件代替 .bash_profile 文件以兼容不同的 shell。
- `~/.bashrc`：作用域仅限当前用户，当用户登录时以及**每次打开新的 shell 时**，该文件被读取。

执行顺序见下图<br>
<img src=../img/linux_1.png width=50%><br>

### Ubuntu 格式化U盘
- `sudo mkfs.vfat -F 32 /dev/sdb1` 即可将u盘格式化为fat32格式，其中 `/dev/sdb1` 是 U 盘的挂载位置。
- 如果U盘上写入了文件系统，直接用上面那条命令格式化可能会报错：*mkdosfs: /dev/sdb1 contains a mounted file system*，此时需要先 `sudo umount /dev/sdb1` 然后再执行 `sudo mkfs.vfat -F 32 /dev/sdb1`。
- 如果格式化之后，往U盘里复制文件出现权限问题
  - 先打开U盘目录，然后右键，“在终端打开”，就可以找到U盘在文件系统中的路径名，如 */media/yan/1a27ccc0-c38e-4b5d-a8c2-96dd49c13f2c*。
  - 再给这个路径读写权限：`sudo chmod 777 /media/yan/1a27ccc0-c38e-4b5d-a8c2-96dd49c13f2c`

## Troubleshooting
### curl: (6) Could not resolve host: ...
- 报错描述：在使用 curl 下载时报错
- 报错原因：参考 [stackoverflow](https://stackoverflow.com/questions/24967855/curl-6-could-not-resolve-host-google-com-name-or-service-not-known)，发现应该是 DNS Server 的问题，于是打开 DNS Server 配置文件
  ```bash
  eaibot@DashgoD1:~$ cat /etc/resolv.conf
  # Dynamic resolv.conf(5) file for glibc resolver(3) generated by resolvconf(8)
  #     DO NOT EDIT THIS FILE BY HAND -- YOUR CHANGES WILL BE OVERWRITTEN
  ```
  发现 */etc/resolv.conf* 居然连 `nameserver 192.168.31.200` 都没有（192.168.31.200 是自己的 ip）。
- 解决方法
  - 首先尝试加入本地 ip 作为 DNS server：`nameserver 192.168.31.200`，发现没用
  - 再加入 google 的 DNS server：`nameserver 8.8.8.8`，问题解决（只不过每次重启之后 */etc/resolv.conf* 又变为默认了）

### E: 无法获得锁 /var/lib/dpkg/lock - open (11: Resource temporarily unavailable)
- 报错描述：sudo apt-get install ... 报错
- 报错原因：有其他进程占用了 apt
- 解决办法：查看有哪些进程占用了 apt，然后 kill 掉这些进程
  ```bash
  eaibot@DashgoD1:~$ ps -e | grep apt
  15621 ?        00:00:00 apt.systemd.dai
  eaibot@DashgoD1:~$ sudo kill 15621
  ```