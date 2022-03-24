## Ubuntu 安装
> 参考官网：[Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)<br>
PS：如果还没买电脑，可以先参考 [Ubuntu certified hardware](https://ubuntu.com/certified?q=&limit=20&category=Desktop&category=Laptop)，看看想买的电脑型号对 ubuntu 的支持性如何。
## Ubuntu 扩容

1. 在ubuntu下制作启动盘。参考 [Create a bootable USB stick on Ubuntu](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview)。ubuntu 镜像下载地址：[阿里云](http://mirrors.aliyun.com/ubuntu-releases/20.04/)。<br>
   如果使用 win 键找到不到 Startup Disk Creator，试试换个名字搜索：usb-creator-gtk，如果发现没安装，通过 `sudo apt install usb-creator-gtk` 安装。
2. 使用GParted进行扩容。参考 [Ubuntu无损扩展分区(目录)容量的方法](https://blog.csdn.net/jwq2011/article/details/54599744)。