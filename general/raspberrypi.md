> 说明：本文并不是一个标准的树莓派装机流程，使用的硬件很多都是看手头上有哪些东西就用哪些东西。

> CONTENT
- [现有硬件](#现有硬件)
- [配置步骤](#配置步骤)
  - [路由器连接校园网](#路由器连接校园网)
  - [操作系统安装](#操作系统安装)
  - [给树莓派设置静态 ip](#给树莓派设置静态-ip)
  - [使树莓派允许远程桌面控制](#使树莓派允许远程桌面控制)
  - [设置用户自动登录](#设置用户自动登录)
  - [设置 VNC 显示分辨率](#设置-vnc-显示分辨率)
  - [远程连接树莓派](#远程连接树莓派)
- [小结](#小结)
- [参考链接](#参考链接)


## 现有硬件
1. 树莓派 4B (8GB) + Argon one v2 树莓派外壳 + 5V 3A 充电头 + USB-TypeC 电源线
2. 三星 Pro Plus 128G SD 卡（读速度 160 MB/s，写速度 120 MB/s）+ SD Adapter（适配器是买卡的时候自带的）
3. 路由器 + 网线
4. 键鼠 + HDMI 显示器 + 公对公 HDMI 传输线（实验室服务器的配件，只能暂时用用）
5. VGA 显示器（实验室闲置品，可长期使用）+ 公对公 HDMI 转 VGA 传输线（特地为 VGA 显示器买的）

## 配置步骤
### 路由器连接校园网
因为有路由器和网线，网络配置起来就容易很多。<br>
把实验室的校园网网线接入到路由器的 WAN 口，然后在自己电脑上连接路由器的wifi（或者用网线连也行），此时电脑上就会自动弹出“热点登陆”的界面，输入校园网账号密码之后路由器就通网了，并且连接上的任意设备都能共享校园网。

### 操作系统安装
考虑到尽量保持开发环境和部署环境一致，选择给树莓派安装 Ubuntu 20.04 LTS。<br>
因为偶尔也需要在树莓派上做少量的开发工作，所以想装个带桌面的 Ubuntu 20.04，但参考 [How to install Ubuntu Desktop on Raspberry Pi 4](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview) 后发现 Ubuntu 官方为树莓派提供的带桌面的 Ubuntu 版本只有 20.10，没有 20.04，就采取了 [Install Ubuntu Desktop 20.04 LTS on Raspberry Pi 4](https://linuxhint.com/install-ubuntu-desktop-20-04-lts-on-raspberry-pi-4/) 的方法：先装 Ubuntu Server 20.04（不带桌面），再通过命令行安装 ubuntu-desktop。
- 烧写镜像：将 Ubuntu Server 20.04 烧写到 SD 卡中。
- 连线：将树莓派的网口连接到路由器的 LAN 口中的任意一个，连接键鼠、HDMI 显示器。（这里一定要连 HDMI 显示器，尝试了实验室的 3 个 VGA 显示器都不行）。
- 安装桌面：开机，按照 [Install Ubuntu Desktop 20.04 LTS on Raspberry Pi 4](https://linuxhint.com/install-ubuntu-desktop-20-04-lts-on-raspberry-pi-4/) 的步骤安装 GNOME 3 桌面。

### 给树莓派设置静态 ip
- 打开树莓派的终端，`cat /sys/class/net/wlan0/address` 查看无线网卡的 MAC 地址。
- 在浏览器中输入192.168.xxx.xxx（这个 ip 一般路由器上会写或者路由器说明书或官网上有），进入路由器管理页面。给上一步得到的 MAC 地址添加一个静态 IP。

### 使树莓派允许远程桌面控制
- 断开树莓派的有线网连接，转而连接路由器的 wifi（否则无法开启 Screen Sharing）
- 在 Settings > Sharing 里开启 Screen Sharing，设置远程登录密码，Networks 选择路由器的 wifi。

### 设置用户自动登录
Settings > Users > 开启 Automatic Login

### 设置 VNC 显示分辨率
`sudo gedit /boot/firmware/config.txt`，然后复制下面这段进去
```txt
# uncomment the following to adjust overscan. Use positive numbers if console
# goes off screen, and negative if there is too much border
overscan_left=0
overscan_right=0
overscan_top=0
overscan_bottom=0

# uncomment to force a console size. By default it will be display's size minus
# overscan.
framebuffer_width=1440
framebuffer_height=900

# uncomment if hdmi display is not detected and composite is being output
hdmi_force_hotplug=1

# uncomment to force a specific HDMI mode (this will force VGA)
max_usb_current=1
hdmi_group=2
hdmi_mode=87
hdmi_cvt 1440 900 60 6 0 0 0
```
- 上面这段里各个参数的具体含义（参考链接 3, 4）：
  - overscan_xxx：用于调整显示器里画面的上下左右（建议不要用这个调，可以直接调显示器）
  - framebuffer_xxx：分辨率（实验室 VGA 显示器的分辨率是 1440*900）
  - **hdmi_force_hotplug：这个参数一定要设置**，它告诉树莓派即便没有检测到 HDMI 显示器也要使用 HDMI 模式（不设置的话会导致一旦 HDMI 口没有检测到负载，树莓派就不会加载桌面环境）
  - hdmi_group：指定HDMI的输出类型，通常值为1或2，分别代表CEA（电视规格分辨率）和DMT（计算机显示器分辨率）
  - hdmi_mode：指定输出分辨率，不同的 mode 对应不同的分辨率。DMT 的 mode 只有 86 个，这里设置成 87 是为了自定义分辨率。
  - hdmi_cvt: 自定义的参数（分辨率 + 刷新频率？）

### 远程连接树莓派
- 完成 *config.txt* 的修改后，重启树莓派。
- 将自己电脑连上路由器的 wifi，然后用 remmina 连接上前面为树莓派设置的静态 IP（使用 VNC 协议）。
- 将 VGA 显示器用 VGA 转 HDMI 的线连到自己电脑，然后把 VNC 窗口拉到 VGA 显示器屏幕上。这样就相当于了树莓派一块屏幕，自己电脑一块屏幕，且共用同一套键鼠。


## 小结
- 总的来说，有 HDMI 显示器 + 键鼠 + 路由器的豪华配件，配置起来还是方便多了。
- 树莓派对 VGA 显示器的支持很差，因此如果要自己配屏幕的话还是要配个 HDMI 接口的显示器。
- 对 Ubuntu Server 20.04 系统的理解
  - 在把系统写入到 SD 卡中之后，发现此时电脑会把 SD 卡读成两个磁盘，一个叫 system-boot，一个叫 writable。
  - 而 HDMI 的配置文件 *config.txt* 就是在 system-boot 里面，如果要在树莓派的 Ubuntu 系统里看的话，这个文件的路径是 `/boot/firmware/config.txt`。
  - 由以上两条可以看出，Ubuntu 为树莓派（ARM 架构）定制的镜像其实是由两部分组成的：一部分是需要适配树莓派架构（ARM）的 boot 文件系统，即 SD 卡里的 system-boot 磁盘或 Ubuntu 下的 `/boot/firmware` 文件夹；还有一部分则是 Ubuntu 本身的文件系统，即 SD 卡里的 writable 磁盘或 Ubuntu 下除 `/boot/firmware` 以外的文件系统。


## 参考链接
1. [How to install Ubuntu Desktop on Raspberry Pi 4](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview)
2. [Install Ubuntu Desktop 20.04 LTS on Raspberry Pi 4](https://linuxhint.com/install-ubuntu-desktop-20-04-lts-on-raspberry-pi-4/)
3. [树莓派 4B 设置 HDMI 分辨率设置](https://www.cnblogs.com/zhangzhicheng1996/p/13499600.html)
4. [将树莓派连接到显示器](https://blog.csdn.net/u012952807/article/details/70183420)