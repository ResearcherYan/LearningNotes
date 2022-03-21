## 总览：显卡驱动, cuda, cudatoolkit ( conda 版本和 Nvidia 官方版本), nvcc, cudnn 是什么？有什么关系？

- cuda 是一个服务于 Nvidia GPU 的通用并行计算架构
- Nvidia 官方版本的 cudatoolkit 包含了
	- cuda driver（也就是显卡驱动）
	- cuda 编译器
	- 库、代码实例等等
- conda 版本的 cudatoolkit 主要包含一些动态链接库，没有 cuda driver。conda 的 cudatoolkit 只要与系统安装的显卡驱动是兼容的，就可以工作。
- nvcc 是 CUDA 的编译器，类似于 c 语言的 gcc。
- cudnn 是 Nvidia 的一个为深度学习提供 GPU 加速的软件库
- 参考链接
	- https://blog.csdn.net/qq_41094058/article/details/116207333
	- https://zhuanlan.zhihu.com/p/91334380

## pytorch, cudatoolkit, 显卡驱动, cudnn 的版本对应关系

- pytorch 与 cudatoolkit：https://pytorch.org/get-started/previous-versions/
- 显卡驱动与 cudatoolkit（显卡驱动可以用最新的，对 cudatoolkit 向前兼容）
	- cudatoolkit 所要求的最低显卡驱动版本：https://docs.nvidia.com/deploy/cuda-compatibility/index.html#minor-version-compatibility
	- cudatoolkit 自带的显卡驱动版本 (Table 3)：https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html#cuda-major-component-versions
- cudatoolkit与cudnn：https://developer.nvidia.com/rdp/cudnn-archive#a-collapse742-10

## 显卡驱动与 cuda 安装 & 卸载

- 安装
	- 方法1：直接安装 cudatoolkit，Nvidia 官方的 cudatoolkit 包含了显卡驱动。（推荐）
		1. https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local
		2. https://docs.nvidia.com/cuda/cuda-quick-start-guide/index.html#ubuntu-x86_64
	- 方法2：先安装显卡驱动，再单独安装 cudatoolkit（有点多此一举）
	- 方法3：通过 ubuntu 软件源安装显卡驱动（据说包含了 cuda ）：`sudo ubuntu-drivers autoinstall`或在“软件和更新”里面选择驱动（不建议）。这种做法可能会出现“挂起再唤醒会死机的现象”（反正至少升级到 20.04.3 LTS 之后就出现这种情况了）。
- 卸载
	- deb installation：`sudo apt-get --purge remove cuda`（现在装的 cudatoolkit 11.6.1 就是通过 deb 包安装的）
	- runfile installation
		- 卸载通过 runfile 安装的 cudatoolkit：`sudo /usr/local/cuda-X.Y/bin/cuda-uninstaller`
		- 卸载通过 runfile 安装的 driver：`sudo /usr/bin/nvidia-uninstall`