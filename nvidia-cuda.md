## 总览：显卡驱动, cuda, cudatoolkit (conda版本和Nvidia官方版本), nvcc, cudnn是什么？有什么关系？

- cuda是一个服务于Nvidia GPU的通用并行计算架构
- Nvidia官方版本的cudatoolkit包含了
	- cuda driver（也就是显卡驱动）
	- cuda编译器
	- 库、代码实例等等
- conda版本的cudatoolkit主要包含一些动态链接库，没有cuda driver。conda的cudatoolkit只要与系统安装的显卡驱动是兼容的，就可以工作。
- nvcc是CUDA的编译器，类似于c语言的gcc。
- cudnn是Nvidia的一个为深度学习提供GPU加速的软件库
- 参考链接
	- https://blog.csdn.net/qq_41094058/article/details/116207333
	- https://zhuanlan.zhihu.com/p/91334380

## pytorch, cudatoolkit, 显卡驱动, cudnn的版本对应关系

- pytorch与cudatoolkit：https://pytorch.org/get-started/previous-versions/
- 显卡驱动与cudatoolkit（显卡驱动可以用最新的，对cudatoolkit向前兼容）
	- cudatoolkit所要求的最低显卡驱动版本：https://docs.nvidia.com/deploy/cuda-compatibility/index.html#minor-version-compatibility
	- cudatoolkit自带的显卡驱动版本(Table 3)：https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html#cuda-major-component-versions
- cudatoolkit与cudnn：https://developer.nvidia.com/rdp/cudnn-archive#a-collapse742-10

## 显卡驱动与cuda安装 & 卸载

- 安装
	- 方法1：直接安装cudatoolkit，Nividia官方的cudatoolkit包含了显卡驱动。（推荐）
		1. https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local
		2. https://docs.nvidia.com/cuda/cuda-quick-start-guide/index.html#ubuntu-x86_64
	- 方法2：先安装显卡驱动，再单独安装cudatoolkit（有点多此一举）
	- 方法3：通过ubuntu软件源安装显卡驱动（据说包含了cuda）：`sudo ubuntu-drivers autoinstall`或在“软件和更新”里面选择驱动（不建议）。这种做法可能会出现“挂起再唤醒会死机的现象”（反正至少升级到20.04.3 LTS之后就出现这种情况了）。
- 卸载
	- deb installation：`sudo apt-get --purge remove cuda`（现在装的cudatoolkit 11.6.1就是通过deb包安装的）
	- runfile installation
		- 卸载通过runfile安装的cudatoolkit：`sudo /usr/local/cuda-X.Y/bin/cuda-uninstaller`
		- 卸载通过runfile安装的driver：`sudo /usr/bin/nvidia-uninstall`