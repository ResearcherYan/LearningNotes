## Eigen (ch3)
- 安装：`sudo apt install libeigen3-dev`
- 头文件位置：/usr/include/eigen3

## Pangolin (ch3)
- 安装
	1. 主要参考这个[博客](https://blog.csdn.net/jlm7689235/article/details/122287743)。
		- 下载：`git clone https://github.com/stevenlovegrove/Pangolin.git`
		- 安装依赖：`cd Pangolin && ./script/install_prerequisites.sh recommended`
		- 退回到v0.6以避免一些错误：`git checkout v0.6`
		- configure, build, install
        ```
        mkdir build && cd build
        cmake ..
        cmake --build .（可能直接用make也行，但似乎官网和博客用的都是这个命令）
        sudo make install
        ```
  2. [官方github](https://github.com/stevenlovegrove/Pangolin/tree/1ec721d59ff6b799b9c24b8817f3b7ad2c929b83)。看看就行参考意义不大。
- 头文件位置：/usr/local/include/pangolin

## Sophus (ch4)
- 安装
	1. `git clone https://github.com/strasdat/Sophus.git`
	2. cmake编译安装
    ```
    mkdir build && cd build
    cmake ..
    make -j8
    sudo make install
    ```
- 头文件位置：/usr/local/include/sophus
- Trouble shooting
	- fatal error:fmt/core.h: No such file or directory：在CMakeLists.txt里添加`target_link_libraries([你定义的可执行文件名，如jointMap] Sophus::Sophus)`

## OpenCV (ch5)
### 安装 & 卸载
- 安装（提前安装了cudatoolkit 11.6）
	- 3.4版本（带CUDA版本的configure失败，但如果在cmake-gui里把WITH_CUDA取消勾选，再次configure，应该就能configure成功。）
		1. [博客](https://blog.csdn.net/echoamor/article/details/83022352)。建议主要参考这个，但安装之前一定要把全文读完，因为作者在前面写的是试错部分，看完才知道正确安装方法。另外注意make的时候用8个线程，`make -j8`。
		2. [官方](https://docs.opencv.org/3.4.3/d7/d9f/tutorial_linux_install.html)。建议看看就行，cmake-gui的操作讲的不是很详细，而且没有讲如何安装opencv_contrib，没有make install，也没有配置环境变量。
		3. CUDA编译问题（只是记录一下，对于11.6版本的cuda并没有用）：[知乎](https://zhuanlan.zhihu.com/p/76737748)，[stackoverflow](https://stackoverflow.com/questions/46584000/cmake-error-variables-are-set-to-notfound)
	- 4.5版本
		1. [博客](https://blog.csdn.net/echoamor/article/details/83022352)
			- 大部分操作跟3.4的一样，只不过在使用cmake-gui的时候会默认不勾选WITH_CUDA，需要手动勾选之后再次configure。
			- 还要注意博客里没有写make install，make完之后记得通过make install安装。
			- 博客最后还提到了配置环境变量，暂时没有做这一步，似乎没有影响。
		2. [官方](https://docs.opencv.org/4.5.5/d7/d9f/tutorial_linux_install.html)。这个可以看看，相比于3.4版本，官方的installation tutorial新增了make install，并说清楚了每个部分的文件会被copy到/usr/local的哪些子文件夹下面。
		3. [CMake时一些重要configuration options的描述](https://docs.opencv.org/4.5.5/db/d05/tutorial_config_reference.html#tutorial_config_reference_general_contrib)。可以用于备查。
		4. 问题记录
			- 带cuda的make耗时需要2-3h，尤其是build前面cuda的相关库，非常慢。
			- 在“[ 64%] Built target opencv_python3”这一步的时候，报错“make: *** [Makefile:163：all] 错误 2”。原因应该是在安装opencv的前置python依赖包时出错。安装python-dev的时候报错“下列软件包有未满足的依赖关系：python-dev : 依赖: python (= 2.7.11-1)”。再尝试sudo apt install python2.7，提示“python2.7 已经是最新版 (2.7.18-1~20.04.1)”。再通过sudo apt install aptitude之后，使用sudo aptitude install python-dev，aptitude给出的依赖关系处理方案要求删除一些软件包（包括deepin-wine, python2.7等），由于担心会破坏一些其他依赖，就没有选择删除这些软件包。尝试在cmake-gui里面搜索python，把python相关的全部取消勾选，再重新configure。
			- 有点奇怪的一点是在sudo make install的时候还要再build一遍（明明make的时候已经完成build了），build结束之后才会install到/usr/local各个文件夹里面。
- 卸载
	- 3.4版本
		- 找到opencv build文件夹，在终端打开，`sudo make uninstall`
		- 删除/usr/local/share下的opencv和opencv2文件夹：`sudo rm -rf opencv*`
		- 删除/usr/local/lib下的opencv的库文件：`sudo rm -rf *opencv*`
		- 删除opencv的源代码文件，放在了/home/yan/software下面。
		- 以上应该就把opencv的CMake版本卸载完全了，可以再在/usr/local文件夹下搜索一下，确认一下还有没有opencv相关的残余文件。
	- 4.5版本（没有实操）
		- 找到opencv build文件夹，在终端打开，`sudo make uninstall`
		- 卸载完之后check一下[官方的installation tutorial](https://docs.opencv.org/4.5.5/d7/d9f/tutorial_linux_install.html)最后面说的那些安装位置，看还有没有残留。

### CMake编译opencv程序
- Trouble shooting
	- cmake error: Could NOT find CUDA: Found unsuitable version "10.1", but required is exact version "11.6" (found /usr)：如果在CMakeLists.txt中仅通过find_package(OpenCV REQUIRED)来寻找opencv的路径，cmake在寻找对应的cuda版本的时候容易找到在base环境下conda安装的cudatoolkit 10.2（但cmake居然报的是10.1，有点离谱），而opencv 4.5.5当初在build的时候用的是Nvidia官方的cudatoolkit 11.6，所以这里就会报cuda版本错误。解决方法有三种：
		1. find_package的时候指定opencv版本。
		2. 先指定CUDA路径（否则cmake容易找到anaconda安装的cudatoolkit），再通过find_package找opencv库。
		3. 直接指定Opencv的路径，由于编译opencv的时候默认用的是cuda 11.6，所以也就会自然找到正确的cuda位置。
  ```
  # 方法1
  find_package(OpenCV 4.5.5 REQUIRED)
  # 方法2
  set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.6)
  find_package(OpenCV REQUIRED)
  # 方法3
  set(OpenCV_INCLUDE_DIRS /usr/local/include/opencv4)
  ```

## Boost (ch5)
在最后讲RGB-D的程序jointMap.cpp里面用到了boost库，但书上没有提前说要安装。
- 安装：`sudo apt install libboost-all-dev`

## Ceres (ch6)
- 安装：[官方教程](http://www.ceres-solver.org/installation.html#linux)。安装的时候选择github上面的[1.14.0版本](https://github.com/ceres-solver/ceres-solver/releases/tag/1.14.0)，不要按官方的装最新的2.x版本。
- 头文件位置：/usr/local/include/ceres
- 库文件位置：/usr/local/lib/
- Trouble shooting
	- 在CMakeLists.txt里面直接find_package会报错，要手动set ceres的路径：`set(CERES_INCLUDE_DIRS /usr/local/include/ceres)`
	- 如果按照官方教程安装最新的2.x版本，会导致编译ceresCurveFitting.cpp的时候报错，降到1.14.0版本之后就好了

## g2o (ch6)
- 安装：[官方教程](https://github.com/RainerKuemmerle/g2o/tree/9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a#requirements)。安装的时候不要安装最新版本，可以git clone下来之后git checkout 20200410_git换成旧版本，再进行cmake。
- 头文件位置：/usr/local/g2o
- 库文件位置：/usr/local/lib/
- Trouble shooting
	- 之前装了最新版本发现编译出错，然后git checkout 20200410_git换成旧版本，再切换到新版本之后发现没法sudo make uninstall了，可能是因为之前build没有用git保存更改到本地，直接切换分支之后master branch下的build files全没了。就只能通过把/usr/local/include, /usr/local/lib, /usr/local/bin里面关于g2o的文件全删了（g2o应该只安装到了这些位置），再重新cmake。