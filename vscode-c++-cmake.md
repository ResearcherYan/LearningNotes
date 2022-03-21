## Prerequisites
1. 安装CMake。（编译器gcc和调试器gdb一般linux会自带）
2. 在VSCode里面安装C/C++（主要作用为intelliSense, debug）, CMake（主要用于CMakeLists.txt高亮）和CMake Tools（将CMake的各种操作移植到VSCode里）扩展。
---

## C++
- 说明：这部分主要用于理解一下VSCode编译运行C++程序的逻辑，实际编译调试C++工程的时候还是用CMake多一些。单个cpp也可以用这个方法。
- 参考链接：https://code.visualstudio.com/docs/cpp/config-linux
- 示例程序：/home/yan/Learning/slam/C++VSCode

### 三个核心json文件
1. tasks.json：编译器和build的相关设置。
2. launch.json：调试器的相关设置。
3. c_cpp_properties.json：编译器路径与IntelliSense的相关设置。

### Build & Run
1. 选择编译器，编辑task.json：Terminal > Configure Default Build Task，选择g++最新版的编译器生成task.json。
2. Build the task：Ctrl+Shift+B，或Terminal > Run Build Task。
3. 运行可执行程序helloWorld：`./helloworld`。

### Debug
1. 通过写launch.json文件配置调试器(gdb)的相应设置：Run > Add Configuration，选择C++ (GDB/LLDB)，会生成一个launch.json。点击右下角的Add Configuration，选择C/C++：(gdb)启动，将program的值改为`${fileDirname}/${fileBasenameNoExtension}`。（注意在debug之前一定要先build得到executable才可以）
2. 启动调试：F5或Run > Start Debugging。
3. (optional)将变量添加到watch里：可以在VARIABLES里面选择变量，右键选择Add to Watch，也可以直接在WATCH里面输入变量名添加。调试时可以将一些重要变量放到watch里单独观察。

### 配置IntelliSense
1. 创建c_cpp_properties.json：打开Command Palatte (Ctrl+Shift+P)，输入C/C++: Edit Configurations (UI)，会打开IntelliSense Configuration的UI界面（同时创建c_cpp_properties.json，UI和json文件是同步的）。
2. 修改编译器路径，用g++的最新版本，IntelliSense模式改为linux-gcc-x64。
3. 如果程序里用到了非工作目录下或者非标准库的头文件，还需要在UI界面中的包含路径下填上该头文件的路径，或者在c_cpp_properties.json的includePath里添加路径。
---

## CMake
- 参考链接
	- https://code.visualstudio.com/docs/cpp/cmake-linux
	- https://github.com/microsoft/vscode-cmake-tools/blob/main/docs/debug-launch.md
- 示例程序：/home/yan/Learning/slam/cmakeVScode

### Build & Run
1. 创建一个空的CMake project：打开Command Palatte，输入CMake: Quick Start，输入project的名字，project type选择Executable。这样会创建一个CMake project，包含了main.cpp, CMakeLists.txt（用于告诉CMake如何build你的project）以及一个空的build文件夹。
2. 选择编译器：打开Command Palatte， 输入CMake: Select a Kit，选择高版本的gcc。
3. 选择build type：打开Command Palatte， 输入CMake: Select Variant，默认选择Debug模式就可以。build type也可以在CMakeLists.txt里面声明（优先级更高）。
4. (Optional) Configure：打开Command Palatte， 输入CMake: Configure，基于上述选择的kit（编译器）和variant（build type），configure操作会在build文件夹中生成一些build文件。（不用configure直接build也行，但是不configure的话会导致如果要单独build某一个cpp则需要手动输入它的名字）
5. Build：打开Command Palatte， 输入CMake: Build，或直接在下方的status bar指定build target然后点击build button。这样生成的可执行程序就在build文件夹下。
6. 运行build文件夹里的可执行程序。终端输入`./build/[可执行程序名]`，或直接点击下方status bar里的run button。

### Debug
1. 写launch.json、启动调试、添加变量到watch中：这几步与上面C++的一样，不同的在于此时launch.json里面program的值应改为`${command:cmake.launchTargetPath}`。
2. (optional) 使用Quick-debugging
	- CMake project除了可以通过配置launch.json来debug以外，还有一种Quick-debugging模式，通过Ctrl+F5或点击下方status bar里面的debug button或者在Command Palatte中输入CMake: Debug Target启动。Quick-debugging模式会忽略launch.json里的内容，无法给debugger传递参数，用于快速调试。
	- 这个模式有一个麻烦的地方在于每次启动debug之前都要重新build，这个可以通过将~/.config/Code/User/settings.json（在Command Palatte中输入Preferences: Configure Language Specific Settings，然后随便选一个语言就能进入这个json）加入一项"cmake.buildBeforeRun": false，这样做之后，在进行Quick-debugging之前只会在没有executable的时候build一下，如果已经build了就不会再重复build。

### 配置IntelliSense
- 编译器路径换成gcc的最新版本要好一点，其余与上面的C++一样。

### 设置CMake settings
如果希望vscode在configure自己的cmake project的时候设置一些configuration options的话，可以在.vscode下创建settings.json。
- cmake.configureSettings：给CMake传入一些键值对，相当于cmake命令行的`-DVAR_NAME=ON`这种的。示例程序：/home/yan/Learning/slam/realsense/test。
- cmake.buildDirectory：设置在哪里build，vscode默认是`${workspaceFolder}/build`。
---

## Trouble Shooting

- 如果debug CMake project的时候无法在断点处停下来，总是一次性执行完整个程序，可能是因为下方status bar里面的build variant选成了Release，或者在CMakeLists.txt里面设置了build type（优先级高于build variant）为Release。