> CONTENT
- [Prerequisites](#prerequisites)
- [C++](#c)
	- [三个核心 json 文件](#三个核心-json-文件)
	- [Build & Run](#build--run)
	- [Debug](#debug)
	- [配置 IntelliSense](#配置-intellisense)
- [CMake](#cmake)
	- [Build & Run](#build--run-1)
	- [Debug](#debug-1)
	- [配置 IntelliSense](#配置-intellisense-1)
	- [设置 CMake settings](#设置-cmake-settings)
- [Trouble Shooting](#trouble-shooting)
## Prerequisites
1. 安装 CMake。（编译器 gcc 和调试器 gdb 一般 linux会自带）
2. 在 VSCode 里面安装 C/C++（主要作用为 intelliSense, debug）, CMake（主要用于 CMakeLists.txt 高亮）和 CMake Tools（将 CMake 的各种操作移植到 VSCode 里）扩展。

## C++
- 说明：这部分主要用于理解一下 VSCode 编译运行 C++ 程序的逻辑，实际编译调试 C++ 工程的时候还是用 CMake 多一些。单个 cpp 可以用这个方法。
- 参考链接：https://code.visualstudio.com/docs/cpp/config-linux
- 示例程序：/home/yan/Learning/slam/C++VSCode

### 三个核心 json 文件
1. tasks.json：编译器和 build 的相关设置。
2. launch.json：调试器的相关设置。
3. c_cpp_properties.json：编译器路径与 IntelliSense 的相关设置。

### Build & Run
1. 选择编译器，编辑 task.json：Terminal > Configure Default Build Task，选择 g++ 最新版的编译器生成 task.json。
2. Build the task：Ctrl+Shift+B，或 Terminal > Run Build Task。
3. 运行可执行程序 helloWorld：`./helloworld`。

### Debug
1. 通过写 launch.json 文件配置调试器 (gdb) 的相应设置：Run > Add Configuration，选择 C++ (GDB/LLDB)，会生成一个 launch.json。点击右下角的 Add Configuration，选择 C/C++：(gdb) 启动，将 program 的值改为 `${fileDirname}/${fileBasenameNoExtension}`。（注意在 debug 之前一定要先 build 得到 executable 才可以）
2. 启动调试：F5 或 Run > Start Debugging。
3. (optional) 将变量添加到 watch 里：可以在 VARIABLES 里面选择变量，右键选择 Add to Watch，也可以直接在 WATCH 里面输入变量名添加。调试时可以将一些重要变量放到 watch 里单独观察。

### 配置 IntelliSense
1. 创建 c_cpp_properties.json：打开 Command Palatte (Ctrl+Shift+P)，输入 C/C++: Edit Configurations (UI)，会打开 IntelliSense Configuration 的 UI 界面（同时创建 c_cpp_properties.json，UI 和 json 文件是同步的）。
2. 修改编译器路径，用 g++ 的最新版本，IntelliSense 模式改为 linux-gcc-x64。
3. 如果程序里用到了非工作目录下或者非标准库的头文件，还需要在 UI 界面中的包含路径下填上该头文件的路径，或者在 c_cpp_properties.json 的 includePath 里添加路径。

## CMake
- 参考链接
	- https://code.visualstudio.com/docs/cpp/cmake-linux
	- https://github.com/microsoft/vscode-cmake-tools/blob/main/docs/debug-launch.md
- 示例程序：/home/yan/Learning/slam/cmakeVScode

### Build & Run
1. 创建一个空的 CMake project：打开 Command Palatte，输入 CMake: Quick Start，输入 project 的名字，project type 选择 Executable。这样会创建一个 CMake project，包含了 main.cpp, CMakeLists.txt（用于告诉 CMake 如何 build 你的 project）以及一个空的 build 文件夹。
2. 选择编译器：打开 Command Palatte， 输入 CMake: Select a Kit，选择高版本的 gcc。
3. 选择 build type：打开 Command Palatte， 输入 CMake: Select Variant，默认选择 Debug 模式就可以。build type 也可以在 CMakeLists.txt 里面声明（优先级更高）。
4. (Optional) Configure：打开 Command Palatte， 输入 CMake: Configure，基于上述选择的 kit（编译器）和 variant（build type），configure 操作会在 build 文件夹中生成一些 build 文件。（不用 configure 直接 build 也行，但是不 configure 的话会导致如果要单独 build 某一个 cpp 则需要手动输入它的名字）
5. Build：打开 Command Palatte， 输入 CMake: Build，或直接在下方的 status bar 指定 build target 然后点击 build button。这样生成的可执行程序就在 build 文件夹下。
6. 运行 build 文件夹里的可执行程序。终端输入 `./build/[可执行程序名]`，或直接点击下方 status bar 里的 run button。

### Debug
1. 写 launch.json、启动调试、添加变量到 watch 中：这几步与上面C++的一样，不同的在于此时 launch.json 里面 program 的值应改为 `${command:cmake.launchTargetPath}`。
2. (optional) 使用 Quick-debugging
	- CMake project 除了可以通过配置 launch.json 来 debug 以外，还有一种 Quick-debugging 模式，通过 Ctrl+F5 或点击下方 status bar 里面的 debug button 或者在 Command Palatte 中输入 CMake: Debug Target 启动。Quick-debugging 模式会忽略 launch.json 里的内容，无法给 debugger 传递参数，用于快速调试。
	- 这个模式有一个麻烦的地方在于每次启动 debug 之前都要重新 build，这个可以通过将 ~/.config/Code/User/settings.json（在 Command Palatte 中输入 Preferences: Configure Language Specific Settings，然后随便选一个语言就能进入这个 json）加入一项 `"cmake.buildBeforeRun": false`，这样做之后，在进行 Quick-debugging 之前只会在没有 executable 的时候 build 一下，如果已经 build 了就不会再重复 build。

### 配置 IntelliSense
- 编译器路径换成 gcc 的最新版本要好一点，其余与上面的 C++ 一样。

### 设置 CMake settings
如果希望 vscode 在 configure 自己的 cmake project 的时候设置一些 configuration options 的话，可以在 .vscode 下创建 settings.json。
- cmake.configureSettings：给 CMake 传入一些键值对，相当于 cmake 命令行的 `-DVAR_NAME=ON` 这种的。示例程序：/home/yan/Learning/slam/realsense/test。
- cmake.buildDirectory：设置在哪里 build，vscode 默认是 `${workspaceFolder}/build`。

## Trouble Shooting
- 如果 debug CMake project 的时候无法在断点处停下来，总是一次性执行完整个程序，可能是因为下方 status bar 里面的 build variant 选成了 Release，或者在 CMakeLists.txt 里面设置了 build type（优先级高于 build variant）为 Release。