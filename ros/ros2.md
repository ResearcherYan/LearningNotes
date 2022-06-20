> 本文用于记录 ROS2 的安装和学习


- [Installation](#installation)
- [ROS vs. ROS2](#ros-vs-ros2)
- [Beginner Tutorial - Concept](#beginner-tutorial---concept)
  - [Base](#base)
  - [Nodes](#nodes)
  - [Topics](#topics)
  - [Services](#services)
    - [Services vs. Topics](#services-vs-topics)
  - [Parameters](#parameters)
  - [Actions](#actions)
    - [Background](#background)
    - [Usage](#usage)
  - [rqt_console](#rqt_console)
  - [Launch](#launch)
  - [Bag](#bag)
  - [Workspace](#workspace)
    - [Background](#background-1)
    - [Usage](#usage-1)
  - [Package](#package)
    - [Background](#background-2)
    - [Usage](#usage-2)
- [Beginner Tutorial - Practice](#beginner-tutorial---practice)
  - [Writing a simple publisher and subscriber (C++)](#writing-a-simple-publisher-and-subscriber-c)
  - [Writing a simple service and client (C++)](#writing-a-simple-service-and-client-c)
  - [Creating custom ROS 2 msg and srv files](#creating-custom-ros-2-msg-and-srv-files)
  - [Expanding on ROS 2 interfaces](#expanding-on-ros-2-interfaces)
  - [Using parameters in a class (C++)](#using-parameters-in-a-class-c)
  - [Getting started with ros2doctor](#getting-started-with-ros2doctor)
  - [Creating and Using Plugins (C++)](#creating-and-using-plugins-c)
    - [Background](#background-3)
    - [Usage](#usage-3)
- [Other Tutorials](#other-tutorials)
  - [tf2 Tutorials](#tf2-tutorials)
    - [Introduction to tf2](#introduction-to-tf2)
      - [tf2 tools](#tf2-tools)
    - [Writing a tf2 static broadcaster (C++)](#writing-a-tf2-static-broadcaster-c)
    - [Writing a tf2 broadcaster (C++)](#writing-a-tf2-broadcaster-c)
    - [Writing a tf2 listener (C++)](#writing-a-tf2-listener-c)
    - [Adding a frame (C++)](#adding-a-frame-c)
    - [Learning about tf2 and time (C++)](#learning-about-tf2-and-time-c)
    - [Time travel with tf2 (C++)](#time-travel-with-tf2-c)


# Installation
新电脑只有 Ubuntu 22 能够识别无线网卡，而 Ubuntu 22 只支持 ROS2，因此被迫使用 ROS 2 Humble Hawksbill。
参考[官方安装链接](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)，安装桌面版，注意在添加软件源的时候换成中科大的就行：<br>
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros2/ubuntu/ $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

# ROS vs. ROS2
- ROS2 没有 master 节点了。
- ROS2 没有 filesystem tools 了，没法使用 rospack, roscd, rosls, rosed 等文件管理功能。
- ROS2 不再使用 catkin 为默认的 build tool，使用的是 colcon。

# Beginner Tutorial - Concept
[官方教程](https://docs.ros.org/en/humble/Tutorials.html#) Beginner Level: From [Configuring your ROS 2 environment](https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html) to [Creating your first ROS 2 package](https://docs.ros.org/en/humble/Tutorials/Creating-Your-First-ROS2-Package.html)

## Base
- 把启动 ROS2 的 setup file 写到 bashrc 里，以免每次打开新的终端都要 source 一下：`echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
- 为自己 ROS2 的 group 添加 **Domain ID**：`echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc`（这里设置的 Domain ID 是 1）
- 安装 rqt：`sudo apt install ~nros-humble-rqt*`
- 安装 colcon: `sudo apt install python3-colcon-common-extensions`
- 安装 git: `sudo apt install git-all`
- 安装并初始化 rosdep
  ```bash
  sudo apt-get install python3-rosdep
  sudo rosdep init # 可能会出现 read operation timed out 的错误，需要多试几次
  rosdep update
  ```
  如果在执行 `rosdep update` 的时候出现 read operation time out 的话，建议按照网上修改等待时间然后多次尝试碰运气的方法，因为原因在于用的源是 github，有时候就是没办法访问。<br>
  这里参考[博客](https://blog.csdn.net/qq_28901541/article/details/116134317?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-116134317-blog-87903654.pc_relevant_downloadblacklistv1&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-116134317-blog-87903654.pc_relevant_downloadblacklistv1&utm_relevant_index=2#commentBox)里方法一的做法，不用碰运气且原理清晰：
  1. 打开一个新终端，保证当前目录为主目录：`git clone https://github.com/ros/rosdistro`
  2. 修改 `/etc/ros/rosdep/sources.list.d/20-default.list` 文件（这个文件在执行 `sudo rosdep init` 之后就会生成），修改内容如下
  ```
  yaml file:///home/yan/rosdistro/rosdep/osx-homebrew.yaml osx

	# generic
	yaml file:///home/yan/rosdistro/rosdep/base.yaml
	yaml file:///home/yan/rosdistro/rosdep/python.yaml
	yaml file:///home/yan/rosdistro/rosdep/ruby.yaml
	gbpdistro file:///home/yan/rosdistro/releases/fuerte.yaml fuerte
  ```
  3. 修改 `/usr/lib/python3/dist-packages/rosdep2/sources_list.py` 文件，修改内容如下
  ```
  DEFAULT_SOURCES_LIST_URL = 'file:///etc/ros/rosdep/sources.list.d/20-default.list'
  ```
  4. 修改 `/usr/lib/python3/dist-packages/rosdep2/rep3.py` 文件，修改内容如下
  ```
  REP3_TARGETS_URL = 'file:///home/yan/rosdistro/releases/targets.yaml'
  ```
  5. 修改 `/usr/lib/python3/dist-packages/rosdistro/__init__.py` 文件，修改内容如下
  ```
  DEFAULT_INDEX_URL = 'file:///home/yan/rosdistro/index-v4.yaml'
  ```
  6. 完成以上修改后，再执行 `rosdep update`

## Nodes
- **ros2 run**: Launch an executable from a package.（启动一个节点）
  - `ros2 run <package_name> <executable_name>`
- **ros2 node list**: Show the names of all *running* nodes.
  - `ros2 node list`
- **Remapping**: Reassign default node properties, like node name, topic names, service names, etc., to custom values.<br>
  下面这条指令会重新开一个乌龟节点，但节点名不再是默认的 /turtlesim，而是被修改为了 /my_turtle。
  - `ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle`
- **ros2 node info**: Show a list of subscribers, publishers, services, and actions (the ROS graph connections) that interact with that node.
  - `ros2 node info <node_name>`

## Topics
- **ros2 topic list**: Show a list of all the topics *currently active* in the system.
  - `ros2 topic list`
  - `ros2 topic list -t`: with topic type appended（topic type 用于说明这个 topic 负责传递的是什么类型的 message）
- **ros2 topic echo**: Show the data being published on a topic.
  - `ros2 topic echo <topic_name>`
- **ros2 topic info**: Show the topic type, publisher count, subsciption count of a topic.
  - `ros2 topic info <topic_name>`
- **ros2 interface show**: Show the structure of data the message expects.
  - `ros2 interface show <msg type>`<br>
  这里的 msg_type 就是前面说的 topic type，即 `ros2 topic list -t 中括号里面的内容`，或者用 `ros2 topic info <topic_name>` 也能查看到 topic type。
- **ros2 topic pub**: Publish data onto a topic.
  - `ros2 topic pub <topic_name> <msg_type> '<args>'`：按照默认频率循环发布消息
  - `ros2 topic pub --once <topic_name> <msg_type> '<args>'`：发布一个消息然后退出
  - `ros2 topic pub --rate <frequency> <topic_name> <msg_type> '<args>'`：以规定的频率发布消息
- `ros2 topic hz`: View the rate at which data is published.
  - `ros2 topic hz <topic_name>`

## Services
- **ros2 service list**: Show a list of all the services currently active in the system.
  - `ros2 service list`
  - `ros2 service list -t`: with the service type appended
- **ros2 service type**: Show how the request and response data of a service is structured.
  - `ros2 service type <service_name>`
- **ros2 service find**: Find all the services of a specific type.
  - `ros2 service find <type_name>`
- **ros2 interface show**: 作用同 [Topics](#topics) 里的 *ros2 interface show*，就是看这个 service 传递的消息是什么格式。只不过这里后面接的不再是 topic type，而是 service type 了。
  - `ros2 service find <service_type>`
- **ros2 service call**: Call a service.
  - `ros2 service call <service_name> <service_type> <arguments>`: arguments 是可选项，不一定所有的 service 都有 arguments

### Services vs. Topics
Service 和 topic 的区别在于
- Service 是一个 request/responde 模式，client node 需要给 server node 发一个 request，然后由 server node 给出一个 response
- Topic 是一个 publish/subscribe 模式，publisher 发布一个 topic，然后由 subscriber 订阅这个 topic

## Parameters
A parameter is a configuration value of a node.
- **ros2 param list**: Show the parameters belonging the running nodes.
  - `ros2 param list`
- **ros2 param get**: Show the type and current value of a parameter.
  - `ros2 param get <node_name> <parameter_name>`
- **ros2 param set**: Change a parameter’s value at runtime.
  - `ros2 param set <node_name> <parameter_name> <value>`
- **ros2 param dump**: Show a node’s all current parameter values.
  - `ros2 param dump <node_name>`
  - `ros2 param dump /turtlesim > turtlesim.yaml`: 这条命令是将 /turtlesim 节点下的参数配置写到 yaml 文件里
- **ros2 param load**: Load parameters from a file to a currently running node.
  - `ros2 param load <node_name> <parameter_file>`
- **Load parameter file on node startup**
  - `ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>`

## Actions
### Background
Action 是用于长期运行任务的一种 communication type。<br>
Action 包括三部分: a goal, feedback, and a result。<br>
Action 用的是 client-server 模型，但跟 service 的差别在于 action 在进行过程中**可以收到反馈**并且是**可以被中断**的，如果是由 client 侧提出要停止一个 goal，称为 cancel a goal，如果是由 server 侧决定要停止一个 goal，称为 abort a goal。<br>
Action 的具体工作流程为
- action client 借助 goal service 给 action server 发一个 request，然后 action server 给出相应的 response。
- action client 借助 result service 再给 action server 发一个 request，在完成 goal 之前，action server 会一直通过 feedback topic 给 action client 发送 feedback，任务完成后会给出一个 response。
<img src='../img/ros2_1.gif'><br>

### Usage
- **ros2 node info**: `ros2 node info <node_name>` 会显示 node 下面有哪些 action server 和 action client
- **ros2 action list**: Show all the actions in the ROS graph.
  - `ros2 action list`
  - `ros2 action list -t`: with the action type appended
- **ros2 action info**: Further introspect an action.
  - `ros2 action info <action_name>`
- **ros2 interface show**: Show the structure of an action type. 一个 action structure 由三部分组成，三部分由`---`隔开，第一部分是 goal 的数据格式，第二部分是 result，第三部分是 feedback。
  - `ros2 interface show <action_type>`
- **ros2 action send_goal**: Send an action goal from the command line.
  - `ros2 action send_goal <action_name> <action_type> <values>`: 其中 `value` 需要是 YAML 格式
  - `ros2 action send_goal <action_name> <action_type> <values> --feedback`: 终端除了会输出 goal 和 result 之外，还会输出 feedback（直到 goal 完成之前）。

## rqt_console
`rqt_console` is a GUI tool used to introspect log messages.
- **Start *rqt_console***: `ros2 run rqt_console rqt_console`
- **Logger levels**: *Fatal, Error, Warn, Info, Debug* 严重等级依次降低，默认的 logger level 是 *Info*，即没有 *Info* 严重的 log 是看不到的。
  - `ros2 run <package_name> <executable_name> --ros-args --log-level WARN`: 修改默认的 logger level 为 Warn

## Launch
Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.
- **Running a launch file**: `ros2 launch <node_name> <launch_file>`. Launch file 可以用 python 写，也可以使用 XML 和 YAML 格式。如何写一个 launch file 参考 [Launch Tutorials](https://docs.ros.org/en/humble/Tutorials/Launch/Launch-Main.html)。

## Bag
- **ros2 bag record**: Record the data published to a topic.
  - `ros2 bag record <topic_name>`
  - `ros2 bag record -o <bag_file_name> <topic1> <topic2>`: Record multiple topics and change the name of the rosbag file.(The default name pattern is `rosbag2_year_month_day-hour_minute_second`)
- **ros2 bag info**: Show details about a bag file.
  - `ros2 bag info <bag_file_name>`
- **ros2 bag play**: Replay a bag file.
  - `ros2 bag play <bag_file_name>`

## Workspace
### Background
A workspace is a directory containing ROS 2 packages.
- **overlay**: A secondary workspace where you can add new packages without interfering with the existing ROS 2 workspace that you’re extending.
- **underlay**: Contain the dependencies of all the packages in your overlay. Packages in your overlay will override packages in the underlay.

### Usage
- **Create a workspace**: `mkdir -p ~/dev_ws/src`
- **Create a package under the workspace**: `cd ~/dev_ws/src && git clone https://github.com/ros/ros_tutorials.git -b humble-devel`
- **Resolve dependencies**: `cd ~/dev_ws && rosdep install -i --from-path src --rosdistro humble -y`
- **Build the workspace with colon**: `colcon build`. Once the build is finished, there will be 4 new folders in your workspace: `build install log src`. The `install` directory is where your workspace’s setup files are, which you can use to source your overlay.
  - `--packages-select` builds the package you want.
  - `--packages-up-to` builds the package you want, plus all its dependencies.
  - `--cmake-clean-cache` remove the CMake cache file `CMakeCache.txt` from the build directory before proceeding with the build. This implicitly **forces a CMake configure step**.
  - `--symlink-install` saves you from having to rebuild every time you tweak python scripts.
  - `--event-handlers console_direct+` shows console output while building (can otherwise be found in the log directory).
- **Source the overlay**: `turtlesim` 本来就存在于 ROS2 的主环境里，这是一个 underlay 版本，上面的操作又在自己的 workspace 里面 build 了一个 overlay 版本的 `turtlesim`。此时，如果新开一个终端，执行 `source ~/dev_ws/install/local_setup.bash`，相当于只会把 overlay 环境的包（即自己 workspace 里面的包）添加到终端环境中，但如果执行的是 `source ~/dev_ws/install/setup.bash`，就会同时把 overlay 版本的包和 ROS2 主环境的包都添加到终端环境里，不过 `setup.bash` 会先 source ROS2 主环境，然后再 source overlay 环境（即自己的 workspace），这样就能保证如果 ROS2 主环境和 overlay 环境有相同的包，终端默认使用的包就是 overlay 环境的包。

## Package
### Background
Package creation in ROS 2 uses **ament** as its build system and **colcon** as its build tool.
- `package.xml` file containing meta information about the package.
- `CMakeLists.txt` file that describes how to build the code within the package.

### Usage
在同一个终端下执行以下命令
- **Create a package**: 基本的格式为 `ros2 pkg create --build-type ament_cmake <package_name>`
  - `cd ~/dev_ws/src && ros2 pkg create --build-type ament_cmake --node-name my_node my_package`: 加上 `--node-name` 会在 my_package 里创建一个简单的 Hello World type executable.
- **Build a package**: `cd .. && colcon build --packages-select my_package`
- **Source the setup file**: `source install/local_setup.bash`
- **Use the package**: `ros2 run my_package my_node`
- **Customize package.xml**: 参考 [ros2 doc](https://docs.ros.org/en/humble/Tutorials/Creating-Your-First-ROS2-Package.html#customize-package-xml) 修改 `package.xml`。主要需要修改的几个地方是：
  - `maintainer`, `descprition`, `license`: 如果要发布自己的包，这几项需要填写。
  - tags names ending with `_depend`: 用于向 colcon 描述这个 package 的依赖。

# Beginner Tutorial - Practice
[官方教程](https://docs.ros.org/en/humble/Tutorials.html#) Beginner Level: From [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) to [Creating and Using Plugins (C++)](https://docs.ros.org/en/humble/Tutorials/Pluginlib.html)

## Writing a simple publisher and subscriber (C++)
在同一个终端下执行以下命令
1. **Create a package**: `cd ~/dev_ws/src && ros2 pkg create --build-type ament_cmake cpp_pubsub`
2. **Write the publisher code**
   - `wget -O cpp_pubsub/src/publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp`
   - 按照 [2.2 Add dependencies](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#add-dependencies) 的要求修改 `package.xml` 文件，添加上 publisher 节点所需的依赖（与后面 subscriber 节点的依赖相同）。
   - 按照 [2.3 CMakeLists.txt](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#cmakelists-txt) 的要求修改 `CMakeLists.txt` 文件。
3. **Write the subscriber node**
   - `wget -O cpp_pubsub/src/subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp`。下载好 subscriber 的代码后，发现跟 publisher 的代码很像，只不过 subscriber 没有 timer 了，因为它只需要对 topic 发出的数据给出 response 就行，不需要 timer。
   - 按照 [3.2 CMakeLists.txt](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#id2) 的要求修改 `CMakeLists.txt` 文件。
4. **Build and run**
   - 在 build 之前先确认安装了所有的依赖（在本例里所需的依赖就是 `rclcpp` 和 `std_msgs`，在安装 ROS2 的时候就已经安装了）: `cd .. && rosdep install -i --from-path src --rosdistro humble -y`
   - Build cpp_pubsub package: `colcon build --packages-select cpp_pubsub`
   - Source the setup file and run the **talker** node: `source install/setup.bash && ros2 run cpp_pubsub talker`
   - Open a new terminal. Source the setup file and run the **listener** node: `source ~/dev_ws/install/setup.bash && ros2 run cpp_pubsub listener`

## Writing a simple service and client (C++)
The structure of the request and response, which are generated during the *service* communication, is determined by a `.srv` file.<br><br>
在同一个终端执行以下命令
1. **Create a package**: `cd ~/dev_ws/src && ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces`。其中 `--dependencies` 会将依赖添加到 `package.xml` 和 `CMakeLists.txt` 中。依赖 `example_interfaces` 包括了本例中需要用到的 *.srv* 文件。
2. **Write the service node**: 按照 [2 Write the service node](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html#write-the-service-node) 写一个 `add_two_ints_server.cpp` 文件。
3. **Write the client node**
   - 按照 [3 Write the client node](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html#write-the-client-node) 写一个 `add_two_ints_client.cpp` 文件。
   - 按照 [3.2 Add executable](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html#id2) 修改好 `CMakeLists.txt`。
4. **Build and run**
   - 在 build 之前先确认安装了所有的依赖（在本例里所需的依赖在安装 ROS2 的时候就已经安装了）: `cd .. && rosdep install -i --from-path src --rosdistro humble -y`
   - Build cpp_srvcli package: `colcon build --packages-select cpp_srvcli`
   - Source the setup file and run the **service** node: `source install/setup.bash && ros2 run cpp_srvcli server`
   - Open a new terminal. Source the setup file and run the **client** node: `source ~/dev_ws/install/setup.bash && ros2 run cpp_srvcli client 2 3`

## Creating custom ROS 2 msg and srv files
前两个 tutorial 都是用的已经定义好的接口，但经常需要自己定义数据接口，这就需要自己创建 `.msg` 和 `.srv` 文件。<br><br>
在同一个终端执行以下命令
1. **Create a package**
   ```bash
   cd ~/dev_ws/src && ros2 pkg create --build-type ament_cmake tutorial_interfaces
   cd tutorial_interfaces && mkdir msg && mkdir srv
   ```
2. **Create custom definitions**
   - msg definition: `cd /msg && echo "int64 num" >> Num.msg`
   - srv definition: `cd ../srv && echo -e "int64 a\nint64 b\nint64 c\n---\nint64 sum" >> AddThreeInts.srv`
3. **Modify `CMakeLists.txt`**: 按照 [3 `CMakeLists.txt`](https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html#cmakelists-txt) 修改 `CMakeLists.txt`。
4. **Modify `package.xml`**: 按照 [4 `package.xml`](https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html#package-xml) 修改 `package.xml`。
5. **Build**: `cd ~/dev_ws && colcon build --packages-select tutorial_interfaces`
6. **Test the new interfaces**
   1. Testing `Num.msg` with pub/sub. 参考 [7.1 Testing `Num.msg` with pub/sub](https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html#testing-num-msg-with-pub-sub)，修改 cpp_pubsub 包，使其 message type 变为 `Num.msg`。
   2. Testing `AddThreeInts.srv` with service/client. 参考 [7.2 Testing `AddThreeInts.srv` with service/client](https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html#testing-addthreeints-srv-with-service-client)，修改 cpp_srvcli 包，使其 service type 变为 `AddThreeInts.srv`。注意教程里并没有改 cpp_srvcli 包下源代码的文件名，但为了统一起见，还是把源代码文件名改成了 add_three_ints_xxx.cpp，这就要求把 `CMakeLists.txt` 里面的两句 `add_executable` 也给改了。

## Expanding on ROS 2 interfaces
前一个教程主要讲怎么自定义接口，这个教程主要讲怎么把所有的接口（可能会包括很多的数据类型）都集成在一个包里面。
1. **Create a package**
   ```bash
   cd ~/dev_ws/src && ros2 pkg create --build-type ament_cmake more_interfaces
   mkdir more_interfaces/msg
   ```
2. **Create a msg file**: 按照 [2 Create a msg file](https://docs.ros.org/en/humble/Tutorials/Single-Package-Define-And-Use-Interface.html#create-a-msg-file) 和 [3 Use an interface from the same package](https://docs.ros.org/en/humble/Tutorials/Single-Package-Define-And-Use-Interface.html#use-an-interface-from-the-same-package) 创建 msg file，写好 publisher 源代码，构建好 `CMakeLists.txt` 和 `package.xml`。
3. **Build and run**
   - `cd ~/dev_ws && colcon build --packages-up-to more_interfaces`
   - `source install/local_setup.bash && ros2 run more_interfaces publish_address_book`
   - Open a new terminal. `source ~/dev_ws/install/local_setup.bash && ros2 topic echo /address_book`

## Using parameters in a class (C++)
This tutorial will show you how to create parameters in a C++ class, and how to **set them in a launch file**.
1. **Create a package**: `cd ~/dev_ws/src && ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp`
2. **Write the C++ node**: 按照 [2 Write the C++ node](https://docs.ros.org/en/humble/Tutorials/Using-Parameters-In-A-Class-CPP.html#write-the-c-node) 写好节点的 C++ 源代码。
3. **Build and run**
   - `cd ~/dev_ws && rosdep install -i --from-path src --rosdistro humble -y`
   - `colcon build --packages-select cpp_parameters`
   - `source install/setup.bash && ros2 run cpp_parameters parameter_node`
   - Change the parameter via the console: `cd ~/dev_ws && source install/setup.bash && ros2 param set /parameter_node my_parameter earth`
   - Change the parameter via a launch file: 按照 [3.2 Change via a launch file](https://docs.ros.org/en/humble/Tutorials/Using-Parameters-In-A-Class-CPP.html#change-via-a-launch-file)，创建一个 launch file 并修改 `CMakeLists.txt`，然后 `source install/setup.bash && ros2 launch cpp_parameters cpp_parameters_launch.py`。

## Getting started with ros2doctor
**ros2doctor** checks all aspects of ROS 2, including platform, version, network, environment, running systems and more, and warns you about possible errors and reasons for issues.
- Examine your general *ROS 2 setup* as a whole: `ros2 doctor`
- Get a full report: `ros2 doctor --report`

## Creating and Using Plugins (C++)
This is the last Beginner Level tutorial. This tutorial shows how to create and load a simple plugin using pluginlib.<br>

### Background
pluginlib is a C++ library for loading and unloading plugins from within a ROS package. Plugins are dynamically loadable classes that are loaded from a runtime library (i.e. shared object, dynamically linked library). With pluginlib, one does not have to explicitly link their application against the library containing the classes – instead pluginlib can open a library containing exported classes at any point without the application having any prior awareness of the library or the header file containing the class definition. Plugins are useful for extending/modifying application behavior without needing the application source code.

### Usage
1. **Create two packages**: 在本教程中，会创建两个包：一个用于定义 base class，另一个提供 plugin。
   - Create the base class package: 参考 [1 Create the Base Class Package](https://docs.ros.org/en/humble/Tutorials/Pluginlib.html#create-the-base-class-package)
   - Create the plugin package: 参考 [2 Create the Plugin Package](https://docs.ros.org/en/humble/Tutorials/Pluginlib.html#create-the-plugin-package)，但注意在修改 `CMakeLists.txt` 的时候，教程里说需要添加的代码可能有一部分在原本的 `CMakeLists.txt` 里已经存在了，需要自己对比着看看，不要重复添加。
2. **Build and run**
   - `cd ~/dev_ws && colcon build --packages-select polygon_base polygon_plugins`
   - `source install/setup.bash && ros2 run polygon_base area_node`

# Other Tutorials
## tf2 Tutorials
> 参考教程目录: [tf2 tutorials](http://docs.ros.org/en/humble/Tutorials/Tf2/Tf2-Main.html)

最近看 LeGO-LOAM 源代码，需要使用到 tf 包，就学习了以下 tf 的用法。ROS1 是同时支持 tf 和 tf2 的，但 ROS2 似乎只支持 tf2 了。不过二者概念上应该差别不大，因为新电脑装的是 ROS2，这里就记录一下 tf2 的学习过程。

### Introduction to tf2
> 参考教程: [Introduction to tf2](http://docs.ros.org/en/humble/Tutorials/Tf2/Introduction-To-Tf2.html)
#### tf2 tools
- **view_frames**: `view_frames` creates a diagram of the frames being broadcasted by tf2 over ROS.
  - `ros2 run tf2_tools view_frames`
- **tf2_echo**: `tf2_echo` reports the transform between any two frames broadcasted over ROS.
  - `ros2 run tf2_ros tf2_echo [reference_frame] [target_frame]`
- **rviz**: `rviz` is a visualization tool that is useful for examining tf2 frames.
  - `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz`

### Writing a tf2 static broadcaster (C++)
> 参考教程: [Writing a tf2 static broadcaster (C++)](http://docs.ros.org/en/humble/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html)
- 第一部分讲了如何写一个 static broadcaster node。
- 第二部分讲了如何用命令行发布 static transforms，以及如何在 launch file 里启动 static_transform_publisher 节点
  - `ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id`: 用 xyz/rpy 的方式表达位姿
  - `ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id`: 用 xyz/quaterion 的方式表达位姿

### Writing a tf2 broadcaster (C++)
> 参考教程: [Writing a tf2 broadcaster (C++)](http://docs.ros.org/en/humble/Tutorials/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html)
- 主要内容：自己写一个 tf2 broadcaster，实现键盘控制乌龟移动，通过 `tf2_echo` 查看从世界坐标系到乌龟坐标系的坐标变换（也就是乌龟的位姿）。

### Writing a tf2 listener (C++)
> 参考教程: [Writing a tf2 listener (C++)](http://docs.ros.org/en/humble/Tutorials/Tf2/Writing-A-Tf2-Listener-Cpp.html)
- 主要内容：自己写一个 tf2 listener，实现前面 Introduction to tf2 里面的功能，即键盘控制乌龟 1 移动，乌龟 2 订阅乌龟 1 的位姿信息，并跟随乌龟 1。

### Adding a frame (C++)
> 参考教程: [Adding a frame (C++)](http://docs.ros.org/en/humble/Tutorials/Tf2/Adding-A-Frame-Cpp.html)
- 第一部分讲了如何添加一个相对 turtle 1 静止的参考系，然后让 turtle 2 跟随这个固定参考系运动。
- 第二部分讲了如何添加一个相对 turtle 2 运动的参考系，然后让 turtle 2 跟随这个动态参考系运动。

### Learning about tf2 and time (C++)
> 参考教程: [Learning about tf2 and time (C++)](http://docs.ros.org/en/humble/Tutorials/Tf2/Learning-About-Tf2-And-Time-Cpp.html)
- 主要内容：讲了如何使用 `lookupTransform()` 获取某个特定时间戳的 transform，以及 `lookupTransform()` 预留的 timeout 机制。

### Time travel with tf2 (C++)
> 参考教程: [Time travel with tf2 (C++)](http://docs.ros.org/en/humble/Tutorials/Tf2/Time-Travel-With-Tf2-Cpp.html)
- 主要内容：讲了 tf2 的 time travel feature（在机器人示教等应用场景中经常用到），可以求某个时间戳(5 secs ago)的坐标系(turtule2)到另一个时间戳(now)的另一个坐标系(carrot1)的变换。这需要使用到 `lookupTransform()` 的高级 API（需要传入 6 个参数）。