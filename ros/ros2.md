> 本文用于记录 ROS2 的安装和学习


- [Installation](#installation)
- [Tutorial](#tutorial)
  - [Base](#base)
  - [Nodes](#nodes)
  - [Topics](#topics)
  - [Services](#services)
    - [Services vs. Topics](#services-vs-topics)
  - [Parameters](#parameters)
  - [Actions](#actions)
    - [Mechanism of Actions](#mechanism-of-actions)
    - [Usage of Actions](#usage-of-actions)
  - [rqt_console](#rqt_console)
  - [Launch](#launch)
  - [Bag](#bag)
  - [Workspace](#workspace)


## Installation
新电脑只有 Ubuntu 22 能够识别无线网卡，而 Ubuntu 22 只支持 ROS2，因此被迫使用 ROS 2 Humble Hawksbill。
参考[官方安装链接](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)，安装桌面版，注意在添加软件源的时候换成中科大的就行：<br>
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros2/ubuntu/ $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## Tutorial
参考[官方教程](https://docs.ros.org/en/humble/Tutorials.html#)，把 Beginner Level 的教程过了一遍。

### Base
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

### Nodes
- **ros2 run**: Launch an executable from a package.（启动一个节点）
  - `ros2 run <package_name> <executable_name>`
- **ros2 node list**: Show the names of all *running* nodes.
  - `ros2 node list`
- **Remapping**: Reassign default node properties, like node name, topic names, service names, etc., to custom values.<br>
  下面这条指令会重新开一个乌龟节点，但节点名不再是默认的 /turtlesim，而是被修改为了 /my_turtle。
  - `ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle`
- **ros2 node info**: Show a list of subscribers, publishers, services, and actions (the ROS graph connections) that interact with that node.
  - `ros2 node info <node_name>`

### Topics
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

### Services
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

#### Services vs. Topics
Service 和 topic 的区别在于
- Service 是一个 request/responde 模式，client node 需要给 server node 发一个 request，然后由 server node 给出一个 response
- Topic 是一个 publish/subscribe 模式，publisher 发布一个 topic，然后由 subscriber 订阅这个 topic

### Parameters
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

### Actions
#### Mechanism of Actions
Action 是用于长期运行任务的一种 communication type。<br>
Action 包括三部分: a goal, feedback, and a result。<br>
Action 用的是 client-server 模型，但跟 service 的差别在于 action 在进行过程中**可以收到反馈**并且是**可以被中断**的，如果是由 client 侧提出要停止一个 goal，称为 cancel a goal，如果是由 server 侧决定要停止一个 goal，称为 abort a goal。<br>
Action 的具体工作流程为
- action client 借助 goal service 给 action server 发一个 request，然后 action server 给出相应的 response。
- action client 借助 result service 再给 action server 发一个 request，在完成 goal 之前，action server 会一直通过 feedback topic 给 action client 发送 feedback，任务完成后会给出一个 response。
<img src='../img/ros2_1.gif'><br>

#### Usage of Actions
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

### rqt_console
`rqt_console` is a GUI tool used to introspect log messages.
- **Start *rqt_console***: `ros2 run rqt_console rqt_console`
- **Logger levels**: *Fatal, Error, Warn, Info, Debug* 严重等级依次降低，默认的 logger level 是 *Info*，即没有 *Info* 严重的 log 是看不到的。
  - `ros2 run <package_name> <executable_name> --ros-args --log-level WARN`: 修改默认的 logger level 为 Warn

### Launch
Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.
- **Running a launch file**: `ros2 launch <node_name> <launch_file>`. Launch file 可以用 python 写，也可以使用 XML 和 YAML 格式。如何写一个 launch file 参考 [Launch Tutorials](https://docs.ros.org/en/humble/Tutorials/Launch/Launch-Main.html)。

### Bag
- **ros2 bag record**: Record the data published to a topic.
  - `ros2 bag record <topic_name>`
  - `ros2 bag record -o <bag_file_name> <topic1> <topic2>`: Record multiple topics and change the name of the rosbag file.(The default name pattern is `rosbag2_year_month_day-hour_minute_second`)
- **ros2 bag info**: Show details about a bag file.
  - `ros2 bag info <bag_file_name>`
- **ros2 bag play**: Replay a bag file.
  - `ros2 bag play <bag_file_name>`

### Workspace
A workspace is a directory containing ROS 2 packages.