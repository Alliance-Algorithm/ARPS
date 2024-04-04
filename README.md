# Alliance战队-雷达兵种项目文档 

## 概述

本项目基于ROS2开发，旨在实现车辆目标检测和位置信息传输，流程为利用YOLOv5进行图像识别、将识别结果传输至Unity程序，并通过串口发送至雷达系统。项目通过多个节点协同工作，完成从图像处理到位置数据传输的全流程。

## 整体架构

项目的整体架构如下：

```
.                   #根目录
├── Dockerfile               # Docker容器构建文件
├── README.md                # 项目说明文档
└── radar_ws                 # ROS2工作空间
    ├── build                   # 编译输出目录
    ├── install                 # 安装输出目录
    ├── launch.sh               # 启动脚本
    ├── log                     # 日志目录
    ├── rebuild.sh              # 重新构建脚本
    ├── resources               # 资源文件夹
    │   ├── models               # 模型文件夹
    │   │   ├── armor_identfy.onnx   # 装甲板识别模型
    │   │   └── car_identfy.onnx     # 车辆识别模型
    │   ├── user_logs            # 用户日志文件夹
    │   │   ├── detect.log           # 车辆检测日志
    │   │   └── serial.log           # 串口通信日志
    │   └── videos               # 视频文件夹
    └── src             # ROS2包源代码目录
        ├── ROS_TCP_Endpoint    # ROS TCP终端节点
        ├── car_detect          # 车辆检测节点
        │   ├── CMakeLists.txt      # CMake构建配置文件
        │   ├── package.xml         # 包描述文件
        │   └── src                 # 源代码目录
        │       ├── Logger.hpp      # 日志工具头文件
        │       └── Yolov5.cpp      # YOLOv5检测实现源文件
        ├── radar_bringup       # 雷达启动节点
        │   ├── CMakeLists.txt      # CMake构建配置文件
        │   ├── launch              # 启动文件目录
        │   │   └── radar.launch.py # 雷达启动配置文件
        │   └── package.xml         # 包描述文件
        ├── radar_serial        # 串口通信节点
        │   ├── CMakeLists.txt      # CMake构建配置文件
        │   ├── package.xml         # 包描述文件
        │   └── src                 # 源代码目录
        │       ├── Logger.hpp      # 日志工具头文件
        │       └── radar_serial.cpp  # 串口通信实现源文件
        ├── ros2_serial         # ROS2串口通信包
        │   ├── CMakeLists.txt      # CMake构建配置文件
        │   ├── LICENSE             # 许可证文件
        │   ├── README.md           # 说明文档
        │   ├── include             # 头文件目录
        │   │   └── serial          # 串口库头文件目录
        │   ├── package.xml         # 包描述文件
        │   └── src                 # 源代码目录
        │       ├── impl            # 实现文件目录
        │       └── serial.cc       # 串口通信实现源文件
        └── unity_raycast       # Unity节点

```

- **car_detect节点**：负责读取摄像机图像，利用双层YOLOv5网络进行目标识别，分别得到机器人坐标和种类，发送给unity仿真程序。
  
- **unity_raycast程序**：接收car_detect发送的坐标信息，进行碰撞检测获得三维坐标，发送给radar_serial。

- **radar_serial节点**：接收unity程序发送的目标位置信息，将数据通过串口发送至雷达系统。

## 3. 功能节点详解

### 3.1. car_detect节点

#### 3.1.1. 功能介绍

car_detect节点是整个系统的核心节点，主要负责图像处理和目标检测。其功能包括：

- 读取摄像机图像。
- 利用YOLOv5模型进行目标识别，包括车辆和装甲板。
- 提取识别结果中的机器人坐标和种类。
- 发布目标位置信息至Unity程序。

#### 3.1.2. 代码结构

car_detect节点的代码结构如下：

- **YOLOv5Detector类**：用C++重写检测部分代码，封装了YOLOv5目标检测器，负责加载模型和执行检测。
  
- **Points_publisher类**：数据发布类，负责发布检测结果至指定话题。

### 3.2. radar_serial节点

#### 3.2.1. 功能介绍

radar_serial节点负责接收unity程序发送的目标位置信息，并通过串口将数据发送至裁判系统。其功能包括：

- 订阅unity程序发布的目标位置信息。
- 对接收的数据进行解析和打包。
- 通过串口将打包好的数据发送至雷达系统。

#### 3.2.2. 代码结构

radar_serial节点的代码结构如下：

- **CRC16_Check函数**：实现CRC16校验算法，用于数据完整性校验。
  
- **CRC8_Check函数**：实现CRC8校验算法，用于帧头完整性校验。

- **serial_data_pack函数**：将接收到的目标位置信息打包成串口数据。

- **PositionsSubscriber类**：数据订阅类，负责订阅unity程序发布的目标位置信息，并发送至串口。

## 4. 部署方法

克隆本项目，按照DockerFile构建镜像后进入docker容器内

运行脚本进行编译(约20s)
```zsh
cd radar_ws
./rebuild.sh
```

在 ```./radar_ws/resources/models```下放置训练好的onnx模型(命名为```car_identfy.onnx```和```armor_identfy.onnx```)


运行节点
```zsh
./launch.sh
```

程序运行日志会输出至```/resources/user_logs```目录下

日志示例:
```
---[ 开始日志 | 时间 2024-04-04 12:33:08 ]---
[2024-04-04 12:33:08] [INFO]: ROS2Message node starting...
[2024-04-04 12:33:08] [INFO]: [√]successfully started.
[2024-04-04 12:33:08] [INFO]: YOLOv5Detector starting...
[2024-04-04 12:33:08] [INFO]: [√]car_detector successfully started.
[2024-04-04 12:33:08] [INFO]: [√]armor_detector successfully started.
[2024-04-04 12:33:08] [INFO]: Loading model...
[2024-04-04 12:33:08] [INFO]: [√]car_detect model loaded.
[2024-04-04 12:33:08] [ERROR]: [x]Error in command_callback: OpenCV(4.5.4) ./modules/dnn/src/onnx/onnx_importer.cpp:739: error: (-2:Unspecified error) in function 'handleNode'
---[ 结束日志 | 时间 2024-04-04 12:33:08 ]---
```



