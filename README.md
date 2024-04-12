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

# 在Docker内配置TensorRT + GPU环境
流程：yolo v5 训练模型 --> model.pt --> 利用TensorRTx 转为 .engine
## CUDA + Cudnn + TensorRT 配置
### Nvidia显卡驱动安装
选择软件和更新
![软件和更新](resources/nvidia-driver-1.png)

在附加驱动中安装显卡驱动
![安装显卡驱动](resources/nvidia-driver-2.png)

测试是否安装成功：

输入
```
nvidia-smi
```
如果出现类似以下输出说明安装成功：
```
Fri Apr 12 20:55:28 2024       
+---------------------------------------------------------------------------------------+
| NVIDIA-SMI 535.161.07             Driver Version: 535.161.07   CUDA Version: 12.2     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage | GPU-Util  Compute M. |
|                                         |                      |               MIG M. |
|=========================================+======================+======================|
|   0  NVIDIA GeForce GTX 1660 ...    Off | 00000000:01:00.0  On |                  N/A |
| 40%   35C    P8              10W / 125W |    263MiB /  6144MiB |      2%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+
                                                                                         
+---------------------------------------------------------------------------------------+
| Processes:                                                                            |
|  GPU   GI   CI        PID   Type   Process name                            GPU Memory |
|        ID   ID                                                             Usage      |
|=======================================================================================|
|    0   N/A  N/A      1044      G   /usr/lib/xorg/Xorg                          131MiB |
|    0   N/A  N/A      1274      G   /usr/bin/gnome-shell                         58MiB |
|    0   N/A  N/A      7083      G   ...96,262144 --variations-seed-version       34MiB |
|    0   N/A  N/A    103872      G   ...erProcess --variations-seed-version       35MiB |
+---------------------------------------------------------------------------------------+

```
### Docker中使用GPU的配置
#### 安装NVIDIA Container Toolkit
进入官网按步骤安装即可：
[NVIDIA Container Toolkit官网](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
#### 配置Dockerfile和devcontainer.json
在dockerfile最后加入
```
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
```

Devcontainer.json中加入```"--gpus=all"```

示例
[Devcontainer.json](resources/devcontainer.json)

>注意：这是vscode的devcontainer,若在命令行中运行container,在命令后加上```--gpus=all```参数即可
### CUDA 安装
进入docker内，输入```nvidia-smi```验证驱动是否安装成功

>注意：不同电脑GPU型号不同，请在nvidia-smi处查看你最高支持的cuda版本，自行选择合适版本安装，本文仅作为示例

以cuda11.7 + cudnn8.5为例

按下图选择```runfile```,复制下面两行指令
![cuda](resources/cuda.png)
![cuda2](resources/cuda2.png)
![cuda3](resources/cuda3.png)

记得取消勾选安装驱动，然后选择```install```，回车，
接下来就会自动安装了

安装完成后，添加路径:

```
vim ~/.bashrc
```
>如果你用zsh，改成 vim ~/.zshrc

文件最后增加路径(记得改成你的版本)
```
export PATH=/usr/local/cuda-11.7/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-11.7/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```
让路径生效
```
source ~/.bashrc
```
>zsh用 source ~/.zshrc

测试安装是否成功
```
nvcc -V
```
输出应该类似于
```
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2021 NVIDIA Corporation
Built on Sun_Feb_14_21:12:58_PST_2021
Cuda compilation tools, release 11.7, V11.7.152
Build cuda_11.7.r11.7/compiler.29618528_0
```
### Cudnn安装
[下载链接](https://link.zhihu.com/?target=https%3A//developer.nvidia.com/rdp/cudnn-download)

选择和你的cuda版本匹配的cudnn版本(不推荐9.0.0版本！会导致TensorRT版本过高不支持Tensorrtx,
这里我选择8.6.0)


记得选择

```Local Install for Linux x86_64(Tar)```
等待下载完成后原地解压

```
tar -xvf cudnn-linux-x86_64-8.6.0.163_cuda11-archive.tar.gz
```

更改```usr/local/cuda/```文件夹下面的```include```和```lib64```文件夹的权限:
```
cd /usr/local/cuda
sudo chmod 777 include
sudo chmod 777 lib64
```
执行命令复制cudnn的文件进入cuda文件夹(在解压文件夹所在目录执行)
```
sudo cp cudnn-*-archive/include/cudnn*.h /usr/local/cuda/include 
sudo cp cudnn-*-archive/lib/libcudnn* /usr/local/cuda/lib64 
sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*
```
验证是否安装成功，执行命令：
```
sudo cat /usr/local/cuda/include/cudnn_version.h | grep CUDNN_MAJOR -A 2
```
输出应该类似于
```
#define CUDNN_MAJOR 8
#define CUDNN_MINOR 6
#define CUDNN_PATCHLEVEL 0
--
#define CUDNN_VERSION (CUDNN_MAJOR * 1000 + CUDNN_MINOR * 100 + CUDNN_PATCHLEVEL)

/* cannot use constexpr here since this is a C-only file */
```
安装成功

### TensorRT安装
[TensorRT下载网址](https://developer.nvidia.com/nvidia-tensorrt-8x-download)

我的CUDA版本是11.7，因此我要选择TensorRT8.4版本(不推荐10.0版本！会导致TensorRT版本过高不支持Tensorrtx)。在网页中找到对应版本,类似于：

```TensorRT 8.0 EA for Linux x86_64 and CUDA 11.1, 11.2 & 11.3 TAR package```
这样会下载tar压缩包格式的安装包

解压到你想要的文件夹(你自己记得住就行)

我把TensorRT安装在了```/usr/local```下
```
tar -xzvf TensorRT-8.0.0.3.Linux.x86_64-gnu.cuda-11.3.cudnn8.2.tar.gz -C /usr/local/
```
>若解压出错尝试```tar -xvf```

解压完成后，添加路径:

```
vim ~/.bashrc
```


文件最后增加路径(记得改成你的版本)
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/TensorRT-8.4.3.1/lib
```
让路径生效
```
source ~/.bashrc
```
>zsh用 source ~/.zshrc

测试安装是否成功,编译TensorRT的示例，会在bin路径下生成一个针对MINIST数据集的sample_mnist可执行文件
```
cd /usr/local/TensorRT-8.0.0.3/samples/sampleMNIST
sudo make
cd ../../bin/
./sample_mnist
```
输出应该类似于
```
&&&& RUNNING TensorRT.sample_mnist [TensorRT v8000] # ./sample_mnist
[10/14/2022-17:01:44] [I] Building and running a GPU inference engine for MNIST
[10/14/2022-17:01:44] [I] [TRT] [MemUsageChange] Init CUDA: CPU +146, GPU +0, now: CPU 151, GPU 407 (MiB)
[10/14/2022-17:01:44] [I] [TRT] [MemUsageSnapshot] Builder begin: CPU 153 MiB, GPU 407 MiB
[10/14/2022-17:01:45] [W] [TRT] TensorRT was linked against cuBLAS/cuBLAS LT 11.4.2 but loaded cuBLAS/cuBLAS LT 11.4.1

......
```
为了防止找不到 TensorRT 的库，建议把 TensorRT 的库和头文件链接一下
```
sudo ln -s /home/zjl/Downloads/TensorRT/TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.3.cudnn8.2/TensorRT-8.4.3.1/lib/* /usr/lib/
sudo ln -s /home/zjl/Downloads/TensorRT/TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.3.cudnn8.2/TensorRT-8.4.3.1/include/* /usr/include/
```
## Yolov5模型转换
根据你训练模型所用的yolov5版本，```git clone```
yolov5和tensorrtxx

(这里我是yolov5-v7.0和tensorrtx-yolov5-v7.0)

[yolov5-v7.0](https://link.zhihu.com/?target=https%3A//github.com/ultralytics/yolov5/tree/v7.0)

[tensorrtx-yolov5-v7.0](https://link.zhihu.com/?target=https%3A//github.com/wang-xinyu/tensorrtx/tree/yolov5-v7.0)

将```/tensorrtx-yolov5-v7.0/yolov5```中的```gen_wts.py```文件拷贝到```yolov5-7.0```目录中，并将模型文件也拷贝到此目录

运行

```
python gen_wts.py -w your_model_name.pt
```
生成```.wts```文件（模型转换的中间文件）

然后进入```/tensorrtx-yolov5-v7.0/yolov5```

修改```src/config.h```文件中的```CLASS_NUM```为你训练的模型的标签类别数

>其他较旧版本:
修改```yololayer.h```文件中的```CLASS_NUM```为你训练的模型的标签类别数

![tensorrtx](resources/tensorrtx.png)

修改```CMakeLists.txt```中的这一段:
```
...

# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
# TODO(Call for PR): make TRT path configurable from command line
include_directories(/usr/local/TensorRT-8.4.3.1/include/) #这两行修改为你的路径
link_directories(/usr/local/TensorRT-8.4.3.1/lib/) #这两行修改为你的路径

...
```

然后编译
```
mkdir build
cd build
cmake ..
make
```
生成可执行文件

然后将```.wts```文件拷贝到```build```文件夹中，并执行
```
./yolov5_det -s your_model_name.wts your_model_name.engine s
```
>其他较旧版本:
>```
>./yolov5 -s best.wts best.engine s
>```

生成```.engine```文件，这便是我们tensorrt部署所需要的模型文件


