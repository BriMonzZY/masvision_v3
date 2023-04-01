# masvision_v2

**Mas步兵自瞄v2**



## TODO

- [ ] 结果预测
- [ ] 结果滤波
- [ ] 修改图像预处理方案



## 整体架构

masvision_v2基于 [RoboRTS](https://github.com/RoboMaster/RoboRTS) 的detection部分编写，运行于ros-noetic

整个工程使用**抽象工场模式**来构建

下面是框架包含的软件包和对应的功能以及依赖的软件包

| Package           | 功能             | 内部依赖                                   |
| ----------------- | ---------------- | ------------------------------------------ |
| roborts_base      | 嵌入式通信接口   | roborts_msgs                               |
| roborts_camera    | 相机驱动包       | roborts_common                             |
| roborts_common    | 通用依赖包       | -                                          |
| roborts_detection | 视觉识别算法包   | roborts_msgs roborts_common roborts_camera |
| roborts_msgs      | 自定义消息类型包 | -                                          |
| roborts_bringup   | 启动包           | roborts_base roborts_common roborts_msgs   |
| roborts_tracking  | 视觉追踪算法包   | roborts_msgs                               |



## masvision_v2依赖环境和编译工具

- Ubuntu 20.04LTS
- ROS-noetic

- OpenCV 4.2.0**(一定要4.2.0)**：https://github.com/opencv/opencv/archive/4.2.0.zip

直接执行下面的命令以安装依赖：

```shell
sudo apt-get install -y ros-noetic-cv-bridge           \
                        ros-noetic-image-transport     \
                        ros-noetic-interactive-markers \
                        ros-noetic-tf                  \
                        ros-noetic-pcl-*               \
                        ros-noetic-libg2o              \
                        ros-noetic-rviz                \
                        ros-noetic-usb-cam*			   \
                        ros-noetic-image-view		   \
                        ros-noetic-teleop-twist-keyboard \
                        python3-catkin-tools		   \
                        protobuf-compiler              \
                        libprotobuf-dev                \
                        libsuitesparse-dev             \
                        libgoogle-glog-dev             \
                        libeigen3-dev				   \
                        openni2-utils				   \
                        libpcap-dev					   \
                        liburdfdom-tools
```

**使用catkin_tools工具编译：(注意不是使用catkin_make工具)**

```shell
sudo apt-get install python3-catkin-tools -y

# 常用命令
catkin build # 编译
catkin build package_name # 编译指定包
catkin clean # 清除build文件
catkin clean package_name # 清除指定build文件
```

在启动节点前务必执行：（或放到~/.bashrc中）

```shell
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
```



## roborts_base

**使用最新开源的base模块** [RoboRTS-Base](https://github.com/RoboMaster/RoboRTS-Base)



**Module**是父类，定义了一个虚函数Module，和一个Handle类类型的智能指针handle_

sdk中有自己的协议命令订阅和发布结构



使用base.launch文件启动base模块，调用base.yaml，可以在base.yaml中修改参数



---

### 协议

协议的代码描述在`protocol.cpp`和`protocol.c`中

协议内容和数据包内容在`protocol_content.h`中定义

协议包含：**帧头数据 + 命令码ID + 数据 + 帧尾校验数据**

> 协议数据按照通信方式分为
>
> - 底层发送给上层的数据：
>
>   1 反馈信息：包含各个机构传感器反馈信息、底层计算出来的一些反馈信息；
>
>   2 底层状态信息：包含底层设备运行状态、底层对上层数据的一些响应等；
>
>   3 转发数据：包含裁判系统的全部信息、服务器端的自定义信息；
>
> - 底层接收的上层数据：
>
>   1 控制信息：上层对底层 3 个执行机构的控制信息；

#### 帧头数据

```c++
typedef struct Header {
  uint32_t sof : 8;
  uint32_t length : 10;
  uint32_t version : 6;
  uint32_t session_id : 5;
  uint32_t is_ack : 1;
  uint32_t reserved0 : 2; // Always 0
  uint32_t sender: 8;
  uint32_t receiver: 8;
  uint32_t reserved1 : 16;
  uint32_t seq_num : 16;
  uint32_t crc : 16;
} Header;
```

| 帧头数据     | 占用字节 | 描述                     | 备注                          |
| ------------ | -------- | ------------------------ | ----------------------------- |
| SOF          | 1        | 数据的域ID               | 0xAA                          |
| ver_data_len | 2        | 每帧数据长度和协议版本号 | length、version               |
| session      | 1        | 包序号                   | session_id、is_ack、reserved0 |
| sender       | 1        | 发送者地址               |                               |
| receiver     | 1        | 接收者地址               |                               |
| res          | 2        | 保留位                   |                               |
| seq          | 2        | 包序号                   |                               |
| crc16        | 2        | 帧头的CRC16校验结果      |                               |

帧头在一个数据包中占用16个字节

aa 16 0 0 0 1 0 0 5 0 22 7e 1 0 0 0 0 0 7f 4d 5f b4

aa 16 0 0 0 2 0 0 6 0 66 8e 1 0 0 0 0 0 7c 6c b 2b

aa 16 0 0 0 2 0 0 0 0 65 2e 1 0 0 0 0 0 b6 c8 31 ef

aa 16 0 0 0 1 0 0 7 0 23 1e 1 0 0 0 0 0 39 d1 b6 f7

aa 16 0 0 0 2 0 0 2b 0 7b de 1 0 0 0 0 0 51 29 c3 55

aa 17 0 0 0 2 0 0 43 0 59 8e 3 3 1 c2 39 0 72 81 98 c3 85

aa 17 0 0 0 2 0 0 29 0 77 2e 3 3 0 3a 39 0 72 78 ff 74 a9

#### 命令码

命令码包含了一帧的具体信息数据

```c++
uint8_t cmd_set_prefix[] = {cmd_id, cmd_set};
```

| 命令码 | 占用字节 |
| ------ | -------- |
| cmdid  | 2        |



#### 数据

| 数据 | 占用字节    |
| ---- | ----------- |
| data | data_length |

##### 裁判系统学生串口信息

查看裁判系统手册

##### 控制信息与推送信息

`protocol_content.h`

从机接收：`RoboRTS-Firmware/application/infantry_cmd.h`

主机发送：`roborts_base/roborts_sdk/protocol/protocol_define.h`

`/roborts_base/roborts_sdk/include/protocol_content.h`



#### 校验数据

对一帧的数据进行CRC32校验

| 校验数据 | 占用字节 |
| -------- | -------- |
| crc_data | 4        |



## roborts_camera

[文档](https://robomaster.github.io/RoboRTS-Tutorial/#/sdk_docs/roborts_camera)

本软件包为相机驱动，发布图像数据以及相机信息

启动节点：

```shell
rosrun roborts_camera roborts_camera_node
```

在`config/camera_param.prototxt`中修改相机参数



## roborts_detection

检测装甲板

编译`catkin build roborts_detection`

### 图像输入说明



可以直接使用ros的usb_cam包调用usb摄像头，而**不使用roborts_camera发布摄像头图像信息**：

（roborts_camera也能用）

安装usb_cam：

```shell
sudo apt install ros-noetic-usb-cam* -y
sudo apt install ros-noetic-image-view -y
```

启动usb_cam：

```shell
roslaunch usb_cam usb_cam-test.launch
```

launch位置：`/opt/ros/noetic/share/usb_cam/launch`

rqt查看图像：`rqt_image_view`

随后将`/roborts_detection/armor_detection/config/armor_detection.prototxt`中的摄像头名称改为usb_cam

启动节点：`rosrun roborts_detection armor_detection_node`



#### 摄像头标定

[ros标定](https://blog.csdn.net/weixin_44543463/article/details/120704327)

然后将`~/.ros/camera_info`中的文件复制到`roborts_bringup/params`中

如果不能读取文件可以把标定文件换成绝对路径

roborts_bringup的usb_cam.launch文件里修改标定文件的路径



#### 使用视频作为输入(videototopic)

工作环境下的videototopic包提供了视频发布topic的服务

使用：

```
 roslaunch roborts_bringup image_publisher.launch
```

即可将视频发布到指定的topic

可调参数位置：rm_ws/src/videototopic/config/**videototopic.yaml**

本节点同时输出camera_info信息

（开发者记：image_raw和camera_info必须同步发布（使用advertiseCamera），否则detection模块无法启动）



### 装甲板检测

在armor_detection_node.cpp的**ExecuteLoop()**函数中执行检测算法并发布姿态信息

装甲板检测流程均在constraint_set.cpp的**DetectArmor()**中执行

**需要启动调试节点**`rosrun roborts_detection armor_detection_client`并输入“1”以启动检测线程



在没有摄像头的情况下可以使用videototopic节点和detection节点进行快速验证



## roborts_tracking

KCF追踪算法(目前未打算使用)

```shell
rosrun roborts_tracking roborts_tracking_test
```







## 启动文件说明

启动usb摄像头：

```shell
roslaunch roborts_bringup usb_cam.launch
```

视频发布话题：（注意修改/config/videototopic.yaml中的视频路径）

```shell
roslaunch roborts_bringup image_publisher.launch
```

启动装甲板检测(roborts_detection)

```shell
rosrun roborts_detection a rmor_detection_node
rosrun roborts_detection armor_detection_client
```

一键启动装甲板检测：

```shell
roslaunch roborts_bringup armor_detection.launch
```





Linux虚拟串口：

```shell
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

