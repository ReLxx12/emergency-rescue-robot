# 紧急救援机器人核心代码

基于 ROS 1 的紧急救援机器人核心源码，包含任务管理、航点导航、视觉定位与检测、底盘和机械臂控制，以及端侧大模型接入。

## 核心技术

### 1. ROS 多节点协作与救援任务调度

使用 Python 与 C++ 编写 ROS 节点，通过 Topic、自定义消息、TF 坐标变换和 YAML 参数连接感知、导航及执行机构。任务管理节点根据目标颜色和放置区域生成任务，依次组织途经点导航、目标抓取、分类放置、返航与泊车，并结合左右机械臂动作序列完成救援流程。

代码入口：[任务管理](mission_ws/src/mission/mission_manage/scripts/mission_manage_node.py)、[自定义消息](catkin_ws/src/user_msgs/msg)、[任务参数](mission_ws/src/mission/mission_manage/params/mission_params.yaml)。

### 2. AprilTag 视觉定位与卡尔曼滤波

通过 AprilTag 标签检测、相机内参与标签尺寸，使用 OpenCV `solvePnP` 解算相机与标签的相对位姿；再结合相机安装外参和标签在地图中的已知位姿，计算车体在地图坐标系下的位置与姿态。定位节点以位置、速度和欧拉角构成九维状态，使用卡尔曼滤波平滑观测，并处理角度跨越 ±π 时的误差。

代码入口：[AprilTag 位姿解算](catkin_ws/src/algorithm/detection/apriltag_localization/src/apriltag_localization_node.cpp)、[车体定位与滤波](mission_ws/src/mission/car_localization/scripts/car_localization_node.py)。

### 3. 三轴 PID 航点闭环控制

分别控制平面位置 `x`、`y` 与航向角 `yaw`，将地图坐标系下的控制速度转换为车体速度，通过 `/cmd_vel` 驱动底盘。控制器支持积分分离、输出限幅、运行时参数调整，以及按位置误差、角度误差和连续满足次数判定到点。航点、到点阈值和 PID 参数均可配置。

代码入口：[航点导航节点](mission_ws/src/mission/wp_nav_pid/scripts/wp_nav_pid_node.py)、[PID 控制器](mission_ws/src/mission/wp_nav_pid/scripts/pid.py)。

### 4. 颜色与几何特征检测、端侧目标检测

圆柱目标检测采用 HSV 颜色阈值分割、腐蚀与膨胀处理，以及轮廓面积、圆度和长宽比筛选，识别红蓝目标并输出图像位置；调试界面支持在线调整阈值。仓库还包含 YOLOv5 的 RKNN 推理接入代码，用于在 Rockchip 平台加载模型、执行推理和完成检测后处理。

代码入口：[圆柱目标检测](catkin_ws/src/algorithm/detection/cylinder_detection/scripts/cylinder_det_node.py)、[YOLOv5-RKNN 推理](catkin_ws/src/algorithm/detection/yolov5_rknn/src/yolov5_rknn_det.cc)。

### 5. 视觉引导的双臂抓取与运动学逆解

依据目标像素位置、相机内参和安装外参，将图像观测转换为车体坐标下的抓取目标；结合场景中标定的目标距离与底盘对正误差进行修正，再补偿夹爪中心相对机械臂末端的偏移。目标位姿通过 TF 转换到机械臂基座坐标系，由逆运动学节点调用运动学库计算关节角，并与左右臂的预设动作配合完成抓取和放置。该方法依赖特定场景的距离标定与抓取姿态配置。

代码入口：[抓取坐标解算](mission_ws/src/mission/mission_manage/scripts/utils.py)、[机械臂逆解节点](catkin_ws/src/actuator/arm/src/arm_ik_node.cpp)、[双臂动作序列](mission_ws/src/mission/mission_manage/scripts/arm_motions.py)。

### 6. DeepSeek / RKLLM 自然语言任务解析

通过 RKLLM 加载端侧语言模型，将中文指令转换为“目标颜色—放置区域”任务列表。例如，“把红色放一号，蓝色放二号”会被整理为对应的抓取与放置任务。任务管理节点采用少样本提示词引导输出格式，并通过正则提取、`ast.literal_eval` 解析和字段映射整理结果；解析失败时使用关键词匹配作为回退。

代码入口：[RKLLM 模型接入](catkin_ws/src/algorithm/llm/deepseek/src/DeepSeek.cpp)、[指令解析与任务生成](mission_ws/src/mission/mission_manage/scripts/mission_manage_node.py)。

### 7. 执行机构通信与导航脱困

底盘控制节点将 ROS 速度指令编码后通过串口发送，结合死区、速度限制及遥控、导航、保持、到点等状态控制底盘。任务管理节点监测地图坐标中的车体位移，在长时间未产生有效位移时执行“暂停导航—后退—恢复导航”的脱困流程。

代码入口：[底盘串口控制](catkin_ws/src/actuator/chassis/scripts/chassis_control_node.py)、[任务管理与脱困](mission_ws/src/mission/mission_manage/scripts/mission_manage_node.py)。

## 代码结构

| 路径 | 主要功能 |
| --- | --- |
| `catkin_ws/src/actuator` | 底盘、机械臂、云台和炮台控制，URDF 模型及机器人描述资源 |
| `catkin_ws/src/user_msgs` | 自定义 ROS 消息 |
| `catkin_ws/src/user_params` | 设备配置与控制参数 |
| `catkin_ws/src/algorithm/detection` | AprilTag 定位、圆柱目标检测和 YOLOv5 RKNN 推理接入 |
| `catkin_ws/src/algorithm/llm/deepseek` | 端侧 DeepSeek 大模型的 ROS 接入节点 |
| `mission_ws/src/mission` | 救援任务管理、航点 PID 导航和基于标签的定位 |

## 构建与运行依赖

本仓库保留 ROS 1 catkin 工程的源码。请先安装目标设备所需的 ROS 发行版及各包 `package.xml` 中声明的依赖，再构建 `catkin_ws`；随后加载该工作区环境，将 `mission_ws` 作为上层工作区构建。具体 ROS 版本和开发板环境需与实际机器人配置核对。

以下依赖需要单独准备：

- YOLO/RKNN/RGA 和 DeepSeek/RKLLM 所需的厂商 SDK 头文件及运行库。
- 机械臂逆运动学包引用的已编译运动学库。
- 本仓库未收录的传感器驱动、导航组件和其他外部 ROS 包。

本项目代码已在实际机器人上运行。本仓库由实机工程提取核心源码整理而成，复现时需补齐上述外部依赖，并按实际设备调整配置。

## 设备配置

部分脚本和 YAML 配置使用 `/home/orangepi/`、`/home/jetson/` 下的绝对路径，并包含针对具体机器人的相机标定、机械臂、航点和地图参数。部署时需按实际环境调整路径，并在启动运动控制节点前核对参数。

任务脱困逻辑会发出后退速度指令。调试时应固定好机器人或确保周围测试区域空旷，并准备急停措施。

## 收录范围

本仓库包含核心源码、ROS 消息、启动文件、必要配置和机器人描述资源。构建产物、Python 缓存、厂商 SDK 二进制及头文件、模型权重、数据集、压缩包、可执行文件、独立地图文件、比赛资料和 LLaMA-Factory 训练数据均未收录。

第三方 ROS 包需单独安装或管理，并保留其各自的许可证及声明。本仓库暂未声明统一开源许可证；若后续改为公开仓库，需先确认各包代码的权属和许可证兼容性。
