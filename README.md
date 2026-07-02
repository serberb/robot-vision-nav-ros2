# Robot Vision & Navigation ROS2 整合项目

> **平台**: Ubuntu 22.04 + ROS2 Humble  
> **目标**: 将 Astra 视觉识别、点云目标定向识别、Nav2 导航三大工作空间整合为统一的完整工程项目

---

## 项目概述

本项目整合了三个独立 ROS2 工作空间，构建一个具备以下能力的智能机器人系统：

1. **Astra 视觉识别** — 基于 YOLOv8 的 RGB-D 目标检测与 3D 位姿估计
2. **点云目标定向识别** — 基于 PCL 的 RANSAC 分割 + 欧式聚类 + OBB 有向包围盒分析
3. **Nav2 自主导航** — SLAM 建图、定位、路径规划、虚拟墙管理、底盘控制

---

## 目录结构

```
robot-vision-nav-ros2/
├── README.md                          # 本文档
├── docs/                              # 设计文档
│   ├── 01-architecture.md             # 系统架构总览
│   ├── 02-workspace-integration.md    # 工作空间整合方案
│   ├── 03-vision-pipeline.md          # 视觉识别管道
│   ├── 04-pointcloud-pipeline.md      # 点云目标定向识别
│   ├── 05-navigation-pipeline.md      # 导航系统设计
│   ├── 06-virtual-walls.md            # 虚拟墙管理
│   ├── 07-topic-topology.md          # 话题拓扑图
│   ├── 08-launch-sequence.md          # 启动流程
│   ├── 09-modification-checklist.md  # 代码修改清单
│   └── 10-deployment-guide.md        # 部署运行指南
├── launch/                            # 整合后的 launch 文件
│   ├── bringup_all.launch.py          # 一键启动全部
│   ├── vision_only.launch.py          # 仅启动视觉
│   ├── pointcloud_only.launch.py      # 仅启动点云
│   ├── navigation_only.launch.py      # 仅启动导航
│   ├── slam_mapping.launch.py         # SLAM 建图模式
│   └── nav2_autonomous.launch.py      # 自主导航模式
├── config/                            # 配置文件
│   ├── nav2_params_final.yaml         # 最终版 Nav2 参数
│   ├── ekf.yaml                       # EKF 融合参数
│   ├── slam_toolbox.yaml              # SLAM 参数
│   └── virtual_walls_points.json      # 虚拟墙坐标
├── scripts/                           # 工具脚本
│   ├── arrow_teleop.py                # 键盘遥控
│   ├── scan_retime.py                 # 激光时间戳重对齐
│   ├── print_odom_pose.py             # 打印里程计位姿
│   ├── collect_wall_points.py         # 虚拟墙采集（RViz）
│   ├── apply_virtual_walls.py         # 虚拟墙应用到地图
│   └── add_multi_virtual_walls.py     # 批量硬编码虚拟墙
└── docker/                            # Docker 部署（可选）
    └── Dockerfile
```

---

## 快速开始

### 1. 环境准备

```bash
# 系统要求
Ubuntu 22.04
ROS2 Humble
Python 3.10
PCL 1.12
OpenCV 4.5

# 依赖安装
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-slam-toolbox ros-humble-robot-localization
sudo apt install -y libpcl-dev ros-humble-pcl-conversions
pip install ultralytics opencv-python pillow pyyaml
```

### 2. 工作空间构建

```bash
# 统一工作空间
cd ~/robot_ws
mkdir -p src
cd src

# 1. 克隆视觉包
git clone <astra_camera_repo> ros2_astra_camera
git clone <robot_vision_repo> robot_vision

# 2. 克隆点云包
git clone <my_pcl_pkg_repo> my_pcl_pkg

# 3. 克隆导航包
git clone <my_robot_bringup_repo> my_robot_bringup
git clone <sllidar_repo> sllidar_ros2

# 构建
cd ~/robot_ws
colcon build --symlink-install --packages-select \
  astra_camera astra_camera_msgs \
  robot_vision robot_vision_msgs \
  my_pcl_pkg \
  my_robot_bringup my_robot_base my_robot_sensors sllidar_ros2
```

### 3. 启动流程

```bash
# 一键启动全部（视觉 + 点云 + 导航）
ros2 launch robot_bringup bringup_all.launch.py

# 或分模块启动
ros2 launch robot_bringup vision_only.launch.py      # 仅视觉
ros2 launch robot_bringup pointcloud_only.launch.py  # 仅点云
ros2 launch robot_bringup navigation_only.launch.py  # 仅导航

# SLAM 建图
ros2 launch robot_bringup slam_mapping.launch.py
# 然后使用键盘遥控
python3 scripts/arrow_teleop.py

# 虚拟墙管理
python3 scripts/collect_wall_points.py    # RViz 采集
python3 scripts/apply_virtual_walls.py    # 应用到地图

# 自主导航
ros2 launch robot_bringup nav2_autonomous.launch.py
```

---

## 系统架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Robot Vision & Navigation System                   │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐ │
│  │   Astra Pro  │   │   SLLIDAR    │   │   IMU        │   │   Chassis    │ │
│  │  (RGB-D)     │   │  (Laser)     │   │  (MPU6050)   │   │  (Mecanum)   │ │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘   └──────┬───────┘ │
│         │                │                │                │          │
│  ┌──────▼───────┐   ┌──────▼───────┐   ┌──────▼───────┐   ┌──────▼───────┐ │
│  │astra_camera  │   │sllidar_ros2  │   │mpu6050_node  │   │base_serial   │ │
│  │ 驱动节点      │   │ 激光驱动      │   │ IMU 节点     │   │ 底盘节点      │ │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘   └──────┬───────┘ │
│         │                │                │                │          │
│  ┌──────┴───────┐   ┌──────┴───────┐   └──────┬───────┘   └──────┬───────┘ │
│  │              │   │              │          │                │          │
│  │  VISION      │   │  NAVIGATION  │          └──────► EKF 融合 ◄──┘          │
│  │  PIPELINE    │   │  PIPELINE    │                 │                       │
│  │              │   │              │            ┌────▼────┐                 │
│  │  ┌────────┐  │   │  ┌────────┐  │            │/odom   │                 │
│  │  │OpenCV  │  │   │  │SLAM    │  │            │ /tf   │                 │
│  │  │预处理  │  │   │  │建图    │  │            └────┬────┘                 │
│  │  └────┬───┘  │   │  └────┬───┘  │                 │                       │
│  │  ┌────▼───┐  │   │  ┌────▼───┐  │            ┌────▼────┐                 │
│  │  │YOLOv8  │  │   │  │AMCL    │  │            │Nav2     │                 │
│  │  │检测    │  │   │  │定位    │  │            │Planner  │                 │
│  │  └────┬───┘  │   │  └────┬───┘  │            └────┬────┘                 │
│  │  ┌────▼───┐  │   │  ┌────▼───┐  │                 │                       │
│  │  │Depth   │  │   │  │Planner │  │            ┌────▼────┐                 │
│  │  │Process │  │   │  │路径规划 │  │            │/cmd_vel │                 │
│  │  └────┬───┘  │   │  └────┬───┘  │            └────┬────┘                 │
│  │       │      │   │       │      │                 │                       │
│  │  ┌────▼────┐ │   │  ┌────▼────┐ │                 │                       │
│  │  │Target   │ │   │  │Virtual │  │                 │                       │
│  │  │3D Pose  │ │   │  │Walls   │  │                 │                       │
│  │  └────┬────┘ │   │  └────┬────┘ │                 │                       │
│  │       │      │   │       │      │                 │                       │
│  └───────┼──────┘   └───────┼──────┘                 │                       │
│          │                   │                        │                       │
│  ┌───────▼───────────────────▼────────────────────────┘                       │
│  │                        POINTCLOUD PIPELINE                                  │
│  │  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐               │
│  │  │PCD     │  │RANSAC  │  │Euclidean│  │OBB     │  │Cylinder│               │
│  │  │Saver   │  │Plane   │  │Cluster │  │Analyzer│  │Convert │               │
│  │  │        │  │Seg     │  │        │  │        │  │        │               │
│  │  └────┬───┘  └────┬───┘  └────┬───┘  └────┬───┘  └────┬───┘               │
│  │       │           │           │           │           │                   │
│  │       └───────────┴───────────┴───────────┴───────────┘                   │
│  │                                      │                                      │
│  │                              ┌───────▼───────┐                             │
│  │                              │ Serial Send   │                             │
│  │                              │ (Arm Control) │                             │
│  │                              └───────────────┘                             │
│  └────────────────────────────────────────────────────────────────────────────┘
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 文档索引

| 文档 | 内容 |
|------|------|
| [01-architecture](docs/01-architecture.md) | 系统架构总览与设计理念 |
| [02-workspace-integration](docs/02-workspace-integration.md) | 三个工作空间的物理/逻辑整合方案 |
| [03-vision-pipeline](docs/03-vision-pipeline.md) | Astra 视觉识别管道详细设计 |
| [04-pointcloud-pipeline](docs/04-pointcloud-pipeline.md) | 点云目标定向识别管道设计 |
| [05-navigation-pipeline](docs/05-navigation-pipeline.md) | Nav2 导航管道设计 |
| [06-virtual-walls](docs/06-virtual-walls.md) | 虚拟墙采集、应用、管理方案 |
| [07-topic-topology](docs/07-topic-topology.md) | 完整 ROS2 话题拓扑图 |
| [08-launch-sequence](docs/08-launch-sequence.md) | 各模式启动流程与 launch 文件 |
| [09-modification-checklist](docs/09-modification-checklist.md) | 代码修改清单与注意事项 |
| [10-deployment-guide](docs/10-deployment-guide.md) | 部署、测试、调试指南 |

---

## 贡献与维护

- 作者: serberb
- 创建日期: 2026-07-02
- 许可证: MIT
