# 01 - 系统架构总览

## 设计目标

将三个独立演进的 ROS2 工作空间整合为统一的、可编排的机器人工程项目，核心要求：

1. **模块化启动**: 各子系统可独立运行，也可一键全部启动
2. **话题一致性**: 统一话题命名空间，避免冲突
3. **坐标系统一**: 所有传感器输出统一到 `odom` → `base_link` → `camera_link` 坐标链
4. **可扩展性**: 新传感器/算法可以即插即用
5. **可维护性**: 配置文件集中管理，脚本工具化

---

## 三大子系统

### 1. 视觉识别子系统 (Vision Pipeline)

**核心能力**: RGB-D 目标检测 → 2D 检测框 → 3D 世界坐标 → 目标位姿

**现有组件**:
- `astra_camera` — Astra Pro 深度相机驱动
- `opencv_preprocess_node` — RGB+深度同步预处理
- `yolo_detect_node` — YOLOv8 目标检测（自定义模型 best.pt）
- `depth_process_node` — 深度反投影计算 3D 坐标
- `visualization_node` — 检测结果可视化
- `gripper_control_node` — 简单抓手控制逻辑

**关键问题**:
- `camera_node.py` 当前读取的是本地视频文件，**需要改为直接订阅 `/camera/color/image_raw`**
- 深度图与 RGB 图的分辨率差异需要处理（原图通常 640×480 或 1280×720，YOLO 输入 640×640）

### 2. 点云目标定向识别子系统 (PointCloud Pipeline)

**核心能力**: 原始点云 → 平面分割 → 障碍物提取 → 欧式聚类 → OBB 分析 → 柱坐标转换 → 机械臂控制

**现有组件**:
- `my_pcl_node` — 点云保存（调试用）
- `pcd_processor_node` — RANSAC 平面分割 + 欧式聚类 + 彩色点云
- `obb_analyzer_node` — PCA 计算 OBB 有向包围盒，判定规则长方体
- `obb_to_cylindrical_node` — 坐标变换到机械臂基座系，笛卡尔转柱坐标
- `serial_send_node` — 串口发送固定柱坐标（当前为固定值，需改为动态）

**关键问题**:
- `serial_send_node` 当前只发送**固定坐标**，需要改为订阅 `/selected_target_pose` 动态发送
- 点云坐标系需要从 `camera_depth_optical_frame` 转换到 `base_link`

### 3. 导航子系统 (Navigation Pipeline)

**核心能力**: 激光雷达 + IMU + 轮式里程计 → EKF 融合 → SLAM/AMCL → Nav2 路径规划 → 底盘控制

**现有组件**:
- `sllidar_ros2` — 思岚激光雷达驱动
- `mpu6050_node` — IMU 传感器
- `base_serial_node` — 麦轮底盘串口驱动 + 正逆运动学 + 里程计
- `scan_retime` — 激光时间戳重对齐（解决时间同步问题）
- `ekf.yaml` — EKF 状态估计融合
- `slam_toolbox.yaml` — SLAM 建图
- `nav2_params.yaml` — Nav2 导航参数

**关键问题**:
- 多版本 `nav2_params` 文件需要整合为最终版本
- 虚拟墙需要在地图加载前注入
- 激光雷达与底盘坐标系需要精确标定

---

## 整合策略

### 物理整合: 统一工作空间

```bash
~/robot_ws/
├── src/
│   ├── astra_camera/           # 从 astra_ws 迁移
│   ├── astra_camera_msgs/    # 从 astra_ws 迁移
│   ├── robot_vision/           # 从 astra_ws 迁移
│   ├── robot_vision_msgs/    # 从 astra_ws 迁移
│   ├── my_pcl_pkg/             # 从 ros2_ws 迁移
│   ├── my_robot_bringup/     # 从 slam_ws 迁移
│   ├── my_robot_base/        # 从 slam_ws 迁移
│   ├── my_robot_sensors/     # 从 slam_ws 迁移
│   ├── sllidar_ros2/         # 从 slam_ws 迁移
│   └── robot_bringup/        # 新建: 整合级 launch 包
└── build/ install/ log/
```

### 逻辑整合: 统一命名空间

| 层级 | 命名空间 | 说明 |
|------|----------|------|
| 传感器层 | `/camera/*` | Astra 相机所有话题 |
| 传感器层 | `/scan` | 激光雷达 |
| 传感器层 | `/imu` | IMU 数据 |
| 传感器层 | `/wheel/odom` | 轮式里程计 |
| 视觉层 | `/vision/*` | 视觉处理中间结果 |
| 点云层 | `/pcl/*` | 点云处理中间结果 |
| 导航层 | `/nav/*` | Nav2 相关话题 |
| 控制层 | `/cmd_vel` | 底盘速度指令 |
| 控制层 | `/gripper/*` | 抓手控制 |

### 坐标系整合

```
world
 └── odom
      └── base_link
           ├── base_scan (激光雷达)
           ├── camera_link
           │    └── camera_depth_optical_frame
           │         └── camera_color_optical_frame
           └── arm_base_link (机械臂基座)
```

---

## 数据流总览

```
┌─────────────────────────────────────────────────────────────────┐
│                        传感器数据层                                │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐            │
│  │/camera/ │  │ /scan   │  │ /imu    │  │/wheel/  │            │
│  │color/...│  │         │  │         │  │odom     │            │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘            │
└───────┼────────────┼────────────┼────────────┼──────────────────┘
        │            │            │            │
        ▼            ▼            │            │
┌──────────────┐  ┌──────────────┐  │            │
│  VISION      │  │  scan_retime │  │            │
│  PIPELINE    │  │  (时间对齐)   │  │            │
│              │  └──────┬───────┘  │            │
│  ┌────────┐  │         │           │            │
│  │/processed│  │         │           │            │
│  │/rgb    │  │         │           │            │
│  └────┬───┘  │         │           │            │
│  ┌────▼───┐  │         │           │            │
│  │/yolo_  │  │         │           │            │
│  │detection│  │         │           │            │
│  └────┬───┘  │         │           │            │
│  ┌────▼───┐  │         ▼           ▼            ▼
│  │/target_│  │    ┌─────────────────────────────────┐
│  │world_  │  │    │          EKF 融合               │
│  │point  │  │    │     (robot_localization)         │
│  └────────┘  │    └────────────┬────────────────────┘
└──────────────┘                 │
        │                        │
        │                        ▼
        │              ┌──────────────────┐
        │              │    /odom (融合)    │
        │              │    /tf (odom→base) │
        │              └────────┬─────────┘
        │                       │
        │         ┌─────────────┼─────────────┐
        │         ▼             ▼             ▼
        │   ┌──────────┐  ┌──────────┐  ┌──────────┐
        │   │SLAM建图  │  │AMCL定位  │  │Nav2规划  │
        │   │(slam_tool)│  │          │  │          │
        │   └────┬─────┘  └────┬─────┘  └────┬─────┘
        │        │            │            │
        │        ▼            ▼            ▼
        │   ┌─────────────────────────────────────┐
        │   │            虚拟墙注入                  │
        │   │    (地图PGM上绘制黑色障碍物)           │
        │   └─────────────────────────────────────┘
        │                       │
        │                       ▼
        │              ┌──────────────────┐
        │              │    /cmd_vel      │
        │              │    (底盘控制)     │
        │              └────────┬─────────┘
        │                       │
        ▼                       ▼
┌──────────────────────────────────────────────────┐
│              POINTCLOUD PIPELINE                  │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐          │
│  │/camera/ │  │/processed│  │/obb_   │          │
│  │depth/   │  │_cloud   │  │markers │          │
│  │points   │  │         │  │         │          │
│  └────┬────┘  └────┬────┘  └────┬────┘          │
│       │            │            │               │
│       └────────────┴────────────┘               │
│                    │                            │
│                    ▼                            │
│           ┌────────────────┐                    │
│           │/selected_target│                    │
│           │_pose           │                    │
│           └───────┬────────┘                    │
│                   │                             │
│                   ▼                             │
│           ┌────────────────┐                    │
│           │ Serial Send    │                    │
│           │ (Arm Control)  │                    │
│           └────────────────┘                    │
└──────────────────────────────────────────────────┘
```

---

## 设计原则

1. **不要重复造轮子**: 优先复用现有代码，只修改必要部分
2. **话题解耦**: 各节点通过话题通信，降低耦合度
3. **配置外化**: 参数全部放入 YAML，不硬编码在代码中
4. **可视化友好**: 所有中间结果都发布 Marker/Image，便于 RViz 调试
5. **容错设计**: 传感器缺失时系统优雅降级，不崩溃

---

## 技术栈

| 组件 | 技术 | 版本 |
|------|------|------|
| OS | Ubuntu | 22.04 LTS |
| ROS | ROS2 | Humble Hawksbill |
| 相机驱动 | ros2_astra_camera | 最新 |
| 激光雷达 | sllidar_ros2 | 最新 |
| 视觉检测 | YOLOv8 (ultralytics) | 8.x |
| 点云处理 | PCL | 1.12 |
| 导航 | Nav2 | 1.1.x |
| SLAM | slam_toolbox | 2.x |
| 状态估计 | robot_localization (EKF) | 3.x |
| 机械臂通信 | 串口 (Python serial / C++ termios) | - |
