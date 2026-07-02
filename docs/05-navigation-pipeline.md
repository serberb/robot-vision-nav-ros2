# 05 - 导航系统设计 (Navigation Pipeline)

## 设计目标

基于激光雷达 + IMU + 轮式里程计，构建完整的自主导航能力：
1. **SLAM 建图**: 使用键盘遥控建图，支持虚拟墙注入
2. **自主定位**: AMCL 蒙特卡洛定位
3. **路径规划**: Nav2 全局 + 局部规划器
4. **底盘控制**: 麦轮底盘正逆运动学 + 串口通信
5. **状态估计**: EKF 融合多传感器位姿

---

## 现有组件分析

### 1. base_serial_node.py (底盘驱动)

**功能**: 麦轮底盘串口通信 + 正逆运动学 + 里程计发布

**硬件接口**:
- 串口: `/dev/serial0` @ 115200 baud
- 协议: 自定义二进制帧 (HEADER="ODOM4")
- 发送: ASCII 指令 `CMD4,<seq>,<lf>,<rf>,<lr>,<rr>\r\n`

**运动学模型**:
```
# 逆运动学 (cmd_vel → 各轮速度)
lf = vx + vy - q      # 左前
rf = vx - vy - q      # 右前
lr = vx + vy + q      # 左后
rr = -vx + vy - q     # 右后
其中 q = k * wz, k = lx + ly

# 正运动学 (编码器增量 → 位姿增量)
d_vx = (d_lf + d_rf + d_lr - d_rr) * 0.25
d_vy = (d_lf - d_rf + d_lr + d_rr) * 0.25
dth = ((-d_lf - d_rf + d_lr - d_rr) / (4*k)) * yaw_odom_scale
```

**参数**:
| 参数 | 值 | 说明 |
|------|-----|------|
| wheel_radius | 0.05m | 轮半径 |
| wheel_base_x | 0.176m | 轮距X |
| wheel_separation_y | 0.2986m | 轮距Y |
| ticks_per_rev | 60400 | 每转编码器计数 |
| yaw_odom_scale | 0.97 | 航向角修正因子 |
| max_wheel_mps | 0.40 m/s | 最大轮速 |

**发布**:
- `/wheel/odom` (Odometry): 轮式里程计
- 订阅 `/cmd_vel` (Twist): 速度指令

**注意**: 坐标系定义特殊 —— `+X` 是原来的车尾方向（前进方向），这是因为底盘安装时车头方向反转。

### 2. mpu6050_node.py (IMU)

**功能**: MPU6050 IMU 数据读取

**发布**:
- `/imu` (sensor_msgs/Imu): 角速度 + 线性加速度

**注意**: 需要与底盘编码器里程计做 EKF 融合，因为单独轮式里程计在打滑时误差大。

### 3. sllidar_ros2 (激光雷达)

**功能**: 思岚激光雷达驱动

**发布**:
- `/scan` (sensor_msgs/LaserScan): 激光扫描数据

**注意**: `scan_retime.py` 用于解决时间戳不同步问题。因为激光雷达和底盘/IMU 的时间戳可能来自不同时钟源，Nav2 中的时间同步检查会报错。`scan_retime` 将 `header.stamp` 替换为当前 ROS 时间，确保时间同步。

### 4. EKF 融合 (robot_localization)

**配置文件**: `ekf.yaml`

**输入**:
- `/wheel/odom`: 轮式里程计 (X, Y, yaw 速度)
- `/imu`: IMU (角速度, 线性加速度)

**输出**:
- `/odom`: 融合后的位姿
- `/tf`: `odom → base_link` 变换

**建议配置**:
```yaml
# ekf.yaml 关键参数
odom0: /wheel/odom
odom0_config: [false, false, false,   # X, Y, Z position
               false, false, false,   # roll, pitch, yaw
               true, true, false,    # X, Y, Z velocity
               false, false, true,   # roll, pitch, yaw velocity
               false, false, false]  # X, Y, Z acceleration

imu0: /imu
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              true, true, true,      # 角速度
              true, true, false]     # 线性加速度 (X, Y)

# 如果只使用 2D 导航，Z、roll、pitch 设为 false
```

### 5. SLAM Toolbox

**配置文件**: `slam_toolbox.yaml`

**功能**: 2D 激光 SLAM 建图

**模式**:
- **在线同步建图** (online_sync): 实时建图，适合小场景
- **在线异步建图** (online_async): 适合大场景，可保存/加载地图

**输入**:
- `/scan`: 激光扫描
- `/tf`: 坐标变换

**输出**:
- `/map`: 地图
- `/tf`: `map → odom` 变换

### 6. Nav2

**配置文件**: `nav2_params.yaml`（多版本需要整合）

**核心组件**:
| 组件 | 功能 | 参数要点 |
|------|------|----------|
| AMCL | 粒子滤波定位 | 粒子数、激光模型参数 |
| Planner | 全局路径规划 | 使用 A* 或 NavFn |
| Controller | 局部轨迹跟踪 | DWB (Dynamic Window Approach) |
| BT Navigator | 行为树导航 | 导航行为编排 |
| Recovery | 恢复行为 | 旋转恢复、清除代价地图 |

**现有参数版本分析**:
当前有 10+ 个版本的 `nav2_params`，分别针对不同场景调优：
- `nav2_params_before_corner_safe.yaml`: 转角安全
- `nav2_params_before_door_tuning.yaml`: 窄门调优
- `nav2_params_before_fast_forward.yaml`: 快速前进
- `nav2_params_before_narrow_door_balance.yaml`: 窄门平衡

**整合方案**: 创建一个最终版本 `nav2_params_final.yaml`，吸收各版本的优点。

---

## 导航管道数据流

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                          Navigation Pipeline                                  │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌────────────┐  ┌────────────┐  ┌────────────────────────────────────┐   │
│  │ /scan      │  │ /imu       │  │ /wheel/odom                        │   │
│  │(LaserScan) │  │(Imu)       │  │(Odometry)                          │   │
│  └──────┬─────┘  └──────┬─────┘  └────────────┬───────────────────────┘   │
│         │               │                     │                          │
│         │               │                     │                          │
│         │         ┌─────┴─────┐               │                          │
│         │         │           │               │                          │
│         │         ▼           ▼               │                          │
│         │  ┌─────────────────────────────────┐│                          │
│         │  │  EKF (robot_localization)       ││                          │
│         │  │  · 融合 wheel_odom + imu        ││                          │
│         │  │  · 输出 /odom (融合位姿)        ││                          │
│         │  │  · 输出 /tf (odom → base_link)  ││                          │
│         │  └──────────────┬──────────────────┘│                          │
│         │                 │                   │                          │
│         │                 │                   │                          │
│         ▼                 ▼                   │                          │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │  SLAM Mode (建图)                                                    │ │
│  │  ┌──────────────────────────┐                                       │ │
│  │  │ slam_toolbox (online_sync)│                                       │ │
│  │  │ · /scan + /tf → /map       │                                       │ │
│  │  │ · 保存 map.pgm + map.yaml  │                                       │ │
│  │  └──────────────┬───────────┘                                       │ │
│  │                 │                                                    │ │
│  │                 ▼                                                    │ │
│  │  ┌──────────────────────────┐                                       │ │
│  │  │ 虚拟墙注入                 │                                       │ │
│  │  │ · collect_wall_points.py │                                       │ │
│  │  │ · apply_virtual_walls.py │                                       │ │
│  │  └──────────────┬───────────┘                                       │ │
│  │                 │                                                    │ │
│  │                 ▼                                                    │ │
│  │  ┌──────────────────────────┐                                       │ │
│  │  │ map_blocked.pgm          │                                       │ │
│  │  │ map_blocked.yaml         │                                       │ │
│  │  └──────────────────────────┘                                       │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                               │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │  Navigation Mode (自主导航)                                         │ │
│  │  ┌──────────────────────────┐                                       │ │
│  │  │ AMCL                     │                                       │ │
│  │  │ · /scan + /map → 定位     │                                       │ │
│  │  │ · 输出 /amcl_pose         │                                       │ │
│  │  └──────────────┬───────────┘                                       │ │
│  │                 │                                                    │ │
│  │                 ▼                                                    │ │
│  │  ┌──────────────────────────┐                                       │ │
│  │  │ Nav2 Planner             │                                       │ │
│  │  │ · 全局: A* / NavFn        │                                       │ │
│  │  │ · 局部: DWB               │                                       │ │
│  │  │ · 输入: /goal_pose        │                                       │ │
│  │  │ · 输出: /cmd_vel          │                                       │ │
│  │  └──────────────────────────┘                                       │ │
│  │                 │                                                    │ │
│  │                 ▼                                                    │ │
│  │  ┌──────────────────────────┐                                       │ │
│  │  │ Recovery Behaviors       │                                       │ │
│  │  │ · 旋转恢复               │                                       │ │
│  │  │ · 清除代价地图           │                                       │ │
│  │  │ · 退后重试               │                                       │ │
│  │  └──────────────────────────┘                                       │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                               │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │  Control Layer                                                       │ │
│  │  ┌──────────────────────────┐                                       │ │
│  │  │ /cmd_vel (Twist)         │                                       │ │
│  │  │  · linear.x, linear.y    │  ← 麦轮支持全向移动                    │ │
│  │  │  · angular.z             │                                       │ │
│  │  └──────────────┬───────────┘                                       │ │
│  │                 │                                                    │ │
│  │                 ▼                                                    │ │
│  │  ┌──────────────────────────┐                                       │ │
│  │  │ base_serial_node         │                                       │ │
│  │  │  · 逆运动学 → 四轮速度    │                                       │ │
│  │  │  · 串口发送 CMD4 指令     │                                       │ │
│  │  │  · 0.5s 超时保护 → 停车   │                                       │ │
│  │  └──────────────────────────┘                                       │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                               │
└───────────────────────────────────────────────────────────────────────────────┘
```

---

## 修改清单 (Navigation)

| 文件 | 修改内容 | 优先级 |
|------|----------|--------|
| `ekf.yaml` | 确认融合参数正确 | 高 |
| `nav2_params.yaml` | 整合多版本为最终版 | 高 |
| `slam_toolbox.yaml` | 确认模式 (online_sync/async) | 中 |
| `base_serial_node.py` | 确认坐标系方向正确 | 高 |
| `scan_retime.py` | 集成到 launch（自动启动） | 中 |
| `arrow_teleop.py` | 确认坐标系方向一致 | 高 |

---

## Nav2 最终参数建议

```yaml
# nav2_params_final.yaml 核心参数
amcl:
  ros__parameters:
    use_sim_time: false
    min_particles: 500
    max_particles: 2000
    laser_min_range: 0.15
    laser_max_range: 12.0
    update_min_a: 0.2
    update_min_d: 0.25

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: navigate_w_replanning_and_recovery.xml

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: progress_checker
    goal_checker_plugin: goal_checker
    controller_plugins: [FollowPath]
    FollowPath:
      plugin: dwb_core::DWBLocalPlanner
      debug_trajectory_details: true
      min_vel_x: 0.0
      max_vel_x: 0.35
      min_vel_y: -0.2
      max_vel_y: 0.2
      max_vel_theta: 0.8
      acc_lim_x: 0.5
      acc_lim_y: 0.3
      acc_lim_theta: 1.0
      sim_time: 2.0
      vx_samples: 6
      vy_samples: 6
      vtheta_samples: 20

planner_server:
  ros__parameters:
    use_sim_time: false
    planner_plugins: [GridBased]
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

recovery_server:
  ros__parameters:
    use_sim_time: false
    recovery_plugins: [spin, backup, wait]
    spin:
      plugin: nav2_recoveries/Spin
    backup:
      plugin: nav2_recoveries/BackUp
    wait:
      plugin: nav2_recoveries/Wait
```

**注意**: 麦轮底盘支持全向移动，所以 `min_vel_y` 和 `max_vel_y` 不能设为 0，否则机器人无法横向移动。

---

## 导航模块 Launch 文件

```python
# navigation_only.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    
    return LaunchDescription([
        # 1. 激光雷达
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}]
        ),
        # 2. scan_retime (时间戳重对齐)
        Node(
            package='robot_bringup',
            executable='scan_retime.py',
            name='scan_retime'
        ),
        # 3. IMU
        Node(
            package='my_robot_sensors',
            executable='mpu6050_node',
            name='mpu6050'
        ),
        # 4. 底盘
        Node(
            package='my_robot_base',
            executable='base_serial_node',
            name='base_serial',
            parameters=[os.path.join(robot_bringup_dir, 'config', 'base_serial.yaml')]
        ),
        # 5. EKF 融合
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(robot_bringup_dir, 'config', 'ekf.yaml')]
        ),
        # 6. TF 静态变换 (激光雷达、相机等)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.15', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        ),
        # 7. Nav2 (AMCL + Planner + Controller)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(robot_bringup_dir, 'config', 'nav2_params_final.yaml'),
                'map': os.path.join(robot_bringup_dir, 'maps', 'map_blocked.yaml')
            }.items()
        ),
    ])
```
