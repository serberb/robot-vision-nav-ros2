# 08 - 启动流程与 Launch 文件

## 启动模式总览

| 模式 | 用途 | 启动文件 | 说明 |
|------|------|----------|------|
| **全部启动** | 完整系统运行 | `bringup_all.launch.py` | 视觉+点云+导航 |
| **仅视觉** | 调试视觉算法 | `vision_only.launch.py` | 不启动导航 |
| **仅点云** | 调试点云算法 | `pointcloud_only.launch.py` | 不启动导航 |
| **仅导航** | 调试导航算法 | `navigation_only.launch.py` | 不启动视觉 |
| **SLAM 建图** | 构建新地图 | `slam_mapping.launch.py` | 需要键盘遥控 |
| **自主导航** | 已有地图导航 | `nav2_autonomous.launch.py` | 加载带虚拟墙的地图 |

---

## 一键启动全部

```python
# bringup_all.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    
    return LaunchDescription([
        # 1. 视觉管道
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_dir, 'launch', 'vision_only.launch.py')
            )
        ),
        # 2. 点云管道
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_dir, 'launch', 'pointcloud_only.launch.py')
            )
        ),
        # 3. 导航管道
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_dir, 'launch', 'navigation_only.launch.py')
            )
        ),
    ])
```

---

## SLAM 建图流程

### 启动命令

```bash
# 终端 1: 启动 SLAM 建图
ros2 launch robot_bringup slam_mapping.launch.py

# 终端 2: 启动键盘遥控
python3 ~/robot_ws/src/robot_bringup/scripts/arrow_teleop.py

# 终端 3: 监控里程计
python3 ~/robot_ws/src/robot_bringup/scripts/print_odom_pose.py
```

### 建图注意事项

1. **移动速度**: 建议 linear=0.20, angular=0.32（arrow_teleop 默认值）
2. **覆盖范围**: 确保所有需要导航的区域都被扫描到
3. **闭环**: 如果可能，回到起点完成闭环，提高地图精度
4. **保存地图**:
   ```bash
   # 服务方式保存
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
     "name: {data: '/home/ubuntu/maps/map_$(date +%Y%m%d_%H%M%S)'}"
   
   # 或者使用 map_saver
   ros2 run nav2_map_server map_saver_cli -f ~/maps/map
   ```

### slam_mapping.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    
    return LaunchDescription([
        # 1. 激光雷达
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser',
                'angle_compensate': True
            }]
        ),
        # 2. scan_retime
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
        # 5. EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(robot_bringup_dir, 'config', 'ekf.yaml')]
        ),
        # 6. TF 静态变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.15', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        ),
        # 7. SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 
                            'launch', 'online_sync_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': os.path.join(robot_bringup_dir, 'config', 'slam_toolbox.yaml')
            }.items()
        ),
        # 8. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(robot_bringup_dir, 'config', 'slam.rviz')]
        ),
    ])
```

---

## 自主导航流程

### 启动命令

```bash
# 终端 1: 启动自主导航
ros2 launch robot_bringup nav2_autonomous.launch.py

# 终端 2: RViz 中设置目标点
ros2 run rviz2 rviz2 -d ~/robot_ws/src/robot_bringup/config/nav2.rviz

# 终端 2: 或者使用命令行发送目标点
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 注意事项

1. **初始位姿**: 启动后需要在 RViz 中点击 "2D Pose Estimate" 设置初始位姿
2. **虚拟墙**: 确保加载的是带虚拟墙的地图 (`map_blocked.yaml`)
3. **代价地图**: 首次启动时可能需要等待代价地图初始化（约 10-30 秒）

### nav2_autonomous.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

def generate_launch_description():
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    map_yaml = os.path.join(robot_bringup_dir, 'maps', 'map_blocked.yaml')
    
    # 检查带虚拟墙的地图是否存在
    if not Path(map_yaml).exists():
        # 尝试从原始地图生成
        orig_map = os.path.join(robot_bringup_dir, 'maps', 'map.yaml')
        auto_inject = ExecuteProcess(
            cmd=['python3', 
                 os.path.join(robot_bringup_dir, 'scripts', 'auto_inject_walls.py'),
                 '--map-yaml', orig_map],
            output='screen'
        )
    else:
        auto_inject = []
    
    return LaunchDescription([
        auto_inject,
        # 1. 激光雷达
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}]
        ),
        # 2. scan_retime
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
        # 5. EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(robot_bringup_dir, 'config', 'ekf.yaml')]
        ),
        # 6. TF 静态变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.15', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        ),
        # 7. Map Server (加载带虚拟墙的地图)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': map_yaml
            }]
        ),
        # 8. AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[os.path.join(robot_bringup_dir, 'config', 'nav2_params_final.yaml')]
        ),
        # 9. Nav2 (Planner + Controller + BT Navigator + Recovery)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(robot_bringup_dir, 'config', 'nav2_params_final.yaml'),
                'autostart': 'true'
            }.items()
        ),
        # 10. Lifecycle Manager (自动启动所有 Nav2 节点)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl', 'planner_server', 
                              'controller_server', 'recoveries_server', 
                              'bt_navigator', 'waypoint_follower']
            }]
        ),
    ])
```

---

## 启动时序图

```
时间轴 ──────────────────────────────────────────────────────────────────────>

[0s]     ┌──────────┐
         │ 硬件初始化 │
         │ 相机/雷达/ │
         │ 底盘/IMU  │
         └────┬─────┘
              │
[2s]         ▼
         ┌──────────┐
         │ 传感器驱动 │
         │ 节点启动   │
         │ (astra,   │
         │  sllidar,│
         │  base,    │
         │  mpu6050) │
         └────┬─────┘
              │
[3s]         ▼
         ┌──────────┐
         │ scan_retime│
         │ TF 静态发布│
         └────┬─────┘
              │
[4s]         ▼
         ┌──────────┐
         │ EKF 启动  │
         │ (/odom)   │
         └────┬─────┘
              │
[5s]         ▼
         ┌──────────┐     ┌──────────┐
         │ SLAM建图  │ 或  │ AMCL +   │
         │ (slam_   │     │ Nav2     │
         │  toolbox) │     │ 启动     │
         └────┬─────┘     └────┬─────┘
              │                │
[6s]         ▼                ▼
         ┌──────────────────────┐
         │ 视觉/点云节点启动      │
         │ (vision + pcl)       │
         └──────────────────────┘
              │
[10s]        ▼
         ┌──────────────────────┐
         │ 系统就绪，等待指令     │
         │ · 建图: 键盘遥控       │
         │ · 导航: RViz 设置目标点 │
         │ · 视觉: 自动检测目标    │
         │ · 点云: 自动识别箱子    │
         └──────────────────────┘
```
