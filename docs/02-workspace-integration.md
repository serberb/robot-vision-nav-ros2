# 02 - 工作空间整合方案

## 现状分析

当前有三个独立工作空间，各自独立编译、独立运行，存在以下问题：

| 问题 | 影响 | 解决方案 |
|------|------|----------|
| 编译冲突 | colcon 构建可能因依赖关系失败 | 统一 src 目录，明确 package.xml 依赖 |
| 话题命名不统一 | 节点间无法通信 | 统一话题命名规范 |
| launch 分散 | 需要手动启动多个终端 | 创建整合级 bringup 包 |
| 配置文件分散 | 参数管理困难 | 集中配置到 bringup/config |
| 坐标系未标定 | 视觉和导航结果不在同一坐标系 | 添加 static_transform_publisher |
| 虚拟墙脚本独立 | 无法自动注入地图 | 整合到 launch 流程 |

---

## 整合方案

### 方案 A: 合并为单一工作空间（推荐）

将所有包迁移到统一工作空间 `~/robot_ws/src/`，利用 ROS2 的 overlay 机制：

```bash
~/robot_ws/
├── src/
│   ├── astra_camera/              # C++ 驱动包
│   ├── astra_camera_msgs/         # 相机消息定义
│   ├── robot_vision/              # Python 视觉处理
│   ├── robot_vision_msgs/         # 视觉消息定义
│   ├── my_pcl_pkg/                # C++ 点云处理
│   ├── my_robot_base/             # Python 底盘驱动
│   ├── my_robot_sensors/          # Python 传感器节点
│   ├── my_robot_bringup/          # Python 启动配置（原）
│   ├── sllidar_ros2/              # C++ 激光雷达
│   └── robot_bringup/             # **新增: 整合级启动包**
│       ├── config/                # 所有配置文件
│       ├── launch/                # 所有 launch 文件
│       ├── maps/                  # 地图文件
│       ├── scripts/               # 工具脚本
│       └── package.xml
└── build/ install/ log/
```

**优点**: 一次编译，依赖自动解析，启动简单  
**缺点**: 包数量多，编译时间可能较长（可 `--packages-select` 部分编译）

### 方案 B: 保持多工作空间 + overlay

```bash
# 基础层: 官方包
/opt/ros/humble/

# 中间层: 传感器驱动
~/astra_ws/install/          # astra_camera, sllidar_ros2

# 中间层: 算法包
~/algo_ws/install/           # robot_vision, my_pcl_pkg

# 顶层: 整合与配置
~/nav_ws/install/            # my_robot_bringup, robot_bringup
```

**优点**: 各层独立编译，底层改动不影响上层  
**缺点**: 环境变量管理复杂，容易 source 顺序错误

---

## 推荐整合步骤

### Step 1: 创建统一工作空间

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# 从原 astra_ws 迁移
cp -r /path/to/astra_ws/src/ros2_astra_camera .
cp -r /path/to/astra_ws/src/my_vision/robot_vision .
cp -r /path/to/astra_ws/src/my_vision/robot_vision_msgs .

# 从原 ros2_ws 迁移
cp -r /path/to/ros2_ws/src/my_pcl_pkg .

# 从原 slam_ws 迁移
cp -r /path/to/slam_ws/src/my_robot_bringup .
cp -r /path/to/slam_ws/src/my_robot_base .
cp -r /path/to/slam_ws/src/my_robot_sensors .
cp -r /path/to/slam_ws/src/sllidar_ros2 .

# 创建整合包
cd ~/robot_ws/src
mkdir -p robot_bringup/config robot_bringup/launch robot_bringup/maps robot_bringup/scripts
```

### Step 2: 修改 package.xml 依赖

确保每个包的 `package.xml` 正确声明依赖：

**robot_vision/package.xml** 添加：
```xml
<depend>astra_camera</depend>
<depend>robot_vision_msgs</depend>
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>message_filters</depend>
```

**my_pcl_pkg/package.xml** 添加：
```xml
<depend>astra_camera</depend>
<depend>visualization_msgs</depend>
<depend>geometry_msgs</depend>
```

**robot_bringup/package.xml**（新建）:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>robot_bringup</name>
  <version>1.0.0</version>
  <description>整合级启动包</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>astra_camera</exec_depend>
  <exec_depend>robot_vision</exec_depend>
  <exec_depend>my_pcl_pkg</exec_depend>
  <exec_depend>my_robot_base</exec_depend>
  <exec_depend>my_robot_sensors</exec_depend>
  <exec_depend>sllidar_ros2</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>robot_localization</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 3: 创建 setup.py

**robot_bringup/setup.py**:
```python
from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Robot bringup package for unified launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
```

### Step 4: 编译

```bash
cd ~/robot_ws
colcon build --symlink-install

# 部分编译（开发时）
colcon build --symlink-install --packages-select robot_bringup

# 环境设置
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 配置集中管理

将所有配置文件放入 `robot_bringup/config/`：

```
robot_bringup/config/
├── ekf.yaml                          # EKF 融合参数
├── nav2_params.yaml                  # Nav2 最终参数
├── slam_toolbox.yaml                 # SLAM 建图参数
├── astra_camera.yaml                 # Astra 相机参数
├── sllidar.yaml                      # 激光雷达参数
├── base_serial.yaml                  # 底盘串口参数
└── virtual_walls/
    ├── default_walls.json            # 默认虚拟墙
    └── map_inject.sh                 # 地图注入脚本
```

在 launch 中通过 `os.path.join(get_package_share_directory('robot_bringup'), 'config', 'xxx.yaml')` 引用。

---

## 脚本集中管理

将根目录工具脚本放入 `robot_bringup/scripts/`：

```
robot_bringup/scripts/
├── arrow_teleop.py                   # 键盘遥控（已有）
├── scan_retime.py                    # 激光时间戳重对齐（已有）
├── print_odom_pose.py               # 打印里程计（已有）
├── collect_wall_points.py           # 虚拟墙采集（已有）
├── apply_virtual_walls.py           # 虚拟墙应用（已有）
├── add_multi_virtual_walls.py       # 批量虚拟墙（已有）
├── fix_map_with_walls.py            # 地图修复（已有）
├── auto_inject_walls.py             # **新增**: 自动注入虚拟墙到地图
└── switch_mode.py                   # **新增**: 模式切换脚本
```

---

## 地图管理

```
robot_bringup/maps/
├── raw/                              # 原始地图（SLAM 输出）
│   ├── map.pgm
│   └── map.yaml
├── blocked/                          # 带虚拟墙的地图
│   ├── map_blocked.pgm
│   └── map_blocked.yaml
└── virtual_walls_points.json         # 虚拟墙坐标定义
```

启动流程中自动检查：如果 `blocked/` 不存在，则调用 `apply_virtual_walls.py` 生成。

---

## 版本控制策略

建议将各子包作为 Git submodule 管理，或独立仓库 + 统一 manifest：

```
robot-vision-nav-ros2/              # 整合仓库（本仓库）
├── .gitmodules
├── README.md
├── docs/
├── robot_bringup/                  # 整合包（主仓库）
│   ├── config/
│   ├── launch/
│   └── scripts/
├── src/                            # 子包（submodule 或独立）
│   ├── astra_camera -> git@github.com:xxx/ros2_astra_camera.git
│   ├── robot_vision -> git@github.com:xxx/robot_vision.git
│   ├── my_pcl_pkg -> git@github.com:xxx/my_pcl_pkg.git
│   └── ...
└── docker/
    └── Dockerfile
```

这样各子包可以独立演进，整合仓库只维护 launch、config、docs。
