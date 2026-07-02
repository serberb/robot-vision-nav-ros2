# 10 - 部署与运行指南

## 环境准备

### 系统要求

```
OS:        Ubuntu 22.04 LTS (Jammy Jellyfish)
ROS:       ROS2 Humble Hawksbill
Python:    3.10+
CMake:     3.22+
GCC:       11.4+
```

### ROS2 基础安装

```bash
# 1. 设置 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 添加 ROS2 源
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装 ROS2
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools

# 4. 环境设置
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 依赖包安装

```bash
# 导航相关
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization

# 视觉相关
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-camera-info-manager

# 点云相关
sudo apt install -y \
  libpcl-dev \
  ros-humble-pcl-conversions \
  ros-humble-pcl-msgs

# 其他工具
sudo apt install -y \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters

# Python 依赖
pip install ultralytics opencv-python pillow pyyaml numpy
```

---

## 项目构建

### 1. 克隆项目

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# 克隆整合仓库
git clone https://github.com/serberb/robot-vision-nav-ros2.git robot_bringup

# 克隆各子包（根据实际仓库地址调整）
git clone <astra_camera_repo> astra_camera
git clone <robot_vision_repo> robot_vision
git clone <my_pcl_pkg_repo> my_pcl_pkg
git clone <my_robot_base_repo> my_robot_base
git clone <my_robot_sensors_repo> my_robot_sensors
git clone <sllidar_repo> sllidar_ros2
```

### 2. 构建

```bash
cd ~/robot_ws

# 完整构建（首次）
colcon build --symlink-install

# 部分构建（开发时）
colcon build --symlink-install --packages-select robot_bringup

# 环境设置
source install/setup.bash
```

---

## 硬件连接

### 硬件清单

| 设备 | 接口 | 权限 |
|------|------|------|
| Astra Pro 深度相机 | USB 3.0 | 不需要特殊权限 |
| 思岚激光雷达 | /dev/ttyUSB0 | dialout 组 |
| 麦轮底盘 | /dev/serial0 | dialout 组 |
| MPU6050 IMU | I2C (/dev/i2c-1) | i2c 组 |
| 机械臂 | /dev/ttyAMA0 | dialout 组 |

### 权限设置

```bash
# 添加用户到 dialout 组（串口访问）
sudo usermod -a -G dialout $USER

# 添加用户到 i2c 组（IMU 访问）
sudo usermod -a -G i2c $USER

# 重新登录使权限生效
# 或者立即生效（不推荐长期使用）
newgrp dialout
newgrp i2c
```

### 设备检查

```bash
# 检查 USB 设备
lsusb | grep -i astra
lsusb | grep -i sllidar

# 检查串口
ls -la /dev/ttyUSB* /dev/ttyAMA* /dev/serial*

# 检查 I2C
ls -la /dev/i2c*
i2cdetect -y 1  # 应该看到 MPU6050 地址 (0x68 或 0x69)
```

---

## 运行测试

### 测试 1: 底盘测试

```bash
# 终端 1: 启动底盘
ros2 run my_robot_base base_serial_node

# 终端 2: 发送测试速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# 终端 3: 监控里程计
ros2 topic echo /wheel/odom
```

### 测试 2: 激光雷达测试

```bash
# 终端 1: 启动雷达
ros2 run sllidar_ros2 sllidar_node

# 终端 2: 查看扫描数据
ros2 topic echo /scan

# 终端 3: RViz 可视化
ros2 run rviz2 rviz2
# 添加 LaserScan 显示，topic: /scan
```

### 测试 3: 相机测试

```bash
# 终端 1: 启动相机
ros2 launch astra_camera astra_camera.launch.py

# 终端 2: 查看图像
ros2 run rqt_image_view rqt_image_view
# 选择 /camera/color/image_raw

# 终端 3: 查看点云
ros2 run rviz2 rviz2
# 添加 PointCloud2，topic: /camera/depth/points
```

### 测试 4: YOLO 检测测试

```bash
# 终端 1: 启动相机+预处理+YOLO
ros2 launch robot_bringup vision_only.launch.py

# 终端 2: 查看检测结果
ros2 topic echo /yolo_detection_result

# 终端 3: RViz
ros2 run rviz2 rviz2
# 添加 Image，topic: /image_detection
```

### 测试 5: 点云处理测试

```bash
# 终端 1: 启动点云管道
ros2 launch robot_bringup pointcloud_only.launch.py

# 终端 2: RViz
ros2 run rviz2 rviz2
# 添加 PointCloud2，topic: /processed_cloud
# 添加 MarkerArray，topic: /obb_markers
```

### 测试 6: 完整导航测试

```bash
# 步骤 1: 建图
ros2 launch robot_bringup slam_mapping.launch.py
python3 ~/robot_ws/src/robot_bringup/scripts/arrow_teleop.py

# 步骤 2: 保存地图
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '/home/ubuntu/maps/map'}"

# 步骤 3: 添加虚拟墙
python3 ~/robot_ws/src/robot_bringup/scripts/collect_wall_points.py
# 在 RViz 中点击虚拟墙端点，终端按 s 保存
python3 ~/robot_ws/src/robot_bringup/scripts/apply_virtual_walls.py

# 步骤 4: 自主导航
ros2 launch robot_bringup nav2_autonomous.launch.py
# 在 RViz 中点击 "2D Pose Estimate" 设置初始位姿
# 点击 "2D Goal Pose" 设置目标点
```

---

## 常见问题排查

### 问题 1: 相机无法打开

```bash
# 检查 USB 带宽
cat /sys/kernel/debug/usb/devices | grep -A5 "Astra"

# 尝试降低分辨率
ros2 param set /astra_camera depth_mode VGA
```

### 问题 2: 激光雷达时间戳错误

```bash
# 启动 scan_retime
ros2 run robot_bringup scan_retime.py

# 检查时间戳同步
ros2 topic echo /scan_sync/header/stamp
```

### 问题 3: Nav2 无法规划路径

```bash
# 检查代价地图
ros2 topic echo /local_costmap/costmap

# 检查 TF 树
ros2 run tf2_tools view_frames

# 检查定位状态
ros2 topic echo /amcl_pose
```

### 问题 4: 底盘不响应

```bash
# 检查串口权限
ls -la /dev/serial0
# 应该是 crw-rw---- 1 root dialout

# 检查串口通信
cat /dev/serial0 | hexdump -C  # 应该看到二进制数据
```

### 问题 5: YOLO 检测不到目标

```bash
# 检查模型路径
ros2 param get /yolo_detect model_path

# 检查置信度阈值
ros2 param get /yolo_detect conf_threshold

# 手动测试模型
python3 -c "from ultralytics import YOLO; m = YOLO('path/to/best.pt'); print(m.names)"
```

---

## 性能优化

### CPU 优化

```bash
# 1. 限制 YOLO 推理频率（已在 preprocess_node 中实现每3帧处理）
# 2. 限制点云处理频率（pcd_processor_node 中定时器 2s）
# 3. 使用 VoxelGrid 降采样（obb_analyzer_node 中已实现）

# 4. 限制 RViz 刷新率
ros2 param set /rviz2 update_rate 10
```

### 内存优化

```bash
# 1. 关闭不必要的节点（pointcloud_saver 调试时关闭）
# 2. 限制点云缓冲区大小
ros2 param set /pcd_processor max_cluster_size 15000

# 3. 使用 ROS2 压缩传输
ros2 run image_transport republish compressed raw /camera/color/image_raw
```

---

## 远程监控

```bash
# 在机器人上启动 ROS2
ros2 launch robot_bringup bringup_all.launch.py

# 在 PC 上查看（同一网络）
export ROS_DOMAIN_ID=0
ros2 topic list
ros2 run rviz2 rviz2
```

---

## 维护与更新

### 日常维护

```bash
# 检查节点健康
ros2 node list
ros2 topic list

# 检查 TF 树
ros2 run tf2_tools view_frames

# 检查日志
ros2 bag record -a -o ~/logs/$(date +%Y%m%d_%H%M%S)
```

### 更新代码

```bash
cd ~/robot_ws/src/robot_bringup
git pull
cd ~/robot_ws
colcon build --symlink-install --packages-select robot_bringup
source install/setup.bash
```
