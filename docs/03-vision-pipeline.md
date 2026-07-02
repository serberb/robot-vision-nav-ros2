# 03 - 视觉识别管道 (Vision Pipeline)

## 设计目标

基于 Astra Pro 深度相机，实现 RGB-D 目标检测：
1. 实时检测视野中的目标（如箱子、包裹等）
2. 计算目标的 3D 世界坐标
3. 输出目标位姿供点云模块和导航模块使用

---

## 现有节点分析

### 1. astra_camera (C++ 驱动)

**发布话题**:
| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/color/image_raw` | sensor_msgs/Image | 彩色图像 (640×480@30fps) |
| `/camera/depth/image_raw` | sensor_msgs/Image | 深度图像 (同分辨率) |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | 点云数据 |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | 相机内参 |

**注意**: 当前 Astra 驱动已能输出同步的彩色和深度图像，不需要额外的时间同步器。

### 2. camera_node.py (需修改)

**当前问题**: 读取本地视频文件 `/home/serberb/ros2_ws/src/robot_vision/test_video.mp4`

**修改方案**: 直接删除此节点，因为 `astra_camera` 已经发布图像话题。或者改为**图像转发节点**（如果需要额外处理）。

```python
# 修改后：直接订阅 Astra 相机，无需本地视频
# 方案 A: 删除 camera_node，launch 中直接启动 astra_camera
# 方案 B: 保留 camera_node 作为图像转发/录制节点
```

### 3. opencv_preprocess_node.py (需小改)

**功能**: RGB+深度同步 + 预处理

**当前代码分析**:
- 使用 `message_filters.ApproximateTimeSynchronizer` 同步 RGB 和深度
- 将 RGB 缩放到 640×640（YOLO 输入尺寸）
- 高斯模糊去噪
- 每 3 帧处理一次（`process_every_n = 3`）

**需要修改**:
1. 订阅的话题改为 `/camera/color/image_raw` 和 `/camera/depth/image_raw`
2. 深度图像不做缩放，但需要在 `depth_process_node` 中进行坐标还原

```python
# 修改后的订阅话题
self.rgb_sub = Subscriber(self, Image, '/camera/color/image_raw')   # 原为 /camera/rgb_image
self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw') # 添加深度订阅

# 发布的话题保持不变
self.rgb_pub = self.create_publisher(Image, '/processed/rgb', 10)
self.depth_pub = self.create_publisher(Image, '/processed/depth', 10)
```

### 4. yolo_detect_node.py (基本可用，需小改)

**功能**: YOLOv8 目标检测

**当前代码分析**:
- 加载模型 `best.pt`（需要在包目录中）
- 订阅 `/processed/rgb`
- 发布 `DetectionResult` 消息（自定义消息类型）
- 置信度阈值 0.5
- 定时保存检测图片（3秒间隔）

**需要修改**:
1. 检测框坐标是 640×640 坐标系下的，需要在 `depth_process_node` 中还原到原始分辨率
2. 考虑添加 TF 发布：检测目标的 frame_id 应该使用 `camera_color_optical_frame`

### 5. depth_process_node.py (核心节点，需修改)

**功能**: 将 2D 检测框转换为 3D 相机坐标

**当前代码分析**:
- 同步订阅 `DetectionResult` 和 `/processed/depth`
- 通过 CameraInfo 获取内参 (fx, fy, cx, cy)
- 使用 640×640 → 原始分辨率的比例因子还原中心坐标
- 反投影公式: `Xc = (u - cx) * Zc / fx`, `Yc = (v - cy) * Zc / fy`
- 发布 `TargetPoint` 消息

**需要修改**:
1. 添加 TF 广播：将 `TargetPoint` 从 `camera_color_optical_frame` 转换到 `base_link`
2. 或者发布 `geometry_msgs/PoseStamped` 而非 `TargetPoint`，因为 PoseStamped 包含 frame_id

```python
# 新增 TF 发布
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def publish_target_tf(self, target_msg):
    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'camera_color_optical_frame'
    t.child_frame_id = f'target_{target_msg.class_name}'
    t.transform.translation.x = target_msg.x
    t.transform.translation.y = target_msg.y
    t.transform.translation.z = target_msg.z
    t.transform.rotation.w = 1.0
    self.tf_broadcaster.sendTransform(t)
```

### 6. visualization_node.py (可选)

**功能**: 检测结果可视化

**注意**: 当前订阅的话题是 `/image_rgb_processed` 和 `/detection_result`，需要统一为 `/processed/rgb` 和 `/yolo_detection_result`。

### 7. gripper_control_node.py (需扩展)

**当前功能**: 简单的距离阈值判断（0.1 < z < 1.2 时开抓手）

**扩展方案**: 与 `serial_send_node` 联动，接收 `/selected_target_pose` 后触发抓取序列。

---

## 视觉管道数据流

```
┌─────────────────────────────────────────────────────────────┐
│                     Vision Pipeline                          │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  astra_camera                                                │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐           │
│  │/camera/    │  │/camera/    │  │/camera/    │           │
│  │color/      │  │depth/      │  │color/      │           │
│  │image_raw  │  │image_raw  │  │camera_info │           │
│  └──────┬─────┘  └──────┬─────┘  └──────┬─────┘           │
│         │                │                │                  │
│         ▼                ▼                │                  │
│  ┌─────────────────────────────────┐      │                  │
│  │  opencv_preprocess_node          │      │                  │
│  │  (同步+预处理)                    │      │                  │
│  │  ┌────────────┐  ┌────────────┐ │      │                  │
│  │  │ Gaussian   │  │ Resize     │ │      │                  │
│  │  │ Blur(3,3)  │  │ 640×640    │ │      │                  │
│  │  └──────┬─────┘  └──────┬─────┘ │      │                  │
│  └───────┼──────────────┼───────┘      │                  │
│          │                │                │                  │
│          ▼                ▼                │                  │
│  ┌────────────┐  ┌────────────┐             │                  │
│  │/processed/ │  │/processed/ │             │                  │
│  │rgb         │  │depth       │             │                  │
│  └──────┬─────┘  └──────────┘             │                  │
│         │                                  │                  │
│         ▼                                  │                  │
│  ┌────────────────────────────┐            │                  │
│  │  yolo_detect_node            │            │                  │
│  │  (YOLOv8 inference)          │            │                  │
│  │  conf=0.5, verbose=False   │            │                  │
│  │  ┌────────────────────┐    │            │                  │
│  │  │ DetectionResult    │    │            │                  │
│  │  │ - class_name       │    │            │                  │
│  │  │ - confidence       │    │            │                  │
│  │  │ - x1, y1, x2, y2   │    │            │                  │
│  │  │ - center_x, center_y│    │            │                  │
│  │  └─────────┬──────────┘    │            │                  │
│  └────────────┼───────────────┘            │                  │
│               │                             │                  │
│               │                             │                  │
│               ▼                             ▼                  │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  depth_process_node (message_filters sync)                 ││
│  │  ┌────────────────────────────────────────────────────┐   ││
│  │  │ 1. 获取 CameraInfo (fx, fy, cx, cy)                │   ││
│  │  │ 2. 640×640 center → original scale               │   ││
│  │  │    orig_cx = det.center_x * (orig_width / 640)   │   ││
│  │  │ 3. 读取深度图对应像素值 (mm → m)                  │   ││
│  │  │ 4. 反投影:                                        │   ││
│  │  │    Xc = (orig_cx - cx) * Zc / fx                 │   ││
│  │  │    Yc = (orig_cy - cy) * Zc / fy                 │   ││
│  │  │ 5. 发布 TargetPoint + TF                         │   ││
│  │  └────────────────────┬───────────────────────────────┘   ││
│  └───────────────────────┼───────────────────────────────────┘│
│                          │                                    │
│                          ▼                                    │
│  ┌────────────────────────────┐                              │
│  │ /target_world_point         │                              │
│  │ (TargetPoint msg)           │                              │
│  │ - class_name                │                              │
│  │ - x, y, z (camera frame)    │                              │
│  └────────────────────────────┘                              │
│                                                               │
│  ┌────────────────────────────┐                              │
│  │ TF: target_<class> →        │                              │
│  │     camera_color_optical_frame                              │
│  └────────────────────────────┘                              │
│                                                               │
└───────────────────────────────────────────────────────────────┘
```

---

## 修改清单 (Vision)

| 文件 | 修改内容 | 优先级 |
|------|----------|--------|
| `camera_node.py` | 删除或改为图像转发节点 | 高 |
| `opencv_preprocess_node.py` | 订阅话题改为 `/camera/color/image_raw` | 高 |
| `yolo_detect_node.py` | 确保 frame_id 正确 | 中 |
| `depth_process_node.py` | 添加 TF 广播; 发布 PoseStamped 替代 TargetPoint | 高 |
| `visualization_node.py` | 订阅话题统一 | 低 |
| `gripper_control_node.py` | 扩展为抓取序列控制 | 中 |

---

## 视觉模块 Launch 文件

```python
# vision_only.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Astra 相机驱动
        Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera',
            parameters=[{'device_id': ''}]  # 自动检测
        ),
        # 2. OpenCV 预处理
        Node(
            package='robot_vision',
            executable='opencv_preprocess_node',
            name='opencv_preprocess'
        ),
        # 3. YOLO 检测
        Node(
            package='robot_vision',
            executable='yolo_detect_node',
            name='yolo_detect'
        ),
        # 4. 深度处理
        Node(
            package='robot_vision',
            executable='depth_process_node',
            name='depth_process'
        ),
        # 5. 可视化 (可选)
        Node(
            package='robot_vision',
            executable='visualization_node',
            name='visualization'
        ),
    ])
```
