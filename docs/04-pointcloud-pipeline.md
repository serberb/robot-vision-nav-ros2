# 04 - 点云目标定向识别管道 (PointCloud Pipeline)

## 设计目标

从 Astra 深度相机点云或视觉模块输出的目标区域，提取目标的几何特征：
1. 平面分割（去除地面/桌面）
2. 欧式聚类（分离独立物体）
3. OBB 有向包围盒分析（计算主方向、尺寸、体积）
4. 规则长方体判定（过滤非目标物体）
5. 坐标变换到机械臂基座系（笛卡尔 → 柱坐标）
6. 串口发送控制指令到机械臂

---

## 现有节点分析

### 1. my_pcl_node.cpp (调试节点)

**功能**: 订阅点云，每 10 帧保存一次为 PCD 文件

**用途**: 调试用，正式上线可关闭

**参数**:
- `save_dir`: PCD 保存路径
- `topic`: 订阅话题（默认 `/camera/depth/points`）

### 2. pcd_processor_node.cpp (核心节点)

**功能**: 原始点云 → 平面分割 → 障碍物提取 → 欧式聚类 → 彩色点云

**处理流程**:
```
输入: /camera/depth/points (PointCloud2)
  ↓
1. ROS → PCL 转换
  ↓
2. RANSAC 平面分割 (SACMODEL_PLANE)
   - distance_threshold: 0.01m
   - max_iterations: 500
   - probability: 0.99
  ↓
3. 提取非平面点 (setNegative=true)
  ↓
4. 去除 NaN/Inf
  ↓
5. 欧式聚类 (EuclideanClusterExtraction)
   - cluster_tolerance: 0.02m
   - min_cluster_size: 100
   - max_cluster_size: 25000
  ↓
6. 彩色标记（每个聚类不同颜色）
  ↓
输出: /processed_cloud (PointCloud2, XYZRGB)
```

**定时器**: 每 2 秒处理一次（`process_interval = 2.0`），避免 CPU 过载

**需要修改**:
1. 添加**感兴趣区域 (ROI)** 过滤：只处理视觉模块检测到的目标区域附近的点云
2. 或者添加**空间滤波**: 限制 Z 范围（0.3m ~ 2.0m），去除远处噪声

```cpp
// 在 pcd_processor_node 中添加 PassThrough 滤波
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud(cloud);
pass.setFilterFieldName("z");
pass.setFilterLimits(0.3, 2.0);  // 只保留 0.3m ~ 2.0m
pass.filter(*cloud);
```

### 3. obb_analyzer_node.cpp (核心节点)

**功能**: 对聚类后的点云计算 OBB 有向包围盒，判定是否为规则长方体

**处理流程**:
```
输入: /processed_cloud (PointCloud2, XYZRGB)
  ↓
1. ROS → PCL XYZRGB 转换
  ↓
2. 提取 XYZ（忽略颜色）用于聚类
  ↓
3. 可选 VoxelGrid 降采样 (0.01m)
  ↓
4. 重新欧式聚类（确保每个物体独立）
  ↓
5. 对每个聚类:
   a. PCA 计算特征值/特征向量
   b. 计算质心
   c. 变换到 PCA 坐标系，计算 AABB
   d. 计算 length, width, height, volume, elongation, flatness
   e. 判定规则长方体
  ↓
6. 生成 MarkerArray（绿色线框 + 白色文本标记）
  ↓
输出: /obb_markers (MarkerArray)
```

**判定规则**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `min_volume` | 0.00001 m³ | 最小体积 |
| `max_volume` | 0.2 m³ | 最大体积 |
| `min_flatness` | 0.2 | 扁平度下限 |
| `max_flatness` | 0.99 | 扁平度上限 |
| `min_elongation` | 1.0 | 长宽比下限 |
| `max_elongation` | 1.70 | 长宽比上限 |
| `min_angle_z` | 60° | 主方向与 Z 轴最小夹角 |
| `max_z` | 1.5m | 最大 Z 坐标 |

**需要修改**:
1. 当与视觉模块联用时，可以优先分析视觉检测到的目标区域
2. 添加**目标跟踪**: 为每个检测到的箱子分配 ID，持续跟踪

### 4. obb_to_cylindrical_node.cpp (需修改)

**功能**: 将 OBB Marker 转换为机械臂柱坐标

**当前问题**:
1. 选择逻辑简单：取 Z 坐标最小的（距离相机最近）
2. 旋转矩阵是单位阵（未考虑相机安装角度）
3. 输出的是 PointStamped，不是完整的 Pose

**修改方案**:

```cpp
// 改进版: 考虑相机安装角度
// 假设相机安装在 base_link 前方 0.15m, 高度 0.3m, 俯仰角 30°
this->declare_parameter<double>("cam_to_base_x", 0.15);
this->declare_parameter<double>("cam_to_base_y", 0.0);
this->declare_parameter<double>("cam_to_base_z", 0.3);
this->declare_parameter<double>("cam_pitch_deg", 30.0);  // 俯仰角

// 构建旋转矩阵 (绕 Y 轴旋转 pitch)
double pitch = cam_pitch_deg_ * M_PI / 180.0;
Eigen::Matrix3f R_y;
R_y << cos(pitch), 0, sin(pitch),
       0, 1, 0,
       -sin(pitch), 0, cos(pitch);

// 完整变换: base_p = R * cam_p + t
base_pt.x = R_y(0,0)*cam_pt.x + R_y(0,2)*cam_pt.z + tx_;
base_pt.y = cam_pt.y + ty_;
base_pt.z = R_y(2,0)*cam_pt.x + R_y(2,2)*cam_pt.z + tz_;
```

### 5. serial_send_node.cpp (需大改)

**当前问题**: 只发送**固定坐标**（`fixed_r=80mm, fixed_z=10mm, fixed_theta=140°`），无法动态控制

**修改方案**: 订阅 `/selected_target_pose`，动态发送柱坐标

```cpp
class SerialSendNode : public rclcpp::Node
{
public:
    SerialSendNode() : Node("serial_send_node"), serial_fd_(-1)
    {
        // 串口参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
        this->declare_parameter<int>("baud_rate", 115200);
        
        // 订阅目标位姿
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/selected_target_pose", 10,
            std::bind(&SerialSendNode::target_callback, this, std::placeholders::_1));
        
        // 打开串口
        if (!openSerial()) { ... }
    }

private:
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // 笛卡尔 → 柱坐标
        double r = sqrt(msg->point.x * msg->point.x + msg->point.z * msg->point.z) * 1000;  // mm
        double theta = atan2(msg->point.x, msg->point.z) * 180.0 / M_PI;  // deg
        double z = msg->point.y * 1000;  // mm
        
        // 发送
        std::ostringstream oss;
        oss << "GRASP " << std::fixed << std::setprecision(1)
            << r << " " << z << " " << theta << "\n";
        sendData(oss.str());
        
        RCLCPP_INFO(this->get_logger(), "Sent: GRASP %.1f %.1f %.1f", r, z, theta);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};
```

---

## 点云管道数据流

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PointCloud Pipeline                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  pcd_processor_node                                                  │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ /camera/depth/  │  输入: 原始点云                                  │   │
│  │  │ points          │       (camera_depth_optical_frame)              │   │
│  │  └────────┬────────┘                                               │   │
│  │           │                                                       │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ 1. PassThrough  │  Z: 0.3m ~ 2.0m (去除远近噪声)                  │   │
│  │  │    Filter       │                                               │   │
│  │  └────────┬────────┘                                               │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ 2. RANSAC Plane │  分割地面/桌面                                  │   │
│  │  │    Segmentation │  distance_threshold=0.01m                      │   │
│  │  └────────┬────────┘                                               │   │
│  │           │                                                       │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ 3. Extract      │  提取非平面点 (障碍物)                          │   │
│  │  │    Negative     │                                               │   │
│  │  └────────┬────────┘                                               │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ 4. Remove NaN   │                                               │   │
│  │  └────────┬────────┘                                               │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ 5. Euclidean    │  聚类分离独立物体                               │   │
│  │  │    Clustering   │  tolerance=0.02m, min=100, max=25000          │   │
│  │  └────────┬────────┘                                               │   │
│  │           │                                                       │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ 6. Colorize     │  每个聚类不同颜色                               │   │
│  │  │    (RGB)        │                                               │   │
│  │  └────────┬────────┘                                               │   │
│  │           │                                                       │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ /processed_cloud│  输出: 彩色聚类点云                             │   │
│  │  │ (XYZRGB)        │                                               │   │
│  │  └─────────────────┘                                               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                   │                                          │
│                                   ▼                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  obb_analyzer_node                                                   │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ /processed_cloud│  输入: 彩色聚类点云                              │   │
│  │  └────────┬────────┘                                               │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ 1. VoxelGrid    │  降采样加速 (0.01m)                             │   │
│  │  │    Downsampling │                                               │   │
│  │  └────────┬────────┘                                               │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ 2. Re-cluster   │  重新聚类确保分离                               │   │
│  │  │    (Euclidean)  │                                               │   │
│  │  └────────┬────────┘                                               │   │
│  │           │                                                       │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ 对每个聚类:        │                                               │   │
│  │  │  · PCA 计算特征向量│                                               │   │
│  │  │  · 计算质心        │                                               │   │
│  │  │  · AABB in PCA   │                                               │   │
│  │  │  · 计算 l,w,h    │                                               │   │
│  │  │  · 规则判定       │                                               │   │
│  │  └────────┬────────┘                                               │   │
│  │           │                                                       │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ /obb_markers    │  输出: 绿色线框 + 文本标记 (MarkerArray)        │   │
│  │  │ (MarkerArray)   │                                               │   │
│  │  └─────────────────┘                                               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                   │                                          │
│                                   ▼                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  obb_to_cylindrical_node                                             │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ /obb_markers    │  输入: OBB 标记                                 │   │
│  │  └────────┬────────┘                                               │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ 1. Select Target│  选择最近目标 (Z最小)                           │   │
│  │  │                 │                                               │   │
│  │  │ 2. Transform    │  camera → base_link (R + t)                     │   │
│  │  │                 │                                               │   │
│  │  │ 3. Cartesian →  │  x = r·sin(θ), z = r·cos(θ)                    │   │
│  │  │    Cylindrical  │                                               │   │
│  │  └────────┬────────┘                                               │   │
│  │           │                                                       │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │/selected_target_│  输出: 目标位姿 (base_link)                      │   │
│  │  │pose             │                                               │   │
│  │  │(PointStamped)   │                                               │   │
│  │  └─────────────────┘                                               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                   │                                          │
│                                   ▼                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  serial_send_node (MODIFIED)                                         │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │/selected_target_│  输入: 目标位姿                                 │   │
│  │  │pose             │                                               │   │
│  │  └────────┬────────┘                                               │   │
│  │           ▼                                                       │   │
│  │  ┌─────────────────┐                                               │   │
│  │  │ Format:         │  输出: 串口指令                                  │   │
│  │  │ GRASP r z theta │  "GRASP 80.0 10.0 140.0\n"                     │   │
│  │  │ (mm, mm, deg)   │                                               │   │
│  │  └─────────────────┘                                               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 修改清单 (PointCloud)

| 文件 | 修改内容 | 优先级 |
|------|----------|--------|
| `pcd_processor_node.cpp` | 添加 PassThrough Z 滤波 (0.3~2.0m) | 中 |
| `obb_analyzer_node.cpp` | 添加目标 ID 跟踪; 参数可调 | 中 |
| `obb_to_cylindrical_node.cpp` | 添加真实旋转矩阵 (俯仰角) | 高 |
| `serial_send_node.cpp` | 改为订阅 `/selected_target_pose` 动态发送 | 高 |
| `my_pcl_node.cpp` | 添加参数控制是否保存 | 低 |

---

## 点云模块 Launch 文件

```cpp
// pointcloud_only.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 1. Astra 相机（点云模式）
        Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera',
            parameters=[{'depth_mode': 'POINTCLOUD'}]
        ),
        # 2. 点云保存（可选，调试）
        Node(
            package='my_pcl_pkg',
            executable='pointcloud_saver',
            name='pointcloud_saver',
            parameters=[{'save_dir': '~/robot_ws/pcd_output'}],
            condition=IfCondition(LaunchConfiguration('save_pcd', default='false'))
        ),
        # 3. 点云处理
        Node(
            package='my_pcl_pkg',
            executable='pcd_processor_node',
            name='pcd_processor',
            parameters=[{
                'input_topic': '/camera/depth/points',
                'output_topic': '/processed_cloud',
                'distance_threshold': 0.01,
                'cluster_tolerance': 0.02,
                'process_interval': 2.0
            }]
        ),
        # 4. OBB 分析
        Node(
            package='my_pcl_pkg',
            executable='obb_analyzer_node',
            name='obb_analyzer',
            parameters=[{
                'input_topic': '/processed_cloud',
                'marker_topic': '/obb_markers',
                'max_volume': 0.2,
                'min_elongation': 1.0,
                'max_elongation': 1.70
            }]
        ),
        # 5. 坐标转换
        Node(
            package='my_pcl_pkg',
            executable='obb_to_cylindrical_node',
            name='obb_to_cylindrical',
            parameters=[{
                'cam_to_base_x': 0.15,
                'cam_to_base_y': 0.0,
                'cam_to_base_z': 0.3,
                'cam_pitch_deg': 30.0
            }]
        ),
        # 6. 串口发送
        Node(
            package='my_pcl_pkg',
            executable='serial_send_node',
            name='serial_send',
            parameters=[{
                'serial_port': '/dev/ttyAMA0',
                'baud_rate': 115200
            }]
        ),
    ])
```
