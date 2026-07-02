# 09 - 代码修改清单 (Modification Checklist)

本清单列出从当前分散状态到完整整合项目所需的所有代码修改。

---

## 高优先级修改（必须先完成）

### 1. camera_node.py → 删除或替换

**文件**: `astra_ws/src/my_vision/robot_vision/robot_vision/camera_node.py`

**问题**: 读取本地视频文件，而非实际相机

**修改方案**:
```python
# 方案 A: 删除 camera_node，launch 中直接启动 astra_camera
# 方案 B: 改为转发节点（保留原始时间戳）

class CameraForwardNode(Node):
    def __init__(self):
        super().__init__("camera_forward")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.cb, 10)
        self.pub = self.create_publisher(Image, '/camera/rgb_image', 10)
    
    def cb(self, msg):
        self.pub.publish(msg)  # 直接转发，保持原始时间戳
```

**影响**: launch 文件需要修改

---

### 2. depth_process_node.py → 添加 TF 广播

**文件**: `astra_ws/src/my_vision/robot_vision/robot_vision/depth_process_node.py`

**问题**: 只发布 `TargetPoint`（无 frame_id），外部无法知道坐标系

**修改**:
```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class DepthProcessNode(Node):
    def __init__(self):
        super().__init__('depth_process_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        # ... existing code ...
    
    def synced_callback(self, det_msg, depth_msg):
        # ... existing code ...
        
        # 新增: 发布 TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = f'target_{det_msg.class_name}'
        t.transform.translation.x = Xc
        t.transform.translation.y = Yc
        t.transform.translation.z = Zc
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        # 发布 TargetPoint
        point_msg = TargetPoint()
        point_msg.class_name = det_msg.class_name
        point_msg.x = Xc
        point_msg.y = Yc
        point_msg.z = Zc
        self.target_pub.publish(point_msg)
```

**影响**: 视觉模块与导航模块可以共享坐标系

---

### 3. serial_send_node.cpp → 动态目标发送

**文件**: `ros2_ws/src/my_pcl_pkg/src/serial_send_node.cpp`

**问题**: 只发送固定坐标，无法响应实际检测到的目标

**修改**:
```cpp
#include <geometry_msgs/msg/point_stamped.hpp>

class SerialSendNode : public rclcpp::Node
{
public:
    SerialSendNode() : Node("serial_send_node"), serial_fd_(-1)
    {
        // ... existing serial params ...
        
        // 新增: 订阅目标位姿
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/selected_target_pose", 10,
            std::bind(&SerialSendNode::target_callback, this, std::placeholders::_1));
        
        if (!openSerial()) { ... }
    }

private:
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // 笛卡尔 → 柱坐标
        double r = std::sqrt(msg->point.x * msg->point.x + msg->point.z * msg->point.z) * 1000.0;
        double theta = std::atan2(msg->point.x, msg->point.z) * 180.0 / M_PI;
        double z = msg->point.y * 1000.0;
        
        std::ostringstream oss;
        oss << "GRASP " << std::fixed << std::setprecision(1)
            << r << " " << z << " " << theta << "\n";
        sendData(oss.str());
        
        RCLCPP_INFO(this->get_logger(), "Sent: GRASP %.1f %.1f %.1f", r, z, theta);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};
```

**影响**: 点云模块可以实际控制机械臂

---

### 4. obb_to_cylindrical_node.cpp → 添加旋转矩阵

**文件**: `ros2_ws/src/my_pcl_pkg/src/obb_to_cylindrical_node.cpp`

**问题**: 旋转矩阵是单位阵，未考虑相机安装角度

**修改**:
```cpp
// 添加俯仰角参数
this->declare_parameter<double>("cam_pitch_deg", 30.0);  // 相机俯仰角
this->get_parameter("cam_pitch_deg", cam_pitch_deg_);

// 构建旋转矩阵 (绕 Y 轴)
double pitch = cam_pitch_deg_ * M_PI / 180.0;
Eigen::Matrix3f R_y;
R_y << std::cos(pitch), 0, std::sin(pitch),
       0, 1, 0,
       -std::sin(pitch), 0, std::cos(pitch);

geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& cam_pt)
{
    geometry_msgs::msg::Point base_pt;
    Eigen::Vector3f p_cam(cam_pt.x, cam_pt.y, cam_pt.z);
    Eigen::Vector3f p_base = R_y * p_cam + Eigen::Vector3f(tx_, ty_, tz_);
    base_pt.x = p_base.x();
    base_pt.y = p_base.y();
    base_pt.z = p_base.z();
    return base_pt;
}
```

**影响**: 坐标变换精度提升

---

## 中优先级修改（建议完成）

### 5. opencv_preprocess_node.py → 订阅实际相机话题

**文件**: `astra_ws/src/my_vision/robot_vision/robot_vision/opencv_preprocess_node.py`

**修改**:
```python
# 修改订阅话题
self.rgb_sub = Subscriber(self, Image, '/camera/color/image_raw')  # 原: /camera/rgb_image
self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')  # 新增

# 深度图像不做缩放，保持原始分辨率
# 但 depth_process_node 需要知道 scale 因子
```

---

### 6. pcd_processor_node.cpp → 添加 ROI 过滤

**文件**: `ros2_ws/src/my_pcl_pkg/src/pcd_processor_node.cpp`

**修改**:
```cpp
#include <pcl/filters/passthrough.h>

// 在 pointcloud_callback 中添加 PassThrough 滤波
pcl::PassThrough<pcl::PointXYZ> pass_z;
pass_z.setInputCloud(cloud);
pass_z.setFilterFieldName("z");
pass_z.setFilterLimits(0.3, 2.0);  // 只处理 0.3m ~ 2.0m
pass_z.filter(*cloud);

// 可选: X/Y 范围过滤
pcl::PassThrough<pcl::PointXYZ> pass_x;
pass_x.setInputCloud(cloud);
pass_x.setFilterFieldName("x");
pass_x.setFilterLimits(-1.0, 1.0);  // 只处理前方 1m 范围内
pass_x.filter(*cloud);
```

---

### 7. 统一话题命名

**文件**: 多个视觉节点

**修改方案**:
```
# 统一命名空间
/camera/color/image_raw       → astra_camera 发布 (不变)
/camera/depth/image_raw       → astra_camera 发布 (不变)
/processed/rgb                → opencv_preprocess 发布 (不变)
/processed/depth              → opencv_preprocess 发布 (不变)
/yolo_detection_result        → yolo_detect 发布 (不变)
/target_world_point           → depth_process 发布 (不变)

# 新增: 视觉层统一命名空间
/vision/yolo_detection_result  → yolo_detect 发布 (推荐)
/vision/target_world_point     → depth_process 发布 (推荐)
/vision/detection_image        → visualization 发布 (推荐)

# 新增: 点云层统一命名空间
/pcl/processed_cloud           → pcd_processor 发布 (推荐)
/pcl/obb_markers               → obb_analyzer 发布 (推荐)
/pcl/selected_target_pose      → obb_to_cylindrical 发布 (推荐)
```

---

## 低优先级修改（可选）

### 8. 参数外化

**文件**: 所有硬编码参数的节点

**修改**:
- 将 `process_every_n = 3` 改为 ROS 参数
- 将 `save_interval = 3.0` 改为 ROS 参数
- 将 `LINE_WIDTH = 4` 改为命令行参数

### 9. 添加诊断信息

**文件**: 所有节点

**修改**:
- 添加 `diagnostic_updater` 发布节点健康状态
- 添加 `heartbeat` 话题，外部监控节点是否存活

### 10. 日志级别控制

**文件**: 所有节点

**修改**:
- 将 `get_logger().info()` 改为 `RCLCPP_INFO_THROTTLE` 或 `throttle_duration_sec`
- 避免日志刷屏影响性能

---

## 修改优先级总结

| 优先级 | 修改项 | 估计工作量 |
|--------|--------|----------|
| 🔴 高 | camera_node 改为转发/删除 | 0.5h |
| 🔴 高 | depth_process 添加 TF 广播 | 1h |
| 🔴 高 | serial_send 改为动态订阅 | 2h |
| 🔴 高 | obb_to_cylindrical 添加旋转 | 1h |
| 🟡 中 | opencv_preprocess 订阅话题 | 0.5h |
| 🟡 中 | pcd_processor 添加 ROI | 1h |
| 🟡 中 | 统一话题命名空间 | 2h |
| 🟢 低 | 参数外化 | 3h |
| 🟢 低 | 诊断信息 | 2h |
| 🟢 低 | 日志级别控制 | 1h |

**总计估计**: ~14 小时（熟练开发者）
