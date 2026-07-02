# 07 - ROS2 话题拓扑图

## 完整话题拓扑

```
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│                              完整 ROS2 话题拓扑图                                           │
├─────────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────────────────────┐    │
│  │  SENSOR LAYER (传感器层)                                                              │    │
│  │                                                                                      │    │
│  │  astra_camera                                                                         │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │    │
│  │  │/camera/color/   │  │/camera/depth/   │  │/camera/depth/   │  │/camera/color/   │  │    │
│  │  │image_raw       │  │image_raw       │  │points          │  │camera_info     │  │    │
│  │  │(Image)         │  │(Image)         │  │(PointCloud2)   │  │(CameraInfo)    │  │    │
│  │  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘  │    │
│  │           │                    │                    │                    │         │    │
│  │  sllidar_ros2                                                                           │    │
│  │  ┌─────────────────┐                                                                  │    │
│  │  │ /scan           │  ──────────────────────────────────────────────────────────────  │    │
│  │  │(LaserScan)      │                                                                  │    │
│  │  └────────┬────────┘                                                                  │    │
│  │           │                                                                           │    │
│  │  mpu6050_node                                                                           │    │
│  │  ┌─────────────────┐                                                                  │    │
│  │  │ /imu            │  ──────────────────────────────────────────────────────────────  │    │
│  │  │(Imu)            │                                                                  │    │
│  │  └─────────────────┘                                                                  │    │
│  │                                                                                      │    │
│  │  base_serial_node                                                                     │    │
│  │  ┌─────────────────┐  ┌─────────────────┐                                           │    │
│  │  │ /wheel/odom     │  │ ← /cmd_vel      │                                           │    │
│  │  │(Odometry)       │  │ (Twist)         │                                           │    │
│  │  └─────────────────┘  └─────────────────┘                                           │    │
│  └─────────────────────────────────────────────────────────────────────────────────────┘    │
│                                            │                                                 │
│                                            │                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────────────────┐    │
│  │  VISION LAYER (视觉层)                                                                │    │
│  │                                                                                      │    │
│  │  opencv_preprocess_node                                                               │    │
│  │  ┌─────────────────┐  ┌─────────────────┐                                            │    │
│  │  │ ← /camera/color/│  │ ← /camera/depth/│                                            │    │
│  │  │   image_raw      │  │   image_raw      │                                            │    │
│  │  └────────┬────────┘  └────────┬────────┘                                            │    │
│  │           │                    │                                                    │    │
│  │           ▼                    ▼                                                    │    │
│  │  ┌─────────────────┐  ┌─────────────────┐                                           │    │
│  │  │ /processed/rgb  │  │ /processed/depth│                                           │    │
│  │  │(Image)          │  │(Image)          │                                           │    │
│  │  └────────┬────────┘  └─────────────────┘                                           │    │
│  │           │                                                                         │    │
│  │  yolo_detect_node                                                                     │    │
│  │  ┌─────────────────┐                                                                │    │
│  │  │ ← /processed/rgb  │                                                                │    │
│  │  └────────┬────────┘                                                                │    │
│  │           │                                                                         │    │
│  │           ▼                                                                         │    │
│  │  ┌─────────────────┐                                                                │    │
│  │  │ /yolo_detection_│                                                                │    │
│  │  │ result           │                                                                │    │
│  │  │(DetectionResult) │                                                                │    │
│  │  └────────┬────────┘                                                                │    │
│  │           │                                                                         │    │
│  │  depth_process_node                                                                 │    │
│  │  ┌─────────────────┐  ┌───────────────────────────────────────────────────────────┐  │    │
│  │  │ ← /yolo_        │  │ ← /camera/color/camera_info                             │  │    │
│  │  │   detection_    │  │ (CameraInfo)                                            │  │    │
│  │  │   result        │  │                                                         │  │    │
│  │  │ ← /processed/   │  │                                                         │  │    │
│  │  │   depth         │  │                                                         │  │    │
│  │  └────────┬────────┘  └───────────────────────────────────────────────────────────┘  │    │
│  │           │                                                                         │    │
│  │           ▼                                                                         │    │
│  │  ┌─────────────────┐                                                                │    │
│  │  │ /target_world_  │                                                                │    │
│  │  │ point           │                                                                │    │
│  │  │(TargetPoint)   │                                                                │    │
│  │  └─────────────────┘                                                                │    │
│  └─────────────────────────────────────────────────────────────────────────────────────┘    │
│                                            │                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────────────────┐    │
│  │  POINTCLOUD LAYER (点云层)                                                            │    │
│  │                                                                                      │    │
│  │  pcd_processor_node                                                                   │    │
│  │  ┌─────────────────┐                                                                │    │
│  │  │ ← /camera/depth/│                                                                │    │
│  │  │   points        │                                                                │    │
│  │  └────────┬────────┘                                                                │    │
│  │           │                                                                         │    │
│  │           ▼                                                                         │    │
│  │  ┌─────────────────┐                                                                │    │
│  │  │ /processed_cloud│                                                                │    │
│  │  │(PointCloud2)    │                                                                │    │
│  │  └────────┬────────┘                                                                │    │
│  │           │                                                                         │    │
│  │  obb_analyzer_node                                                                    │    │
│  │  ┌─────────────────┐                                                                │    │
│  │  │ ← /processed_   │                                                                │    │
│  │  │   cloud         │                                                                │    │
│  │  └────────┬────────┘                                                                │    │
│  │           │                                                                         │    │
│  │           ▼                                                                         │    │
│  │  ┌─────────────────┐                                                                │    │
│  │  │ /obb_markers    │                                                                │    │
│  │  │(MarkerArray)   │                                                                │    │
│  │  └────────┬────────┘                                                                │    │
│  │           │                                                                         │    │
│  │  obb_to_cylindrical_node                                                            │    │
│  │  ┌─────────────────┐                                                                │    │
│  │  │ ← /obb_markers  │                                                                │    │
│  │  └────────┬────────┘                                                                │    │
│  │           │                                                                         │    │
│  │           ▼                                                                         │    │
│  │  ┌─────────────────┐                                                                │    │
│  │  │ /selected_      │                                                                │    │
│  │  │ target_pose     │                                                                │    │
│  │  │(PointStamped)   │                                                                │    │
│  │  └────────┬────────┘                                                                │    │
│  │           │                                                                         │    │
│  │  serial_send_node                                                                     │    │
│  │  ┌─────────────────┐                                                                │    │
│  │  │ ← /selected_    │  → Serial (ttyAMA0) "GRASP r z theta\n"                        │    │
│  │  │   target_pose   │                                                                │    │
│  │  └─────────────────┘                                                                │    │
│  └─────────────────────────────────────────────────────────────────────────────────────┘    │
│                                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────────────────────┐    │
│  │  NAVIGATION LAYER (导航层)                                                            │    │
│  │                                                                                      │    │
│  │  scan_retime                                                                          │    │
│  │  ┌─────────────────┐  ┌─────────────────┐                                           │    │
│  │  │ ← /scan         │  │ /scan_sync      │                                           │    │
│  │  │                 │  │ (LaserScan)     │                                           │    │
│  │  └─────────────────┘  └─────────────────┘                                           │    │
│  │                                                                                      │    │
│  │  ekf_filter_node (robot_localization)                                               │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐                      │    │
│  │  │ ← /wheel/odom   │  │ ← /imu          │  │ /odom           │                      │    │
│  │  │                 │  │                 │  │(Odometry)       │                      │    │
│  │  └─────────────────┘  └─────────────────┘  └────────┬────────┘                      │    │
│  │                                                    │                                │    │
│  │                                                    │                                │    │
│  │  slam_toolbox (建图模式)                                                              │    │
│  │  ┌────────────────────────────────────────────────────────────────────────────┐  │    │
│  │  │ ← /scan_sync + /tf → /map (OccupancyGrid) + /tf (map → odom)               │  │    │
│  │  └────────────────────────────────────────────────────────────────────────────┘  │    │
│  │                                                                                      │    │
│  │  AMCL (定位模式)                                                                      │    │
│  │  ┌────────────────────────────────────────────────────────────────────────────┐  │    │
│  │  │ ← /scan_sync + /map → /amcl_pose (PoseWithCovarianceStamped)               │  │    │
│  │  │                    + /tf (map → odom)                                      │  │    │
│  │  └────────────────────────────────────────────────────────────────────────────┘  │    │
│  │                                                                                      │    │
│  │  Nav2 Planner + Controller                                                            │    │
│  │  ┌────────────────────────────────────────────────────────────────────────────┐  │    │
│  │  │ ← /goal_pose (PoseStamped) + /amcl_pose + /scan_sync + /map               │  │    │
│  │  │ → /cmd_vel (Twist)                                                         │  │    │
│  │  └────────────────────────────────────────────────────────────────────────────┘  │    │
│  │                                                                                      │    │
│  │  base_serial_node                                                                     │    │
│  │  ┌────────────────────────────────────────────────────────────────────────────┐  │    │
│  │  │ ← /cmd_vel (Twist) → Serial CMD4                                          │  │    │
│  │  └────────────────────────────────────────────────────────────────────────────┘  │    │
│  └─────────────────────────────────────────────────────────────────────────────────────┘    │
│                                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────────────────────┐    │
│  │  TF TREE (坐标变换树)                                                                  │    │
│  │                                                                                      │    │
│  │  map                                                                                  │    │
│  │   └── odom (EKF 发布)                                                                 │    │
│  │        └── base_link (base_serial 发布)                                               │    │
│  │             ├── laser (static)                                                        │    │
│  │             │    └── /scan 数据来源                                                    │    │
│  │             ├── camera_link (static)                                                    │    │
│  │             │    └── camera_depth_optical_frame (astra_camera 发布)                  │    │
│  │             │         └── camera_color_optical_frame (astra_camera 发布)               │    │
│  │             │              └── target_<class> (depth_process 发布)                     │    │
│  │             └── arm_base_link (static)                                                │    │
│  │                                                                                      │    │
│  └─────────────────────────────────────────────────────────────────────────────────────┘    │
│                                                                                              │
└──────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 话题命名规范

| 命名空间 | 话题 | 类型 | 发布者 | 订阅者 |
|----------|------|------|--------|--------|
| /camera | /color/image_raw | Image | astra_camera | opencv_preprocess |
| /camera | /depth/image_raw | Image | astra_camera | opencv_preprocess |
| /camera | /depth/points | PointCloud2 | astra_camera | pcd_processor, pointcloud_saver |
| /camera | /color/camera_info | CameraInfo | astra_camera | depth_process |
| /processed | /rgb | Image | opencv_preprocess | yolo_detect |
| /processed | /depth | Image | opencv_preprocess | depth_process |
| /vision | /yolo_detection_result | DetectionResult | yolo_detect | depth_process |
| /vision | /target_world_point | TargetPoint | depth_process | (visualization, gripper) |
| /pcl | /processed_cloud | PointCloud2 | pcd_processor | obb_analyzer |
| /pcl | /obb_markers | MarkerArray | obb_analyzer | obb_to_cylindrical, RViz |
| /pcl | /selected_target_pose | PointStamped | obb_to_cylindrical | serial_send, gripper |
| /sensor | /scan | LaserScan | sllidar | scan_retime |
| /sensor | /scan_sync | LaserScan | scan_retime | EKF, slam_toolbox, AMCL |
| /sensor | /imu | Imu | mpu6050 | EKF |
| /sensor | /wheel/odom | Odometry | base_serial | EKF |
| /nav | /odom | Odometry | EKF | Nav2, AMCL |
| /nav | /cmd_vel | Twist | Nav2 | base_serial |
| /nav | /amcl_pose | PoseWithCovarianceStamped | AMCL | (外部) |
| /nav | /goal_pose | PoseStamped | (外部) | Nav2 |
| /nav | /map | OccupancyGrid | slam_toolbox/AMCL | Nav2, RViz |
| /tf | map→odom | Transform | AMCL | Nav2 |
| /tf | odom→base_link | Transform | EKF | Nav2, astra_camera |
| /tf | base_link→laser | Transform | static | sllidar |
| /tf | base_link→camera_link | Transform | static | astra_camera |
| /tf | camera_link→camera_depth_optical_frame | Transform | astra_camera | depth_process |
| /tf | camera_color_optical_frame→target_<class> | Transform | depth_process | (外部) |
