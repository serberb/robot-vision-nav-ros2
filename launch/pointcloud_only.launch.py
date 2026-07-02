from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """
    仅启动点云处理管道：Astra 点云 → 平面分割 → 聚类 → OBB → 柱坐标 → 串口
    
    用法:
        ros2 launch robot_bringup pointcloud_only.launch.py
    
    可选参数:
        save_pcd: true/false (是否保存 PCD 文件)
        use_roi: true/false (是否使用 ROI 过滤)
    """
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'save_pcd',
            default_value='false',
            description='是否保存点云 PCD 文件'
        ),
        DeclareLaunchArgument(
            'use_roi',
            default_value='true',
            description='是否使用 ROI 空间过滤'
        ),
        
        # 1. Astra 相机驱动（点云模式）
        Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera',
            parameters=[{
                'device_id': '',
                'depth_mode': 'VGA',
                'color_mode': 'VGA',
                'enable_point_cloud': True,  # 点云模式需要
            }]
        ),
        
        # 2. 点云保存（可选，调试）
        Node(
            package='my_pcl_pkg',
            executable='pointcloud_saver',
            name='pointcloud_saver',
            parameters=[{
                'topic': '/camera/depth/points',
                'save_dir': '/home/ubuntu/pcd_output',
            }],
            condition=IfCondition(LaunchConfiguration('save_pcd'))
        ),
        
        # 3. 点云处理（RANSAC + 聚类）
        Node(
            package='my_pcl_pkg',
            executable='pcd_processor_node',
            name='pcd_processor',
            parameters=[{
                'input_topic': '/camera/depth/points',
                'output_topic': '/processed_cloud',
                'distance_threshold': 0.01,
                'max_iterations': 500,
                'probability': 0.99,
                'cluster_tolerance': 0.02,
                'min_cluster_size': 100,
                'max_cluster_size': 25000,
                'process_interval': 2.0,
                'use_roi': LaunchConfiguration('use_roi'),
                'roi_z_min': 0.3,
                'roi_z_max': 2.0,
            }]
        ),
        
        # 4. OBB 分析（PCA + 规则判定）
        Node(
            package='my_pcl_pkg',
            executable='obb_analyzer_node',
            name='obb_analyzer',
            parameters=[{
                'input_topic': '/processed_cloud',
                'marker_topic': '/obb_markers',
                'cluster_tolerance': 0.02,
                'min_cluster_size': 40,
                'max_cluster_size': 20000,
                'voxel_leaf': 0.01,
                'max_volume': 0.2,
                'min_volume': 0.00001,
                'max_flatness': 0.99,
                'min_flatness': 0.2,
                'min_elongation': 1.0,
                'max_elongation': 1.70,
                'min_angle_z': 60.0,
                'max_z': 1.5,
            }]
        ),
        
        # 5. 坐标转换（OBB → 柱坐标）
        Node(
            package='my_pcl_pkg',
            executable='obb_to_cylindrical_node',
            name='obb_to_cylindrical',
            parameters=[{
                'cam_to_base_x': 0.15,
                'cam_to_base_y': 0.0,
                'cam_to_base_z': 0.3,
                'cam_pitch_deg': 30.0,
                'output_topic': '/selected_target_pose',
            }]
        ),
        
        # 6. 串口发送（机械臂控制）
        Node(
            package='my_pcl_pkg',
            executable='serial_send_node',
            name='serial_send',
            parameters=[{
                'serial_port': '/dev/ttyAMA0',
                'baud_rate': 115200,
            }]
        ),
    ])
