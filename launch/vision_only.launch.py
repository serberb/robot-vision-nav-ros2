from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """
    仅启动视觉识别管道：Astra 相机 → 预处理 → YOLO 检测 → 深度处理 → 可视化
    
    用法:
        ros2 launch robot_bringup vision_only.launch.py
    
    可选参数:
        save_frames: true/false (是否保存检测图片)
    """
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'save_frames',
            default_value='false',
            description='是否保存 YOLO 检测图片'
        ),
        
        # 1. Astra 相机驱动
        Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera',
            parameters=[{
                'device_id': '',
                'depth_mode': 'VGA',
                'color_mode': 'VGA',
                'enable_point_cloud': False,  # 视觉模式不需要点云
            }]
        ),
        
        # 2. OpenCV 预处理（RGB+深度同步）
        Node(
            package='robot_vision',
            executable='opencv_preprocess_node',
            name='opencv_preprocess',
            parameters=[{
                'process_every_n': 3,
            }]
        ),
        
        # 3. YOLO 检测
        Node(
            package='robot_vision',
            executable='yolo_detect_node',
            name='yolo_detect',
            parameters=[{
                'conf_threshold': 0.5,
                'save_frames': LaunchConfiguration('save_frames'),
                'save_interval': 3.0,
            }]
        ),
        
        # 4. 深度处理（2D → 3D 坐标）
        Node(
            package='robot_vision',
            executable='depth_process_node',
            name='depth_process',
            parameters=[{
                'publish_tf': True,
            }]
        ),
        
        # 5. 可视化（可选）
        Node(
            package='robot_vision',
            executable='visualization_node',
            name='visualization',
            condition=IfCondition('true')
        ),
    ])
