from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """
    仅启动导航管道：激光雷达 + IMU + 底盘 → EKF → 导航
    
    用法:
        ros2 launch robot_bringup navigation_only.launch.py
    
    可选参数:
        use_amcl: true/false (是否启动 AMCL 定位)
        map_yaml: 地图文件路径
    """
    
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_amcl',
            default_value='false',
            description='是否启动 AMCL 定位（建图时不需要）'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(robot_bringup_dir, 'maps', 'map_blocked.yaml'),
            description='地图 YAML 文件路径'
        ),
        
        # 1. 激光雷达
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }]
        ),
        
        # 2. scan_retime（时间戳重对齐）
        Node(
            package='robot_bringup',
            executable='scan_retime.py',
            name='scan_retime'
        ),
        
        # 3. IMU
        Node(
            package='my_robot_sensors',
            executable='mpu6050_node',
            name='mpu6050',
            parameters=[{
                'i2c_bus': 1,
                'sample_rate': 100,
            }]
        ),
        
        # 4. 底盘驱动
        Node(
            package='my_robot_base',
            executable='base_serial_node',
            name='base_serial',
            parameters=[os.path.join(robot_bringup_dir, 'config', 'base_serial.yaml')]
        ),
        
        # 5. EKF 状态估计
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(robot_bringup_dir, 'config', 'ekf.yaml')]
        ),
        
        # 6. TF 静态变换
        # 激光雷达 → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.15', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),
        # 相机 → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.15', '0.0', '0.3', '0.0', '0.5236', '0.0', 'base_link', 'camera_link']
        ),
        # 机械臂基座 → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_link', 'arm_base_link']
        ),
        
        # 7. Map Server（如果启用 AMCL）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': LaunchConfiguration('map_yaml'),
            }],
            condition=IfCondition(LaunchConfiguration('use_amcl'))
        ),
        
        # 8. AMCL（如果启用）
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[os.path.join(robot_bringup_dir, 'config', 'nav2_params_final.yaml')],
            condition=IfCondition(LaunchConfiguration('use_amcl'))
        ),
        
        # 9. Nav2 导航（如果启用 AMCL）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(robot_bringup_dir, 'config', 'nav2_params_final.yaml'),
                'autostart': 'true',
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_amcl'))
        ),
        
        # 10. Lifecycle Manager（如果启用 Nav2）
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
            }],
            condition=IfCondition(LaunchConfiguration('use_amcl'))
        ),
    ])
