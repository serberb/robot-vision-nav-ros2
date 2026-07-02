from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from pathlib import Path
import os


def generate_launch_description():
    """
    SLAM 建图模式：激光雷达 + IMU + 底盘 → EKF → SLAM Toolbox
    
    用法:
        ros2 launch robot_bringup slam_mapping.launch.py
        # 然后启动键盘遥控
        python3 scripts/arrow_teleop.py
    
    可选参数:
        map_name: 地图保存名称
    """
    
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='map',
            description='地图保存名称（不含路径）'
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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.15', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),
        
        # 7. SLAM Toolbox（在线同步建图）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch', 'online_sync_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': os.path.join(robot_bringup_dir, 'config', 'slam_toolbox.yaml'),
            }.items()
        ),
        
        # 8. RViz（带 SLAM 配置）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(robot_bringup_dir, 'config', 'slam.rviz')],
            output='screen'
        ),
    ])
