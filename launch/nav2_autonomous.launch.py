from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from pathlib import Path
import os


def generate_launch_description():
    """
    自主导航模式：加载已有地图 + AMCL 定位 + Nav2 规划
    
    自动检查带虚拟墙的地图，如果不存在则从原始地图注入虚拟墙。
    
    用法:
        ros2 launch robot_bringup nav2_autonomous.launch.py
        # 在 RViz 中设置初始位姿和目标点
    
    可选参数:
        map_yaml: 地图文件路径（默认 map_blocked.yaml）
    """
    
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    default_map = os.path.join(robot_bringup_dir, 'maps', 'map_blocked.yaml')
    
    blocked_map = Path(os.path.join(robot_bringup_dir, 'maps', 'map_blocked.yaml'))
    if not blocked_map.exists():
        orig_map = os.path.join(robot_bringup_dir, 'maps', 'map.yaml')
        if Path(orig_map).exists():
            auto_inject = ExecuteProcess(
                cmd=[
                    'python3',
                    os.path.join(robot_bringup_dir, 'scripts', 'auto_inject_walls.py'),
                    '--map-yaml', orig_map,
                    '--out-name', 'map_blocked'
                ],
                output='screen'
            )
        else:
            auto_inject = []
    else:
        auto_inject = []
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map,
            description='地图 YAML 文件路径（必须是带虚拟墙的地图）'
        ),
        
        auto_inject,
        
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
        
        # 2. scan_retime
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
        
        # 5. EKF
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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.15', '0.0', '0.3', '0.0', '0.5236', '0.0', 'base_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_link', 'arm_base_link']
        ),
        
        # 7. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': LaunchConfiguration('map_yaml'),
            }]
        ),
        
        # 8. AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[os.path.join(robot_bringup_dir, 'config', 'nav2_params_final.yaml')]
        ),
        
        # 9. Nav2 Planner + Controller + BT Navigator + Recovery
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(robot_bringup_dir, 'config', 'nav2_params_final.yaml'),
                'autostart': 'true',
            }.items()
        ),
        
        # 10. Lifecycle Manager
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
            }]
        ),
        
        # 11. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(robot_bringup_dir, 'config', 'nav2.rviz')],
            output='screen'
        ),
    ])
