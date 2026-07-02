from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    一键启动全部子系统：视觉 + 点云 + 导航
    
    用法:
        ros2 launch robot_bringup bringup_all.launch.py
    """
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    
    return LaunchDescription([
        # 1. 视觉管道
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_dir, 'launch', 'vision_only.launch.py')
            )
        ),
        
        # 2. 点云管道
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_dir, 'launch', 'pointcloud_only.launch.py')
            )
        ),
        
        # 3. 导航管道
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_dir, 'launch', 'navigation_only.launch.py')
            )
        ),
    ])
