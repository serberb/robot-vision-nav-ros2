#!/usr/bin/env python3
"""
模式切换脚本：在 SLAM 建图和自主导航之间切换
用法:
    python3 switch_mode.py --mode slam
    python3 switch_mode.py --mode nav
    python3 switch_mode.py --mode vision
    python3 switch_mode.py --mode pcl
"""
import subprocess
import argparse
import sys


MODES = {
    'slam': 'ros2 launch robot_bringup slam_mapping.launch.py',
    'nav': 'ros2 launch robot_bringup nav2_autonomous.launch.py',
    'vision': 'ros2 launch robot_bringup vision_only.launch.py',
    'pcl': 'ros2 launch robot_bringup pointcloud_only.launch.py',
    'all': 'ros2 launch robot_bringup bringup_all.launch.py',
    'teleop': 'python3 arrow_teleop.py',
}


def main():
    parser = argparse.ArgumentParser(description='机器人模式切换')
    parser.add_argument('--mode', required=True, choices=MODES.keys(),
                        help='运行模式: slam, nav, vision, pcl, all, teleop')
    parser.add_argument('--kill-all', action='store_true',
                        help='先停止所有 ROS2 进程')
    args = parser.parse_args()

    if args.kill_all:
        print('[INFO] 停止所有 ROS2 进程...')
        subprocess.run(['pkill', '-f', 'ros2'], check=False)
        subprocess.run(['pkill', '-f', 'rviz2'], check=False)
        subprocess.run(['sleep', '2'], check=False)

    cmd = MODES[args.mode]
    print(f'[INFO] 启动模式: {args.mode}')
    print(f'[INFO] 命令: {cmd}')
    
    try:
        subprocess.run(cmd, shell=True, check=True)
    except KeyboardInterrupt:
        print('\n[INFO] 用户中断')
    except subprocess.CalledProcessError as e:
        print(f'[ERROR] 启动失败: {e}')
        sys.exit(1)


if __name__ == '__main__':
    main()
