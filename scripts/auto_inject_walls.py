#!/usr/bin/env python3
"""
自动注入虚拟墙到地图
用法:
    python3 auto_inject_walls.py --map-yaml /path/to/map.yaml --walls-json /path/to/walls.json
"""
import subprocess
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description='自动注入虚拟墙到地图')
    parser.add_argument('--map-yaml', default='/home/ubuntu/maps/map.yaml',
                        help='原始地图 YAML 文件路径')
    parser.add_argument('--walls-json', default='/home/ubuntu/maps/virtual_walls_points.json',
                        help='虚拟墙 JSON 文件路径')
    parser.add_argument('--out-name', default='map_blocked',
                        help='输出地图文件名（不含扩展名）')
    parser.add_argument('--width-px', type=int, default=6,
                        help='虚拟墙线宽（像素）')
    args = parser.parse_args()

    map_yaml = Path(args.map_yaml)
    walls_json = Path(args.walls_json)

    # 检查文件存在
    if not map_yaml.exists():
        raise FileNotFoundError(f'地图不存在: {map_yaml}')
    if not walls_json.exists():
        print(f'[WARN] 虚拟墙文件不存在，跳过注入: {walls_json}')
        return

    # 调用 apply_virtual_walls.py
    script_dir = Path(__file__).parent
    apply_script = script_dir / 'apply_virtual_walls.py'
    
    if not apply_script.exists():
        # 如果找不到，使用当前目录的 apply_virtual_walls.py
        apply_script = Path('apply_virtual_walls.py')
    
    cmd = [
        'python3', str(apply_script),
        '--yaml', str(map_yaml),
        '--width-px', str(args.width_px),
        '--out-name', args.out_name
    ]
    
    print(f'[INFO] 执行: {" ".join(cmd)}')
    result = subprocess.run(cmd, check=True, capture_output=True, text=True)
    print(result.stdout)
    
    if result.returncode == 0:
        print(f'[OK] 虚拟墙已注入: {args.out_name}')
    else:
        print(f'[ERROR] 注入失败: {result.stderr}')


if __name__ == '__main__':
    main()
