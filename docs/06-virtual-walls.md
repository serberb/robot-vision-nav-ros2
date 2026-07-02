# 06 - 虚拟墙管理系统 (Virtual Walls)

## 设计目标

在 SLAM 建图完成后，通过交互式方式在地图上添加虚拟墙，禁止机器人进入特定区域：
1. **采集**: 在 RViz 中通过 Publish Point 采集虚拟墙端点
2. **存储**: 保存为 JSON 格式，便于复用和版本控制
3. **应用**: 将虚拟墙绘制到地图 PGM 文件上（黑色 = 障碍物）
4. **加载**: Nav2 自动加载带虚拟墙的地图

---

## 现有工具脚本分析

### 1. collect_wall_points.py

**功能**: 在 RViz 中通过 Publish Point 工具采集虚拟墙端点坐标

**交互方式**:
| 命令 | 功能 |
|------|------|
| `n` | 保存当前墙，开始下一条墙 |
| `u` | 撤销当前墙最后一个点 |
| `p` | 打印当前已采集点 |
| `s` | 保存并退出 |
| `q` | 不保存退出 |

**保存格式** (`virtual_walls_points.json`):
```json
{
  "walls": [
    [[x1, y1], [x2, y2]],           // 直线墙
    [[x1, y1], [x2, y2], [x3, y3]]  // 折线墙
  ]
}
```

**使用步骤**:
1. 启动 SLAM 和 RViz
2. 运行 `collect_wall_points.py`
3. RViz 顶部选择 **Publish Point**，按顺序点击虚拟墙端点
4. 终端按 `n` 保存当前墙，开始下一条
5. 全部完成后按 `s` 保存

### 2. apply_virtual_walls.py

**功能**: 从 JSON 文件读取虚拟墙坐标，绘制到地图 PGM 上

**处理流程**:
```
输入: map.yaml + virtual_walls_points.json
  ↓
1. 加载地图 PGM 图像
2. 读取地图参数 (resolution, origin)
3. 对每个 wall:
   a. 将 world 坐标 → pixel 坐标
   b. 用 PIL ImageDraw 绘制线段
   c. 端点画圆（避免连接处有缝隙）
4. 保存新地图 (map_blocked.pgm)
5. 生成新 YAML (map_blocked.yaml)
```

**坐标转换**:
```python
def world_to_pixel(x, y, origin, resolution, height):
    # 考虑地图旋转
    dx = x - origin[0]
    dy = y - origin[1]
    c = math.cos(-origin[2])
    s = math.sin(-origin[2])
    mx = c * dx - s * dy
    my = s * dx + c * dy
    px = int(round(mx / resolution))
    py = int(round(height - 1 - my / resolution))
    return px, py
```

**线宽**: 默认 4 像素（resolution=0.05 时约 0.20m）

### 3. add_multi_virtual_walls.py

**功能**: 直接在代码中硬编码多组虚拟墙，一键生成

**适用场景**: 地图和虚拟墙坐标已知，不需要交互式采集

### 4. fix_map_with_walls.py

**功能**: 同 `add_multi_virtual_walls.py`，但硬编码坐标不同

**说明**: 这是多个历史版本的脚本，当前推荐使用 `apply_virtual_walls.py`（从 JSON 读取），因为 JSON 更易于编辑和版本控制。

---

## 整合方案

### 自动化流程

```bash
# 1. SLAM 建图
ros2 launch robot_bringup slam_mapping.launch.py

# 2. 键盘遥控建图
python3 scripts/arrow_teleop.py

# 3. 保存地图
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '/home/ubuntu/maps/map_$(date +%Y%m%d_%H%M%S)'}"

# 4. 采集虚拟墙（在 RViz 中点击）
python3 scripts/collect_wall_points.py
# → 终端按 n 保存每条墙，s 保存退出

# 5. 应用虚拟墙到地图
python3 scripts/apply_virtual_walls.py \
  --yaml /home/ubuntu/maps/map.yaml \
  --width-px 6 \
  --out-name map_blocked

# 6. 验证（RViz 加载 map_blocked.yaml 查看）

# 7. 自主导航（加载带虚拟墙的地图）
ros2 launch robot_bringup nav2_autonomous.launch.py
```

### 自动注入脚本

创建 `auto_inject_walls.py` 脚本，自动执行步骤 5:

```python
#!/usr/bin/env python3
"""
自动注入虚拟墙到地图
用法: python3 auto_inject_walls.py --map-yaml /path/to/map.yaml --walls-json /path/to/walls.json
"""
import subprocess
import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", default="/home/ubuntu/maps/map.yaml")
    parser.add_argument("--walls-json", default="/home/ubuntu/maps/virtual_walls_points.json")
    parser.add_argument("--out-name", default="map_blocked")
    parser.add_argument("--width-px", type=int, default=6)
    args = parser.parse_args()

    # 检查文件存在
    if not Path(args.map_yaml).exists():
        raise FileNotFoundError(f"地图不存在: {args.map_yaml}")
    if not Path(args.walls_json).exists():
        print(f"[WARN] 虚拟墙文件不存在，跳过注入: {args.walls_json}")
        return

    # 调用 apply_virtual_walls.py
    cmd = [
        "python3", "apply_virtual_walls.py",
        "--yaml", args.map_yaml,
        "--width-px", str(args.width_px),
        "--out-name", args.out_name
    ]
    subprocess.run(cmd, check=True)
    print(f"[OK] 虚拟墙已注入: {args.out_name}")

if __name__ == "__main__":
    main()
```

---

## 虚拟墙在 Launch 中自动加载

在 `nav2_autonomous.launch.py` 中，检查带虚拟墙的地图是否存在，不存在则自动注入：

```python
from launch.actions import ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    map_yaml = LaunchConfiguration('map', default='/home/ubuntu/maps/map_blocked.yaml')
    map_dir = Path(map_yaml).parent
    blocked_yaml = map_dir / 'map_blocked.yaml'
    
    # 如果带虚拟墙的地图不存在，则自动注入
    if not blocked_yaml.exists():
        auto_inject = ExecuteProcess(
            cmd=['python3', 'auto_inject_walls.py', '--map-yaml', str(map_yaml)],
            output='screen'
        )
    else:
        auto_inject = []
    
    return LaunchDescription([
        auto_inject,
        # ... 其他节点
    ])
```

---

## 虚拟墙设计建议

1. **线宽选择**: 
   - resolution=0.05 时，4px ≈ 0.20m，足够宽让 Nav2 识别为障碍物
   - 窄门/通道处可增加到 6px ≈ 0.30m，确保安全

2. **端点圆处理**: 
   - 每个端点画圆（半径 = max(2, width_px/2)），避免连接处有空隙
   - 这是因为 PGM 是位图，线段连接处可能有白色间隙

3. **坐标验证**: 
   - 在 RViz 中加载带虚拟墙的地图，确认墙的位置正确
   - 使用 `print_odom_pose.py` 在地图上移动机器人，确认坐标系一致

4. **版本管理**: 
   - 将 `virtual_walls_points.json` 加入 Git 版本控制
   - 每次修改后重新生成 `map_blocked.yaml`

---

## 修改清单 (Virtual Walls)

| 文件 | 修改内容 | 优先级 |
|------|----------|--------|
| `collect_wall_points.py` | 路径改为参数化 | 低 |
| `apply_virtual_walls.py` | 路径改为参数化 | 低 |
| `auto_inject_walls.py` | **新增**: 自动注入脚本 | 中 |
| `nav2_autonomous.launch.py` | 自动检查/注入虚拟墙 | 中 |
