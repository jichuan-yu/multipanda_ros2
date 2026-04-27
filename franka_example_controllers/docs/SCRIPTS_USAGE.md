# 辅助脚本使用说明

本文档说明了DualArmMprcController测试和调试辅助脚本的使用方法。

## 📁 脚本列表

### 1. performance_monitor.py - 性能监控脚本

**功能**：
- 实时监控控制频率
- 记录关节命令数据到CSV文件
- 计算关节运动范围和速度统计
- 生成性能摘要报告

**使用方法**：

#### 基础使用
```bash
# 确保ROS 2环境已设置
source install/setup.bash

# 运行性能监控
python3 src/multipanda_ros2/franka_example_controllers/scripts/performance_monitor.py
```

#### 数据输出
脚本会自动生成两个文件：
1. **CSV数据文件**：`performance_data_YYYYMMDD_HHMMSS.csv`
   - 包含时间戳、频率、14个关节位置数据
   - 可用于后续分析和绘图

2. **摘要文件**：`performance_summary_YYYYMMDD_HHMMSS.txt`
   - 包含统计信息：最小/最大/平均/标准差
   - Ctrl+C退出时自动生成

#### 实时输出示例
```
═══════════════════════════════════════
Performance Statistics (Messages: 100)
Control Frequency: 999.50 Hz
Elapsed Time: 10.05 s
─── Joint Motion Range (rad) ───
  Left J0: 0.1234 [-1.2345, -1.1111]
  Left J1: 0.2345 [0.4567, 0.6912]
  ...
─── Joint Velocity (rad/step) ───
  Left J0: avg=0.000123, max=0.000456
  ...
═══════════════════════════════════════
```

#### 数据分析示例
```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV数据
df = pd.read_csv('performance_data_20260427_123456.csv')

# 绘制关节位置曲线
plt.figure(figsize=(12, 6))
for i in range(14):
    plt.plot(df['timestamp'], df[f'joint_{i}'], label=f'Joint {i}')
plt.xlabel('Time (s)')
plt.ylabel('Joint Position (rad)')
plt.title('Joint Trajectories')
plt.legend()
plt.grid()
plt.show()
```

---

### 2. mode_switcher.py - 模式切换脚本

**功能**：
- 在HQP和零空间控制模式之间切换
- 查看当前控制模式
- 交互式命令行界面

**使用方法**：

#### 交互式模式（推荐）
```bash
# 确保ROS 2环境已设置
source install/setup.bash

# 启动交互式模式切换器
python3 src/multipanda_ros2/franka_example_controllers/scripts/mode_switcher.py
```

交互式菜单：
```
============================================================
DualArmMprcController Mode Switcher
============================================================

Current Options:
1. Check current mode
2. Switch to Null-Space mode (default)
3. Switch to HQP mode (experimental)
4. Exit

Enter your choice (1-4):
```

#### 命令行模式
```bash
# 查看当前模式
python3 src/multipanda_ros2/franka_example_controllers/scripts/mode_switcher.py status

# 切换到零空间模式
python3 src/multipanda_ros2/franka_example_controllers/scripts/mode_switcher.py nullspace

# 切换到HQP模式
python3 src/multipanda_ros2/franka_example_controllers/scripts/mode_switcher.py hqp
```

#### 典型工作流程
```bash
# 终端1：启动性能监控
python3 scripts/performance_monitor.py

# 终端2：切换到零空间模式并测试
python3 scripts/mode_switcher.py nullspace
# （进行控制测试，记录性能数据）

# 终端2：切换到HQP模式并对比测试
python3 scripts/mode_switcher.py hqp
# （进行相同控制测试，对比性能差异）

# 终端1：Ctrl+C停止监控，生成摘要报告
```

---

## 🔧 完整测试工作流程

### 场景1：快速功能验证

```bash
# 终端1：启动控制器
ros2 control load_controller dualarm_mprc_controller
ros2 control set_controller_state dualarm_mprc_controller active

# 终端2：启动键盘控制
python3 src/multipanda_ros2/spacemouse_teleop/key_safe_pub.py

# 终端3：模式切换和监控
python3 scripts/mode_switcher.py  # 交互式切换模式
```

### 场景2：性能对比测试

```bash
# 第一步：零空间模式基准测试
python3 scripts/mode_switcher.py nullspace
python3 scripts/performance_monitor.py &
MONITOR_PID=$!

# 进行控制操作... （30秒）

kill $MONITOR_PID  # 停止监控

# 第二步：HQP模式对比测试
python3 scripts/mode_switcher.py hqp
python3 scripts/performance_monitor.py &
MONITOR_PID=$!

# 进行相同控制操作... （30秒）

kill $MONITOR_PID  # 停止监控

# 对比两次生成的CSV数据和摘要报告
```

### 场景3：长时间稳定性测试

```bash
# 启动性能监控（后台运行）
nohup python3 scripts/performance_monitor.py > monitor.log 2>&1 &

# 运行长时间测试（如1小时）
# 在测试期间可以切换模式：
python3 scripts/mode_switcher.py nullspace  # 前30分钟
python3 scripts/mode_switcher.py hqp        # 后30分钟

# 测试结束后停止监控
ps aux | grep performance_monitor
kill <PID>

# 查看监控日志
cat monitor.log
```

---

## 📊 数据分析脚本示例

### 对比分析脚本
```python
#!/usr/bin/env python3
"""
性能数据对比分析脚本
"""

import pandas as pd
import numpy as np
import sys

def analyze_performance(csv_file, mode_name):
    """分析单个CSV文件"""
    df = pd.read_csv(csv_file)

    # 提取关节数据
    joint_cols = [f'joint_{i}' for i in range(14)]
    joint_data = df[joint_cols]

    # 计算统计信息
    stats = {
        'mode': mode_name,
        'duration': df['timestamp'].max(),
        'avg_frequency': df['freq_hz'].mean(),
        'joint_range': joint_data.max() - joint_data.min(),
        'joint_std': joint_data.std(),
        'max_joint_velocity': 0.0  # 简化计算
    }

    return stats

def compare_modes(csv1, csv2, mode1_name, mode2_name):
    """对比两种模式的性能"""
    stats1 = analyze_performance(csv1, mode1_name)
    stats2 = analyze_performance(csv2, mode2_name)

    print("\n" + "="*60)
    print("Performance Comparison Report")
    print("="*60)

    print(f"\n{mode1_name} Mode:")
    print(f"  Control Frequency: {stats1['avg_frequency']:.2f} Hz")
    print(f"  Duration: {stats1['duration']:.2f} s")
    print(f"  Max Joint Motion: {stats1['joint_range'].max():.4f} rad")
    print(f"  Avg Joint Std: {stats1['joint_std'].mean():.6f} rad")

    print(f"\n{mode2_name} Mode:")
    print(f"  Control Frequency: {stats2['avg_frequency']:.2f} Hz")
    print(f"  Duration: {stats2['duration']:.2f} s")
    print(f"  Max Joint Motion: {stats2['joint_range'].max():.4f} rad")
    print(f"  Avg Joint Std: {stats2['joint_std'].mean():.6f} rad")

    print(f"\nDifferences ({mode2_name} - {mode1_name}):")
    print(f"  Frequency: {stats2['avg_frequency'] - stats1['avg_frequency']:+.2f} Hz")
    print(f"  Max Motion: {stats2['joint_range'].max() - stats1['joint_range'].max():+.4f} rad")
    print(f"  Avg Std: {stats2['joint_std'].mean() - stats1['joint_std'].mean():+.6f} rad")

    print("="*60)

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print("Usage: python3 compare_performance.py <csv1> <csv2> <mode1_name> <mode2_name>")
        sys.exit(1)

    compare_modes(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
```

**使用示例**：
```bash
python3 compare_performance.py \
    performance_data_20260427_100000.csv \
    performance_data_20260427_100500.csv \
    "Null-Space" "HQP"
```

---

## 🛠️ 故障排除

### 脚本无法运行
**问题**：`Permission denied`

**解决**：
```bash
chmod +x scripts/performance_monitor.py
chmod +x scripts/mode_switcher.py
```

### 无法连接到控制器
**问题**：`Service not available`

**检查**：
```bash
# 确认控制器正在运行
ros2 control list_controllers

# 确认节点存在
ros2 node list | grep dualarm_mprc_controller
```

### 数据文件过大
**问题**：长时间运行产生大量数据

**解决**：
1. 定期重启监控脚本
2. 使用采样间隔减少数据量
3. 只在需要时启动监控

---

## 📈 高级用法

### 自动化测试脚本
```bash
#!/bin/bash
# 自动化性能测试脚本

MODES=("nullspace" "hqp")
DURATION=30

for mode in "${MODES[@]}"; do
    echo "Testing $mode mode..."

    # 切换模式
    python3 scripts/mode_switcher.py $mode
    sleep 2

    # 启动监控
    python3 scripts/performance_monitor.py &
    MONITOR_PID=$!

    # 运行指定时间
    sleep $DURATION

    # 停止监控
    kill $MONITOR_PID

    echo "Completed $mode mode test"
    sleep 5
done

echo "All tests completed!"
```

### 可视化脚本
```python
#!/usr/bin/env python3
"""
性能数据可视化脚本
"""

import pandas as pd
import matplotlib.pyplot as plt
import sys

def plot_trajectory(csv_file, output_file):
    """绘制关节轨迹"""
    df = pd.read_csv(csv_file)

    fig, axes = plt.subplots(2, 1, figsize=(12, 10))

    # 左臂关节
    for i in range(7):
        axes[0].plot(df['timestamp'], df[f'joint_{i}'], label=f'J{i}')
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Joint Position (rad)')
    axes[0].set_title('Left Arm Joint Trajectories')
    axes[0].legend()
    axes[0].grid()

    # 右臂关节
    for i in range(7, 14):
        axes[1].plot(df['timestamp'], df[f'joint_{i}'], label=f'J{i-7}')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Joint Position (rad)')
    axes[1].set_title('Right Arm Joint Trajectories')
    axes[1].legend()
    axes[1].grid()

    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    print(f"Plot saved to: {output_file}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 plot_trajectory.py <csv_file> <output_png>")
        sys.exit(1)

    plot_trajectory(sys.argv[1], sys.argv[2])
```

---

## 📝 总结

这两个辅助脚本提供了：
- ✅ **性能监控**：实时监控和记录控制性能
- ✅ **模式切换**：方便地在控制模式之间切换
- ✅ **数据分析**：生成详细的统计报告
- ✅ **自动化测试**：支持脚本化测试流程

结合[HQP测试指南](HQP_TESTING_GUIDE.md)，可以完整评估DualArmMprcController的性能和功能。

**更新日期**：2026-04-27
**文档版本**：v1.0
