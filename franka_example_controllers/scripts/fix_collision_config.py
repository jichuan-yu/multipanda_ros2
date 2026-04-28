#!/usr/bin/env python3
"""
自动修改师兄控制器配置以降低碰撞避免严格性
"""

import os
import shutil
import sys

def backup_and_modify_config():
    """备份并修改配置文件"""

    config_file = "/home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/config/control_parameters_sim.yaml"

    if not os.path.exists(config_file):
        print(f"❌ 配置文件不存在: {config_file}")
        return False

    # 备份原文件
    backup_file = config_file + ".backup"
    if not os.path.exists(backup_file):
        shutil.copy(config_file, backup_file)
        print(f"✓ 已备份原配置到: {backup_file}")

    # 读取配置
    with open(config_file, 'r') as f:
        lines = f.readlines()

    # 修改配置
    modified = False
    new_lines = []

    print("\n修改的配置项：")
    print("-" * 50)

    for line in lines:
        # 降低碰撞避免增益
        if 'K_collision_avoidance:' in line and '5' in line and '#' not in line.split('K_collision_avoidance')[0]:
            new_line = line.replace('5', '1', 1)
            print(f"  K_collision_avoidance: 5 → 1")
            new_lines.append(new_line)
            modified = True
        # 降低关节限制增益（可选）
        elif 'K_joint_limit:' in line and '5.0' in line:
            new_line = line.replace('5.0', '2.0')
            print(f"  K_joint_limit: 5.0 → 2.0")
            new_lines.append(new_line)
            modified = True
        else:
            new_lines.append(line)

    if modified:
        # 写回文件
        with open(config_file, 'w') as f:
            f.writelines(new_lines)
        print("\n✓ 配置文件已修改")
        return True
    else:
        print("\n⚠️  配置文件似乎已经是修改后的版本")
        return False

if __name__ == '__main__':
    print("==========================================")
    print("修改师兄控制器配置")
    print("==========================================")
    print()

    if backup_and_modify_config():
        print("\n==========================================")
        print("下一步")
        print("==========================================")
        print()
        print("配置已修改，现在重启师兄控制器测试：")
        print()
        print("1. 如果师兄控制器正在运行，先关闭：")
        print("   ros2 node kill /dual_arm_mprc_node")
        print()
        print("2. 重新启动师兄控制器：")
        print("   ros2 run dual_arm_reactive_control dual_arm_mprc_node")
        print()
        print("3. 发送测试轨迹：")
        print("   python3 src/multipanda_ros2/franka_example_controllers/scripts/test_safe_joint_position.py")
        print()
        print("如果师兄控制器工作正常，说明确实是碰撞约束的问题")
        print()
        print("恢复原配置：")
        print(f"   cp {config_file}.backup {config_file}")
        print()
    else:
        print("\n没有修改配置文件")
        sys.exit(1)
