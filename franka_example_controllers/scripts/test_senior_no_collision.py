#!/usr/bin/env python3
"""
测试师兄控制器 - 禁用碰撞避免约束

这个脚本会检查师兄控制器的源代码，
找到碰撞避免约束的添加位置，
并告诉你如何临时禁用它来测试。
"""

import os
import sys

def find_constraint_additions():
    """查找师兄控制器中约束添加的位置"""

    controller_file = "/home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/src/dual_arm_safe_controller_sim.cpp"

    if not os.path.exists(controller_file):
        print(f"❌ 文件不存在: {controller_file}")
        return

    print("==========================================")
    print("师兄控制器约束添加位置")
    print("==========================================")
    print()

    with open(controller_file, 'r') as f:
        lines = f.readlines()

    print("找到的约束添加代码：")
    print()

    for i, line in enumerate(lines, 1):
        if 'constraint_manager_.addConstraint' in line and 'COLLISION_AVOIDANCE' in line:
            # 打印前后几行作为上下文
            start = max(0, i - 3)
            end = min(len(lines), i + 2)
            for j in range(start, end):
                prefix = ">>> " if j == i - 1 else "    "
                print(f"{prefix}{j+1:4d}: {lines[j]}", end='')
            print()

def suggest_disable_collision():
    """建议如何禁用碰撞避免约束"""

    print("==========================================")
    print("临时禁用碰撞避免约束的方法")
    print("==========================================")
    print()

    controller_file = "/home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/src/dual_arm_safe_controller_sim.cpp"

    print("方法1：注释掉碰撞避免约束")
    print("-" * 50)
    print(f"编辑文件: {controller_file}")
    print()
    print("找到这一行（约第166行）：")
    print("    constraint_manager_.addConstraint(Constraint(3, ConstraintType::COLLISION_AVOIDANCE_ALLCBF));")
    print()
    print("改为（添加注释）：")
    print("    // constraint_manager_.addConstraint(Constraint(3, ConstraintType::COLLISION_AVOIDANCE_ALLCBF));")
    print()
    print("然后重新编译：")
    print("    cd /home/xiaozy24/dual_panda_ws/src/dualarm_mprc")
    print("    colcon build --packages-select dual_arm_reactive_control")
    print("    source install/setup.bash")
    print()

    print("方法2：修改碰撞参数（推荐）")
    print("-" * 50)
    config_file = "/home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/config/control_parameters_sim.yaml"
    print(f"编辑文件: {config_file}")
    print()
    print("降低碰撞避免增益（从5改为1）：")
    print("    K_collision_avoidance: 1")
    print()
    print("或者增大激活距离（减少约束频繁激活）：")
    print("  （需要修改 constraint_manager.cpp 中的 d_active_）")
    print()

def test_collision_config():
    """检查当前碰撞配置"""

    print("==========================================")
    print("检查当前碰撞配置")
    print("==========================================")
    print()

    config_file = "/home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/config/control_parameters_sim.yaml"

    if os.path.exists(config_file):
        with open(config_file, 'r') as f:
            content = f.read()

        print(f"读取配置文件: {config_file}")
        print()

        for line in content.split('\n'):
            if 'K_collision_avoidance' in line or 'w_collision_avoidance' in line:
                print(f"  {line}")

        print()
    else:
        print(f"❌ 配置文件不存在: {config_file}")

    # 检查碰撞球配置
    collision_yaml = "/home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/config/panda_collision_spheres.yaml"

    if os.path.exists(collision_yaml):
        import yaml
        with open(collision_yaml, 'r') as f:
            try:
                collision_config = yaml.safe_load(f)
                print(f"碰撞球配置: {collision_yaml}")
                if 'collision_spheres' in collision_config:
                    num_spheres = len(collision_config['collision_spheres'])
                    print(f"  包含 {num_spheres} 个link的碰撞球定义")
                print()
            except:
                print(f"  (无法解析YAML)")
                print()
    else:
        print(f"⚠️  碰撞球配置不存在: {collision_yaml}")
        print("  这可能导致师兄控制器无法初始化碰撞环境")
        print()

if __name__ == '__main__':
    find_constraint_additions()
    suggest_disable_collision()
    test_collision_config()

    print("==========================================")
    print("推荐测试步骤")
    print("==========================================")
    print()
    print("1. 先降低碰撞增益（不重新编译）")
    print("2. 测试师兄控制器是否能正常工作")
    print("3. 如果能，说明是碰撞约束的问题")
    print("4. 然后可以逐步调整参数找到平衡点")
    print()
