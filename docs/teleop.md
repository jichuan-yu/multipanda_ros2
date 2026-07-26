# 双臂机器人遥操作控制说明

本目录 (`src/multipanda_ros2/teleop/`) 提供了一套完整的遥操作控制脚本，支持通过**键盘、SpaceMouse、CSV轨迹回放**等方式实时控制双臂 Panda 机器人。

所有遥操作脚本使用**标准ROS2消息** `std_msgs/Float64MultiArray` 发送命令到 `/dualarm_teleop_cmd` 话题，与 `dualarm_mprc` 控制器无缝集成。

---

## 消息格式规范

### 话题与消息类型

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/dualarm_teleop_cmd` | `std_msgs/msg/Float64MultiArray` | 遥操作命令输入 |

### 数据格式

`Float64MultiArray` 的 `data` 字段包含 **17个元素**：

| 索引 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0-13 | data | float64[14] | 命令数据（根据命令类型不同） |
| 14 | command_type | float64 | 命令类型（0-5） |
| 15 | arm_selector | float64 | 手臂选择（1-3） |
| 16 | allow_safety_violation | float64 | 安全覆盖标志（0-1，可选） |

### 命令类型 (索引 14)

| 值 | 常量名 | 说明 | 数据格式 (索引0-13) |
|----|--------|------|---------------------|
| 0 | JOINT_POSITION_INCREMENT | 关节空间位置增量 | `[左臂7关节, 右臂7关节]` 全部14个位置 |
| 1 | JOINT_VELOCITY | 关节空间速度 | `[左臂7关节, 右臂7关节]` 全部14个位置 |
| 2 | TASK_SPACE_INCREMENT | 末端位姿增量 | `[左x,y,z,qx,qy,qz, 右x,y,z,qx,qy,qz, 0, 0]` 使用前12个 |
| 3 | TASK_SPACE_VELOCITY | 末端速度 | `[左vx,vy,vz,wx,wy,wz, 右vx,vy,vz,wx,wy,wz, 0, 0]` 使用前12个 |
| 4 | JOINT_POSITION | 关节空间绝对位置 | `[左臂7关节, 右臂7关节]` 全部14个绝对位置 |
| 5 | TASK_SPACE_POSE | 末端绝对位姿 | `[左x,y,z,qx,qy,qz,qw, 右x,y,z,qx,qy,qz,qw]` 全部14个 |

### 手臂选择 (索引 15)

| 值 | 常量名 | 说明 |
|----|--------|------|
| 1 | LEFT_ARM | 仅左臂 |
| 2 | RIGHT_ARM | 仅右臂 |
| 3 | BOTH_ARMS | 双臂 |

---

## 遥操作脚本列表

### 1. 基础类 (base_teleop.py)

所有遥操作脚本的基类，提供通用功能：

- 消息编码/解码
- 安全限位
- Panda关节限制

### 2. 键盘关节空间控制 (key_teleop_joint.py)

通过键盘控制机器人关节运动。

**使用方法**:
```bash
python3 src/multipanda_ros2/teleop/key_teleop_joint.py --step-size 0.002
```

**按键映射**:
| 按键 | 功能 |
|------|------|
| `1/2/3/4/5/6/7` | 关节1-7 正向增量 |
| `Q/W/E/R/T/Y/U` | 关节1-7 负向增量 |
| `Z/X/B` | 选择左臂/右臂/双臂 |
| `[/]` | 减小/增大步长 |

### 3. 键盘任务空间控制 (key_teleop_cartesian.py)

通过键盘控制机器人末端位姿。

**使用方法**:
```bash
python3 src/multipanda_ros2/teleop/key_teleop_cartesian.py --step-position 0.001 --step-rotation 0.01
```

**按键映射**:
| 按键 | 功能 |
|------|------|
| `W/S` | X轴方向（前/后） |
| `A/D` | Y轴方向（左/右） |
| `Q/E` | Z轴方向（上/下） |
| `J/L` | Roll（绕X轴旋转） |
| `I/K` | Pitch（绕Y轴旋转） |
| `U/O` | Yaw（绕Z轴旋转） |
| `Z/X/B` | 选择左臂/右臂/双臂 |

### 4. 键盘任务空间控制 + 脚本内IK (key_teleop_cartesian_absolute_ik.py)

`key_teleop_cartesian_absolute.py` 的变体。按键映射与笛卡尔积分逻辑完全一致，区别在于：**在脚本内部完成逆运动学（IK）求解，再向安全控制器发送关节空间绝对位置指令**（`command_type=4 JOINT_POSITION`），而不是发送任务空间位姿由控制器内部做 IK。

IK 方法与 `dualarm_mprc` 安全控制器 `DualArmSafeControllerSim::computeJointFromTaskSpace()` 一致：阻尼最小二乘（DLS）雅可比伪逆 `J# = Jᵀ(JJᵀ + λ²I)⁻¹`（`λ=0.01`），并以 `/joint_states` 中的实际关节角作为线性化点（与控制器一致），迭代至收敛。机器人模型（DH 参数、FK、雅可比）在 `teleop/panda_kinematics.py` 中，是对 `robot_kinematics.cpp` 的忠实移植。

**使用方法**:
```bash
python3 src/multipanda_ros2/teleop/key_teleop_cartesian_absolute_ik.py --step-position 0.001 --step-rotation 0.01
```

**按键映射**: 与 `key_teleop_cartesian_absolute.py` 完全相同（`W/S A/D Q/E` 平移，`J/L I/K U/O` 旋转，`Z/X/B` 选臂，`N/M` 夹爪，`[/]` 步长）。

**输出**: `command_type=4`，`data` 为 `[左臂7关节, 右臂7关节]` 共14个绝对关节位置；未被选中的臂保持其当前关节构型。

---

### 5. SpaceMouse 任务空间控制 (spacemouse_teleop_cartesian.py)

使用 3Dconnexion SpaceMouse 进行直观的六自由度控制。

**使用方法**:
```bash
python3 src/multipanda_ros2/teleop/spacemouse_teleop_cartesian.py --scale-translation 0.0000007
```

**按键映射**:
| 按键 | 功能 |
|------|------|
| 按钮1 | 选择左臂 |
| 按钮2 | 选择右臂 |

### 6. CSV 关节空间回放 (csv_teleop_joint.py)

从CSV文件读取关节轨迹并模拟遥操作命令发送。

**使用方法**:
```bash
python3 src/multipanda_ros2/teleop/csv_teleop_joint.py --csv-file trajectory.csv --loop
```

**CSV格式**:
```csv
timestamp,q1_left,q2_left,q3_left,q4_left,q5_left,q6_left,q7_left,q1_right,q2_right,q3_right,q4_right,q5_right,q6_right,q7_right
0.0,0.0,-0.785,0.0,-2.356,0.0,1.571,0.785,0.0,-0.785,0.0,-2.356,0.0,1.571,0.785
0.1,0.01,-0.78,0.0,-2.35,0.0,1.57,0.78,0.01,-0.78,0.0,-2.35,0.0,1.57,0.78
...
```

### 7. CSV 任务空间回放 (csv_teleop_cartesian.py)

从CSV文件读取末端轨迹并模拟遥操作命令发送。

**使用方法**:
```bash
python3 src/multipanda_ros2/teleop/csv_teleop_cartesian.py --csv-file cartesian_trajectory.csv --loop
```

**CSV格式**:
```csv
timestamp,x_left,y_left,z_left,qx_left,qy_left,qz_left,qw_left,x_right,y_right,z_right,qx_right,qy_right,qz_right,qw_right
0.0,0.307,0.26,0.487,0.0,0.0,0.0,1.0,0.307,-0.26,0.487,0.0,0.0,0.0,1.0
0.1,0.317,0.26,0.487,0.0,0.0,0.0,1.0,0.317,-0.26,0.487,0.0,0.0,0.0,1.0
...
```

---

## 安全参数

所有遥操作脚本都遵循以下安全限制：

| 参数 | 值 | 说明 |
|------|-----|------|
| `max_joint_increment` | 0.002 rad | 单次命令最大关节增量 |
| `max_position_increment` | 0.002 m | 单次命令最大位置增量 |
| `max_rotation_increment` | 0.01 rad | 单次命令最大旋转增量 |
| `publish_rate` | 100 Hz | 命令发布频率 |

这些限制与 `dualarm_mprc` 控制器中的约束保持一致，确保安全操作。

---

## Python 示例代码

### 关节空间控制示例

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

# 命令类型常量
JOINT_POSITION_INCREMENT = 0
LEFT_ARM = 1
RIGHT_ARM = 2
BOTH_ARMS = 3

class TeleopExample(Node):
    def __init__(self):
        super().__init__('teleop_example')
        self.teleop_pub = self.create_publisher(
            Float64MultiArray, '/dualarm_teleop_cmd', 10)
        
    def send_joint_command(self):
        """发送关节位置增量命令"""
        msg = Float64MultiArray()
        
        # 14个关节数据: [左臂7关节, 右臂7关节]
        joint_data = [0.001] * 7 + [0.001] * 7  # 每个关节增加0.001弧度
        
        # 格式: [数据14, 命令类型, 手臂选择, 安全标志]
        msg.data = joint_data + [JOINT_POSITION_INCREMENT, BOTH_ARMS, 0.0]
        
        self.teleop_pub.publish(msg)
        self.get_logger().info('Joint teleop command sent')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopExample()
    
    # 发送命令
    node.send_joint_command()
    
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 任务空间控制示例

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

# 命令类型常量
TASK_SPACE_INCREMENT = 2
LEFT_ARM = 1

class CartesianTeleopExample(Node):
    def __init__(self):
        super().__init__('cartesian_teleop_example')
        self.teleop_pub = self.create_publisher(
            Float64MultiArray, '/dualarm_teleop_cmd', 10)
        
    def send_cartesian_command(self):
        """发送任务空间增量命令"""
        msg = Float64MultiArray()
        
        # 12个任务空间数据: [左x,y,z,qx,qy,qz, 右x,y,z,qx,qy,qz]
        # 沿X轴正方向移动1mm，无旋转
        task_data = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,  # 左臂
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 右臂
        
        # 格式: [数据12, 填充2个0, 命令类型, 手臂选择, 安全标志]
        msg.data = task_data + [0.0, 0.0] + [TASK_SPACE_INCREMENT, LEFT_ARM, 0.0]
        
        self.teleop_pub.publish(msg)
        self.get_logger().info('Cartesian teleop command sent')

def main(args=None):
    rclpy.init(args=args)
    node = CartesianTeleopExample()
    node.send_cartesian_command()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 常见问题

### 1. 机器人不响应遥操作命令

**检查项**:
- 确认 `dualarm_mprc` 控制器正在运行
- 确认控制器处于 `TELEOPERATING` 或 `TRACKING` 状态
- 检查 `/dualarm_teleop_cmd` 话题是否正确连接

### 2. 命令被拒绝

**可能原因**:
- 增量超过安全限制（关节: 0.002rad, 位置: 0.002m）
- 接近关节限位
- 碰撞避免约束激活

### 3. CSV回放不同步

**解决方法**:
- 确保 CSV 中的时间戳是单调递增的
- 检查 `--rate` 参数是否与控制器频率（100Hz）匹配
- 使用 `--loop` 参数进行循环测试

---

## 相关文档

- [dualarm_mprc 遥操作文档](../../dualarm_mprc/docs/tele_operation.md) - 控制器端遥操作实现详情
- [主文档](main.md) - 项目整体说明
