# 双臂/单臂夹爪 (Franka Gripper) 控制说明

为了方便在外部宿主机（例如 ROS 2 Jazzy 环境）控制仿真容器内（Humble）的机械臂夹爪，且**无需在宿主机端编译和安装任何自定义的 `franka_msgs`**，本项目在控制层实现了一个高级动作桥接节点 (`gripper_action_bridge`)。

该节点已集成至主 launch 文件，随 `my_task_sim.launch.py` 自动启动。在外部主机，你可以直接通过终端命令或 Python 代码发布**原生标准 `std_msgs` 话题**来完成所有的高频或精细夹爪操作。

---

## 控制话题与标准消息格式

此处以**左臂 (`left`)**为例。针对右臂，请将各话题名中的 `left` 替换为 `right`，如 `/mj_right_gripper/width_desired`。

### 1. 简单开合移动 (Move)
* **话题**: `/mj_left_gripper/width_desired`
* **消息类型**: `std_msgs/msg/Float64`
* **功能说明**: 控制夹爪以默认速度 (0.1 m/s) 移动到指定的绝对目标宽度（无特定力控逻辑）。现有自带的键盘节点就是调用的此接口。

### 2. 精细力控抓取 (Grasp)
* **话题**: `/mj_left_gripper/grasp_desired`
* **消息类型**: `std_msgs/msg/Float64MultiArray`
* **功能说明**: 执行含期望抓取力的高级抓取指令。你需要发布一个按以下顺序排列的 Float64 数组：
  - `data[0]`: `width` (目标闭合宽度，单位 m) [必填]
  - `data[1]`: `speed` (闭合速度，单位 m/s，默认为 0.1) [选填]
  - `data[2]`: `force` (施加的恒定抓取力，单位 N，代表力控强度，默认为 10.0) [选填]
  - `data[3]`: `epsilon_inner` (内侧容差，单位 m，默认为 0.005) [选填]
  - `data[4]`: `epsilon_outer` (外侧容差，单位 m，默认为 0.005) [选填]
*注：发布数组时只需提供已知参数，例如发送 `[0.02, 0.05, 40.0]` 将以 5cm/s 速度闭合到 2cm，并在接触时施加 40N 抓取力。未填写的后续参数由底层默认补充。*

### 3. 夹爪归零复位标定 (Homing)
* **话题**: `/mj_left_gripper/homing_desired`
* **消息类型**: `std_msgs/msg/Bool`
* **功能说明**: 发布 `data: True` 时，将触发夹爪执行硬件重置动作（通常包含完全闭合并完全张开的操作），用于寻找并校准极大极限坐标与力矩基准。

### 4. 操作急停 (Stop)
* **话题**: `/mj_left_gripper/stop_desired`
* **消息类型**: `std_msgs/msg/Bool`
* **功能说明**: 发布 `data: True` 时，将直接跨层触发服务进行紧急制动，打断当前所有夹爪自身的物理移动和抓取行为。

---

## Python 示例代码 (外部宿主机直接运行)

下面是一段可在您的外侧 Jazzy 系统中运行的纯原生 ROS2 代码片段示例，展现了相关节点的话题声明与控制触发：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Bool

class GripperControllerNode(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        
        # 声明各个动作的话题发布者
        self.left_move_pub = self.create_publisher(Float64, '/mj_left_gripper/width_desired', 10)
        self.left_grasp_pub = self.create_publisher(Float64MultiArray, '/mj_left_gripper/grasp_desired', 10)
        self.left_homing_pub = self.create_publisher(Bool, '/mj_left_gripper/homing_desired', 10)
        self.left_stop_pub = self.create_publisher(Bool, '/mj_left_gripper/stop_desired', 10)

    def open_gripper(self):
        """完全张开夹爪"""
        self.get_logger().info("Opening gripper using Move...")
        msg = Float64()
        msg.data = 0.08  # Franka夹爪最大张口为 8cm
        self.left_move_pub.publish(msg)

    def grasp_object(self):
        """启用力度控制夹取目标物"""
        self.get_logger().info("Grasping object with Force Control...")
        msg = Float64MultiArray()
        # 数据含义 -> [目标宽度(m), 闭合速度(m/s), 保持力(N)]
        msg.data = [0.03, 0.05, 30.0]
        self.left_grasp_pub.publish(msg)

    def homing_gripper(self):
        """标定/复位"""
        self.get_logger().info("Running gripper Homing...")
        msg = Bool()
        msg.data = True
        self.left_homing_pub.publish(msg)

    def stop_gripper(self):
        """中断停止"""
        self.get_logger().info("Stopping gripper operations...")
        msg = Bool()
        msg.data = True
        self.left_stop_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperControllerNode()
    
    # 在您的控制流程或状态机中调用:
    # node.homing_gripper()
    # node.open_gripper()
    # node.grasp_object()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
