#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig

JOINT_LIMITS = np.array([
    [-2.8973, 2.8973],
    [-1.7628, 1.7628],
    [-2.8973, 2.8973],
    [-3.0718, -0.0698],
    [-2.8973, 2.8973],
    [-0.0175, 3.7525],
    [-2.8973, 2.8973],
])

# 控制模式常量
LEFT_ARM = 1
RIGHT_ARM = 2
BOTH_ARMS = 3

# 默认控制模式
CONTROL_MODE = BOTH_ARMS

# 默认关节位置（用于未控制的手臂）
DEFAULT_JOINTS = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])


class GelloFrankaBridge(Node):
    def __init__(self):
        super().__init__('gello_franka_bridge')

        dynamixel_config = DynamixelRobotConfig(
            joint_ids=(1, 2, 3, 4, 5, 6, 7),
            # 标定整合后的偏移（已包含物理装配偏移）
            joint_offsets=(
                1 * np.pi,           # 关节1: π
                1 * np.pi,           # 关节2: π  
                2 * np.pi,           # 关节3: 2π
                1 * np.pi,           # 关节4: π
                1 * np.pi,           # 关节5: π
                1 * np.pi,           # 关节6: π
                1.25 * np.pi,        # 关节7: 1.25π (5π/4)
            ),
            joint_signs=(1, -1, 1, 1, 1, -1, 1),
            gripper_config=None,
        )

        port_path = "/dev/ttyUSB0"

        try:
            self.gello_agent = GelloAgent(
                port=port_path,
                dynamixel_config=dynamixel_config,
                real=True
            )
        except Exception as e:
            error_msg = f"Failed to initialize Gello Agent: {e}"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)

        self.previous_joints = None
        self.get_logger().info("Calibration offset integrated into DynamixelRobotConfig.joint_offsets")

        self.joint_publisher = self.create_publisher(
            Float64MultiArray,
            '/dual_joint_impedance/joints_desired',
            10
        )

        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        try:
            gello_joints = self.gello_agent.act({})
            self.previous_joints = gello_joints

            if len(gello_joints) > 7:
                arm_joints = gello_joints[:7]
            elif len(gello_joints) == 7:
                arm_joints = gello_joints
            else:
                arm_joints = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

            # 标定已整合到底层 DynamixelRobotConfig.joint_offsets，此处无需额外修正

            arm_joints = np.clip(arm_joints, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
            arm_joints = arm_joints[:7]

            # 根据控制模式生成双臂数据
            if CONTROL_MODE == LEFT_ARM:
                # 只控制左臂，右臂保持默认位置
                dual_joints = list(arm_joints) + list(DEFAULT_JOINTS)
            elif CONTROL_MODE == RIGHT_ARM:
                # 只控制右臂，左臂保持默认位置
                dual_joints = list(DEFAULT_JOINTS) + list(arm_joints)
            else:  # BOTH_ARMS
                # 双臂都使用 GELLO 数据（镜像模式）
                dual_joints = list(arm_joints) + list(arm_joints)

            msg = Float64MultiArray()
            msg.data = dual_joints
            self.joint_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")

    def destroy_node(self):
        if hasattr(self.gello_agent, '_robot'):
            self.gello_agent._robot.set_torque_mode(False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    gello_bridge = None

    try:
        gello_bridge = GelloFrankaBridge()
        rclpy.spin(gello_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if gello_bridge is not None:
            gello_bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
