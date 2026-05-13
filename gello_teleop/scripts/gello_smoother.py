#!/usr/bin/env python3
from dataclasses import dataclass, field
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class GelloSmootherConfig:
    arm_id: str = 'panda'
    source_joint_topic: str = '/gello/joints_desired'
    target_joint_topic: str = '/joint_impedance/joints_desired'
    joint_state_topic: str = '/panda/joint_states'
    publish_hz: float = 100.0
    smooth_threshold: float = 0.5  # 误差大于此值时开始平滑（弧度）
    full_tracking_threshold: float = 0.05  # 误差小于此值时完全跟踪（弧度）
    max_velocity: float = 0.3  # 最大关节速度（弧度/秒）
    max_acceleration: float = 1.0  # 最大关节加速度（弧度/秒²）
    joint_limits: np.ndarray = field(
        default_factory=lambda: np.array([
            [-2.8973, 2.8973],
            [-1.7628, 1.7628],
            [-2.8973, 2.8973],
            [-3.0718, -0.0698],
            [-2.8973, 2.8973],
            [-0.0175, 3.7525],
            [-2.8973, 2.8973],
        ], dtype=float)
    )

    def __post_init__(self):
        if self.joint_limits.shape != (7, 2):
            raise ValueError('joint_limits must be shape (7, 2)')


class GelloSmootherNode(Node):
    def __init__(self, config: GelloSmootherConfig = None):
        super().__init__('gello_smoother')

        self.config = config if config is not None else GelloSmootherConfig()
        self.arm_id = self.config.arm_id
        self.smooth_threshold = float(self.config.smooth_threshold)
        self.full_tracking_threshold = float(self.config.full_tracking_threshold)
        self.max_velocity = float(self.config.max_velocity)
        self.max_acceleration = float(self.config.max_acceleration)
        self.publish_period = 1.0 / float(self.config.publish_hz)
        self.joint_limits = np.asarray(self.config.joint_limits, dtype=float)

        self.arm_joint_names = [f'{self.arm_id}_joint{i + 1}' for i in range(7)]
        
        # 当前状态
        self.q_current = None
        self.q_target_raw = None
        self.q_target_smooth = None
        self.q_velocity = np.zeros(7, dtype=float)
        
        # 状态标志
        self.joint_states_received = False
        self.raw_target_received = False
        self.in_smooth_mode = False

        # 订阅机械臂关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.config.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        # 订阅原始目标（来自GELLO）
        self.raw_target_sub = self.create_subscription(
            JointState,
            self.config.source_joint_topic,
            self.raw_target_callback,
            10,
        )

        # 发布平滑后的目标
        self.smooth_target_pub = self.create_publisher(
            JointState,
            self.config.target_joint_topic,
            10
        )

        # 定时器执行平滑计算
        self.timer = self.create_timer(self.publish_period, self.timer_callback)

        self.get_logger().info('Gello Smoother Node initialized.')
        self.get_logger().info(f'Smooth threshold: {self.smooth_threshold:.3f} rad')
        self.get_logger().info(f'Full tracking threshold: {self.full_tracking_threshold:.3f} rad')
        self.get_logger().info(f'Max velocity: {self.max_velocity:.3f} rad/s')

    def joint_state_callback(self, msg: JointState):
        try:
            indices = [msg.name.index(name) if name in msg.name else None 
                       for name in self.arm_joint_names]
            if not all(idx is not None for idx in indices):
                return

            q_arm = np.array([msg.position[idx] for idx in indices], dtype=float)
            self.q_current = q_arm
            self.joint_states_received = True
            
            # 初始化平滑目标为当前位置
            if self.q_target_smooth is None:
                self.q_target_smooth = q_arm.copy()
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse joint_states: {exc}')

    def raw_target_callback(self, msg: JointState):
        try:
            if len(msg.position) >= 7:
                self.q_target_raw = np.array(msg.position[:7], dtype=float)
                self.raw_target_received = True
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse raw target: {exc}')

    def timer_callback(self):
        if not self.joint_states_received or not self.raw_target_received:
            return

        if self.q_current is None or self.q_target_raw is None:
            return

        # 计算当前位置与原始目标的误差
        error = self.q_target_raw - self.q_current
        error_norm = np.linalg.norm(error)

        # 判断工作模式
        if error_norm < self.full_tracking_threshold:
            # 完全跟踪模式
            self.q_target_smooth = self.q_target_raw.copy()
            self.q_velocity = np.zeros(7)
            if self.in_smooth_mode:
                self.in_smooth_mode = False
                self.get_logger().info(f'Switched to full tracking mode (error: {error_norm:.3f} rad)')
        
        elif error_norm > self.smooth_threshold:
            # 平滑过渡模式
            self.in_smooth_mode = True
            self.smooth_transition()
        
        else:
            # 过渡区域，继续平滑
            if self.in_smooth_mode:
                self.smooth_transition()
            else:
                # 刚进入过渡区域，开始平滑
                self.in_smooth_mode = True
                self.get_logger().info(f'Switched to smooth mode (error: {error_norm:.3f} rad)')
                self.smooth_transition()

        # 应用关节限位
        self.q_target_smooth = np.clip(
            self.q_target_smooth, 
            self.joint_limits[:, 0], 
            self.joint_limits[:, 1]
        )

        # 发布平滑后的目标
        self.publish_target()

    def smooth_transition(self):
        """梯形速度规划平滑过渡"""
        # 计算目标方向
        error = self.q_target_raw - self.q_target_smooth
        error_norm = np.linalg.norm(error)
        
        if error_norm < 0.001:
            return

        direction = error / error_norm

        # 计算期望速度（受最大速度限制）
        desired_velocity = direction * min(self.max_velocity, error_norm / self.publish_period)

        # 计算速度变化（受最大加速度限制）
        velocity_change = desired_velocity - self.q_velocity
        max_velocity_change = self.max_acceleration * self.publish_period
        
        velocity_change = np.clip(
            velocity_change, 
            -max_velocity_change, 
            max_velocity_change
        )

        # 更新速度
        self.q_velocity += velocity_change
        self.q_velocity = np.clip(self.q_velocity, -self.max_velocity, self.max_velocity)

        # 更新位置
        self.q_target_smooth += self.q_velocity * self.publish_period

        # 防止超过目标
        remaining_error = self.q_target_raw - self.q_target_smooth
        if np.dot(remaining_error, self.q_velocity) < 0:
            # 已经超过目标，直接设置为目标
            self.q_target_smooth = self.q_target_raw.copy()
            self.q_velocity = np.zeros(7)

    def publish_target(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.arm_joint_names
        msg.position = self.q_target_smooth.tolist()
        msg.velocity = self.q_velocity.tolist()
        self.smooth_target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GelloSmootherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()