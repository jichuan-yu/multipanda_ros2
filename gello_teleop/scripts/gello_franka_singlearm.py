#!/usr/bin/env python3
from dataclasses import dataclass, field
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig


@dataclass
class GelloFrankaSingleArmConfig:
    arm_id: str = 'panda'
    joint_state_topic: str = '/panda/joint_states'
    target_joint_topic: str = '/gello/joints_desired'  # 发布到中间话题，由平滑脚本处理
    gello_port: str = '/dev/ttyUSB0'
    publish_hz: float = 50.0
    startup_joint_threshold: float = 1.0  # 放宽启动检查，平滑脚本会处理差异
    joint_offsets: np.ndarray = field(
        default_factory=lambda: np.array([
            1.0 * np.pi,
            1.0 * np.pi,
            0.0 * np.pi,
            1.0 * np.pi,
            1.0 * np.pi,
            1.0 * np.pi,
            0.25 * np.pi,
        ], dtype=float)
    )
    joint_signs: np.ndarray = field(
        default_factory=lambda: np.array([1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0], dtype=float)
    )
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
    default_joint_target: np.ndarray = field(
        default_factory=lambda: np.array([
            0.0,
            -0.785398,
            0.0,
            -2.35619,
            0.0,
            1.5708,
            0.785398,
        ], dtype=float)
    )

    def __post_init__(self):
        if self.joint_offsets.shape != (7,):
            raise ValueError('joint_offsets must be length 7')
        if self.joint_signs.shape != (7,):
            raise ValueError('joint_signs must be length 7')
        if self.joint_limits.shape != (7, 2):
            raise ValueError('joint_limits must be shape (7, 2)')


class GelloFrankaSingleArmNode(Node):
    INIT_TIMEOUT_SEC = 2.0

    def __init__(self, config: GelloFrankaSingleArmConfig = None):
        super().__init__('gello_franka_singlearm')

        self.config = config if config is not None else GelloFrankaSingleArmConfig()
        self.arm_id = self.config.arm_id
        self.joint_state_topic = self.config.joint_state_topic
        self.target_joint_topic = self.config.target_joint_topic
        self.publish_hz = float(self.config.publish_hz)
        self.startup_joint_threshold = float(self.config.startup_joint_threshold)
        self.joint_offsets = np.asarray(self.config.joint_offsets, dtype=float)
        self.joint_signs = np.asarray(self.config.joint_signs, dtype=float)
        self.joint_limits = np.asarray(self.config.joint_limits, dtype=float)
        self.default_joint_target = np.asarray(self.config.default_joint_target, dtype=float)

        self.arm_joint_names = [f'{self.arm_id}_joint{i + 1}' for i in range(7)]
        self.q_current = None
        self.joint_states_received = False
        self._missing_joint_state_logged = False

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.joint_publisher = self.create_publisher(JointState, self.target_joint_topic, 10)

        init_start = self.get_clock().now().nanoseconds / 1e9
        while (not self.joint_states_received and (self.get_clock().now().nanoseconds / 1e9 - init_start) < self.INIT_TIMEOUT_SEC):
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.joint_states_received:
            raise RuntimeError(
                f'Timeout waiting for {self.joint_state_topic} (>{self.INIT_TIMEOUT_SEC:.1f}s). '
                'Cannot initialize startup sync check.'
            )

        dynamixel_config = DynamixelRobotConfig(
            joint_ids=(1, 2, 3, 4, 5, 6, 7),
            joint_offsets=tuple(self.joint_offsets.tolist()),
            joint_signs=tuple(self.joint_signs.tolist()),
            gripper_config=None,
        )

        try:
            self.gello_agent = GelloAgent(
                port=self.config.gello_port,
                dynamixel_config=dynamixel_config,
                real=True,
            )
        except Exception as exc:
            raise RuntimeError(f'Failed to initialize Gello Agent: {exc}')

        self._startup_joint_check()

        self.get_logger().info('Gello Franka single-arm teleop node initialized.')

    def joint_state_callback(self, msg: JointState):
        try:
            indices = [msg.name.index(name) if name in msg.name else None for name in self.arm_joint_names]
            if not all(idx is not None for idx in indices):
                if not self._missing_joint_state_logged:
                    missing = [self.arm_joint_names[i] for i, idx in enumerate(indices) if idx is None]
                    self.get_logger().warn(
                        'JointState names do not match expected Panda joints. '
                        f'Missing: {missing}. Received names: {list(msg.name)}'
                    )
                    self._missing_joint_state_logged = True
                return

            q_arm = np.array([msg.position[idx] for idx in indices], dtype=float)
            self.q_current = q_arm
            self.joint_states_received = True
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse joint_states: {exc}')

    def _startup_joint_check(self):
        if self.q_current is None:
            raise RuntimeError('No current robot joint state available for startup check.')

        gello_joints = self.gello_agent.act({})
        if gello_joints is None or len(gello_joints) < 7:
            raise RuntimeError('Failed to read initial Gello joint positions for startup check.')

        q_desired = np.asarray(gello_joints[:7], dtype=float)
        difference = np.abs(q_desired - self.q_current)
        default_difference = np.abs(q_desired - self.default_joint_target)

        self.get_logger().info(
            'First Gello joint read: ' + ', '.join(f'{float(val):.4f}' for val in q_desired)
        )
        self.get_logger().info(
            'Deviation to default joint target: ' + ', '.join(
                f'{float(diff):.4f} rad' for diff in default_difference
            )
        )

        if np.any(difference > self.startup_joint_threshold):
            diff_str = ', '.join(
                f'{name}: {float(diff):.4f} rad'
                for name, diff in zip(self.arm_joint_names, difference)
            )
            raise RuntimeError(
                'Gello and Franka joint states differ too much at startup. '
                f'Max difference: {float(np.max(difference)):.4f} rad. '
                f'Per-joint differences: {diff_str}. '
                'Check controller limits, robot pose, and Gello calibration before enabling motion.'
            )

        self.get_logger().info(
            f'Startup joint sync check passed. Max difference: {float(np.max(difference)):.4f} rad.'
        )

    def _publish_joint_command(self, q_desired):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.arm_joint_names
        msg.position = q_desired.tolist()
        msg.velocity = [0.0] * len(self.arm_joint_names)
        self.joint_publisher.publish(msg)

    def timer_callback(self):
        if self.q_current is None:
            return

        try:
            gello_joints = self.gello_agent.act({})
            if gello_joints is None or len(gello_joints) < 7:
                self.get_logger().warn('Invalid Gello joint reading, skipping publish.')
                return

            q_desired = np.asarray(gello_joints[:7], dtype=float)
            self._publish_joint_command(q_desired)
        except Exception as exc:
            self.get_logger().error(f'Error publishing joint command: {exc}')

    def destroy_node(self):
        if hasattr(self.gello_agent, '_robot'):
            try:
                self.gello_agent._robot.set_torque_mode(False)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GelloFrankaSingleArmNode()
        node.create_timer(1.0 / max(node.publish_hz, 1.0), node.timer_callback)
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
