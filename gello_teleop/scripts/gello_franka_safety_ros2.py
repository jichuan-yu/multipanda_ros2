#!/usr/bin/env python3
"""
GELLO dual-arm bridge routed THROUGH the dual-arm safety controller.

Sibling of ``gello_franka_ros2.py`` (which publishes raw 14-DOF absolute joints
straight to the low-level joint impedance topic, bypassing any safety layer).
This version reuses the exact same GELLO I/O (agents, smoothers, grippers) but
publishes absolute joint positions as ``command_type=4`` (JOINT_POSITION) on
``/dualarm_teleop_cmd`` so that ``main_sim_node`` (dualarm_mprc safety
controller) runs clamping, collision avoidance, relative-pose constraints and
QP before the joint impedance controller is fed.

Topic flow (sim):

    GELLO -> smoother -> /dualarm_teleop_cmd (type=4, arm_selector)
           -> main_sim_node (safety controller)
           -> /mj_{left,right}/joints_desired
           -> dual_joint_impedance_controller (MuJoCo sim)

Message format (Float64MultiArray, see teleop/base_teleop.py):
    [left_q(7), right_q(7), command_type=4, arm_selector, allow_safety_violation]

Requirements:
    - ``main_sim_node`` running (start it from the workspace root, it reads
      config YAMLs via relative paths)
    - ``/joint_states`` streaming (used to initialize the smoothers)

Usage:
    python3 src/multipanda_ros2/gello_teleop/scripts/gello_franka_safety_ros2.py --control-mode 3

    (No virtualenv activation needed on this host: ``gello`` / ``dynamixel-sdk`` are
     installed into the system Python via ``pip install -e ~/gello_software``.)
"""

from dataclasses import dataclass, field
import os
import sys
from pathlib import Path

import numpy as np
import rclpy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Add parent directory to path for imports (for direct execution)
if __name__ == '__main__':
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Teleop-style base node (publishes /dualarm_teleop_cmd, has create_teleop_command)
try:
    from teleop.base_teleop import BaseTeleopNode
except ImportError:
    from base_teleop import BaseTeleopNode

# Shared GELLO I/O helpers / constants from the original bridge script
from gello_franka_ros2 import (
    load_gello_config,
    MAX_GRIPPER_OPEN,
    LEFT_ARM,
    RIGHT_ARM,
    BOTH_ARMS,
    SIM_MODE,
    REAL_MODE,
)
from gello_smoother import Smoother


@dataclass
class GelloFrankaSafetyConfig:
    """Configuration for the safety-routed GELLO dual-arm bridge."""

    arm_id: str = 'mj'
    joint_state_topic: str = '/joint_states'
    teleop_cmd_topic: str = '/dualarm_teleop_cmd'
    left_gripper_topic: str = '/mj_left_gripper/width_desired'
    right_gripper_topic: str = '/mj_right_gripper/width_desired'
    left_gello_config_path: str = ''
    right_gello_config_path: str = ''
    publish_hz: float = 50.0
    control_mode: int = BOTH_ARMS
    hardware_mode: int = SIM_MODE  # 0: 仿真模式, 1: 真机模式
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
class GelloFrankaSafetyNode(BaseTeleopNode):
    """GELLO dual-arm bridge publishing absolute joints to the safety controller."""

    def __init__(self, config: GelloFrankaSafetyConfig = None):
        self.config = config if config is not None else GelloFrankaSafetyConfig()

        super().__init__(
            'gello_franka_safety_bridge',
            publish_rate=self.config.publish_hz,
            use_sim_time=(self.config.hardware_mode == SIM_MODE),
        )

        self.arm_id = self.config.arm_id
        self.joint_state_topic = self.config.joint_state_topic
        self.control_mode = self.config.control_mode
        self.hardware_mode = self.config.hardware_mode
        self.publish_hz = float(self.config.publish_hz)
        self.joint_limits = np.asarray(self.config.joint_limits, dtype=float)
        self.default_joint_target = np.asarray(self.config.default_joint_target, dtype=float)

        self.left_joint_names = [f'{self.arm_id}_left_joint{i + 1}' for i in range(7)]
        self.right_joint_names = [f'{self.arm_id}_right_joint{i + 1}' for i in range(7)]

        self.q_left_current = None
        self.q_right_current = None
        self.joint_states_received = False
        self._missing_joint_state_logged = False

        # 两个独立的平滑器（与原始桥接脚本一致）
        self.left_smoother = Smoother(
            publish_hz=self.publish_hz,
            max_velocity=0.1,
            position_tolerance=0.05,
            joint_limits=self.joint_limits,
            logger=self.get_logger()
        )
        self.right_smoother = Smoother(
            publish_hz=self.publish_hz,
            max_velocity=0.1,
            position_tolerance=0.05,
            joint_limits=self.joint_limits,
            logger=self.get_logger()
        )

        self.left_smoother_initialized = False
        self.right_smoother_initialized = False
        self.latest_left_raw_target = None
        self.latest_right_raw_target = None
        # 最近一次平滑输出：GELLO 单次读取失败时保持上一拍命令（避免控制器超时/跳变）
        self.latest_left_smoothed = None
        self.latest_right_smoothed = None

        # 订阅当前关节状态（用于平滑器初始化和保持）
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        # 夹爪发布器（与原始桥接脚本一致）
        if self.hardware_mode == SIM_MODE:
            self.left_gripper_publisher = self.create_publisher(
                Float64, self.config.left_gripper_topic, 10)
            self.right_gripper_publisher = self.create_publisher(
                Float64, self.config.right_gripper_topic, 10)
        else:
            self.left_gripper_publisher = self.create_publisher(
                Float64, '/left_gripper/width_desired', 10)
            self.right_gripper_publisher = self.create_publisher(
                Float64, '/right_gripper/width_desired', 10)
        # 加载 GELLO 配置（默认 config/ 目录，可命令行覆盖）
        script_dir = Path(__file__).resolve().parent
        left_config_path = (
            self.config.left_gello_config_path
            if self.config.left_gello_config_path
            else script_dir.parent / 'config' / 'gello_arm1.yaml'
        )
        right_config_path = (
            self.config.right_gello_config_path
            if self.config.right_gello_config_path
            else script_dir.parent / 'config' / 'gello_arm2.yaml'
        )

        self.get_logger().info(f"Loading left arm GELLO config from: {left_config_path}")
        left_gello_config = load_gello_config(str(left_config_path))
        self.get_logger().info(f"Loading right arm GELLO config from: {right_config_path}")
        right_gello_config = load_gello_config(str(right_config_path))

        # 创建双臂 GELLO Agent（复用 gello 库，与原始脚本一致）
        from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig

        left_dynamixel_config = DynamixelRobotConfig(
            joint_ids=left_gello_config["joint_ids"],
            joint_offsets=left_gello_config["joint_offsets"],
            joint_signs=left_gello_config["joint_signs"],
            gripper_config=left_gello_config["gripper_config"],
        )
        right_dynamixel_config = DynamixelRobotConfig(
            joint_ids=right_gello_config["joint_ids"],
            joint_offsets=right_gello_config["joint_offsets"],
            joint_signs=right_gello_config["joint_signs"],
            gripper_config=right_gello_config["gripper_config"],
        )

        try:
            self.left_gello_agent = GelloAgent(
                port=left_gello_config["port"],
                dynamixel_config=left_dynamixel_config,
                real=True,
            )
            self.get_logger().info(f"Left arm GELLO Port: {left_gello_config['port']}")
        except Exception as exc:
            raise RuntimeError(f'Failed to initialize left arm Gello Agent: {exc}')

        try:
            self.right_gello_agent = GelloAgent(
                port=right_gello_config["port"],
                dynamixel_config=right_dynamixel_config,
                real=True,
            )
            self.get_logger().info(f"Right arm GELLO Port: {right_gello_config['port']}")
        except Exception as exc:
            raise RuntimeError(f'Failed to initialize right arm Gello Agent: {exc}')

        # 读取初始位置（用于日志，平滑器初始化仍以实际机器人关节为准）
        left_gello_joints = self.left_gello_agent.act({})
        if left_gello_joints is None or len(left_gello_joints) < 7:
            raise RuntimeError('Failed to read initial left arm Gello joint positions')
        left_q_first = np.asarray(left_gello_joints[:7], dtype=float)
        self.get_logger().info(
            'Left arm first Gello joint read (after offset correction): ' +
            ', '.join(f'{float(val):.4f}' for val in left_q_first)
        )

        right_gello_joints = self.right_gello_agent.act({})
        if right_gello_joints is None or len(right_gello_joints) < 7:
            raise RuntimeError('Failed to read initial right arm Gello joint positions')
        right_q_first = np.asarray(right_gello_joints[:7], dtype=float)
        self.get_logger().info(
            'Right arm first Gello joint read (after offset correction): ' +
            ', '.join(f'{float(val):.4f}' for val in right_q_first)
        )

        self.create_timer(1.0 / max(self.publish_hz, 1.0), self.timer_callback)
        self.joint_print_timer = self.create_timer(1.0, self.joint_print_callback)

        self.get_logger().info(
            f'Gello Franka safety bridge initialized '
            f'(control_mode={self.control_mode}, publish_hz={self.publish_hz}, '
            f'topic={self.config.teleop_cmd_topic})'
        )
    # ------------------------------------------------------------------ #
    # 关节状态回调：初始化平滑器（起点锚定到真实机器人关节位置）
    # ------------------------------------------------------------------ #
    def joint_state_callback(self, msg: JointState):
        try:
            name_to_pos = dict(zip(msg.name, msg.position))

            left_indices = [name_to_pos.get(n) for n in self.left_joint_names]
            right_indices = [name_to_pos.get(n) for n in self.right_joint_names]

            # 更新左臂状态
            if all(idx is not None for idx in left_indices):
                self.q_left_current = np.array(left_indices, dtype=float)

                if self.q_left_current is not None and not self.left_smoother_initialized:
                    self.left_smoother.set_actual_position(self.q_left_current)
                    self.left_smoother.set_initial_position(self.q_left_current)
                    initial_target = (
                        self.latest_left_raw_target
                        if self.latest_left_raw_target is not None
                        else self.q_left_current.copy()
                    )
                    self.left_smoother.set_target(initial_target)
                    self.left_smoother.set_follower_enabled(True)
                    self.left_smoother_initialized = True
                    self.get_logger().info(
                        'Left arm smoother initialized from actual robot joint positions.')
            elif not self._missing_joint_state_logged:
                self.get_logger().warn(f'Left arm joint state missing: {self.left_joint_names}')

            # 更新右臂状态
            if all(idx is not None for idx in right_indices):
                self.q_right_current = np.array(right_indices, dtype=float)

                if self.q_right_current is not None and not self.right_smoother_initialized:
                    self.right_smoother.set_actual_position(self.q_right_current)
                    self.right_smoother.set_initial_position(self.q_right_current)
                    initial_target = (
                        self.latest_right_raw_target
                        if self.latest_right_raw_target is not None
                        else self.q_right_current.copy()
                    )
                    self.right_smoother.set_target(initial_target)
                    self.right_smoother.set_follower_enabled(True)
                    self.right_smoother_initialized = True
                    self.get_logger().info(
                        'Right arm smoother initialized from actual robot joint positions.')
            elif not self._missing_joint_state_logged:
                self.get_logger().warn(f'Right arm joint state missing: {self.right_joint_names}')

            # 设置初始化完成标志
            if not self.joint_states_received:
                if ((self.control_mode == LEFT_ARM and self.q_left_current is not None) or
                        (self.control_mode == RIGHT_ARM and self.q_right_current is not None) or
                        (self.control_mode == BOTH_ARMS and
                         self.q_left_current is not None and self.q_right_current is not None)):
                    self.joint_states_received = True

        except Exception as exc:
            self.get_logger().warn(f'Failed to parse joint_states: {exc}')
    # ------------------------------------------------------------------ #
    # 主循环：读 GELLO -> 平滑 -> 发 /dualarm_teleop_cmd (type=4)
    # ------------------------------------------------------------------ #
    def timer_callback(self):
        try:
            # 读取左臂数据
            left_gello_joints = self.left_gello_agent.act({})
            # 读取右臂数据
            right_gello_joints = self.right_gello_agent.act({})

            left_smoothed = None
            right_smoothed = None
            left_gripper_val = None
            right_gripper_val = None

            # 处理左臂数据
            if left_gello_joints is not None and len(left_gello_joints) >= 7:
                # 不做 joint_limits 限幅：直接使用 GELLO 标定后的关节读数
                left_q_target_raw = np.asarray(left_gello_joints[:7], dtype=float)
                self.latest_left_raw_target = left_q_target_raw

                if self.left_smoother_initialized and self.q_left_current is not None:
                    self.left_smoother.set_actual_position(self.q_left_current)
                    self.left_smoother.set_target(left_q_target_raw)
                    left_smoothed = self.left_smoother.update()
                    self.latest_left_smoothed = left_smoothed

                # 处理夹爪数据
                if len(left_gello_joints) >= 8:
                    left_gripper_val = left_gello_joints[7]
            else:
                # 读取失败：保持上一拍平滑输出，避免控制器超时或目标跳变
                left_smoothed = self.latest_left_smoothed

            # 处理右臂数据
            if right_gello_joints is not None and len(right_gello_joints) >= 7:
                # 不做 joint_limits 限幅：直接使用 GELLO 标定后的关节读数
                right_q_target_raw = np.asarray(right_gello_joints[:7], dtype=float)
                self.latest_right_raw_target = right_q_target_raw

                if self.right_smoother_initialized and self.q_right_current is not None:
                    self.right_smoother.set_actual_position(self.q_right_current)
                    self.right_smoother.set_target(right_q_target_raw)
                    right_smoothed = self.right_smoother.update()
                    self.latest_right_smoothed = right_smoothed

                # 处理夹爪数据
                if len(right_gello_joints) >= 8:
                    right_gripper_val = right_gello_joints[7]
            else:
                # 读取失败：保持上一拍平滑输出
                right_smoothed = self.latest_right_smoothed
            # 尚未初始化/读取失败时的兜底：优先保持实际位置，其次默认位姿
            if left_smoothed is None:
                left_smoothed = (
                    self.q_left_current.copy()
                    if self.q_left_current is not None
                    else self.default_joint_target
                )
            if right_smoothed is None:
                right_smoothed = (
                    self.q_right_current.copy()
                    if self.q_right_current is not None
                    else self.default_joint_target
                )

            # 按控制模式组装双臂目标（未选中的臂填默认位姿，控制器会忽略对应索引）
            if self.control_mode == LEFT_ARM:
                left_cmd = left_smoothed
                right_cmd = self.default_joint_target
            elif self.control_mode == RIGHT_ARM:
                left_cmd = self.default_joint_target
                right_cmd = right_smoothed
            else:
                left_cmd = left_smoothed
                right_cmd = right_smoothed

            # 通过安全控制器发布绝对关节位置 (command_type=4 JOINT_POSITION)
            if self.joint_states_received:
                msg = self.create_teleop_command(
                    arm_selector=self.control_mode,
                    command_type=self.JOINT_POSITION,
                    left_joint_delta=left_cmd,
                    right_joint_delta=right_cmd,
                    allow_safety_violation=False,
                )
                self.teleop_pub.publish(msg)

            # 处理夹爪指令（与原始桥接脚本一致）
            if left_gripper_val is not None and (self.control_mode == LEFT_ARM or self.control_mode == BOTH_ARMS):
                left_gripper_width = MAX_GRIPPER_OPEN * (1 - left_gripper_val)
                left_gripper_width = np.clip(left_gripper_width, 0.0, MAX_GRIPPER_OPEN)
                msg_gripper = Float64()
                msg_gripper.data = float(left_gripper_width)
                if self.left_gripper_publisher:
                    self.left_gripper_publisher.publish(msg_gripper)

            if right_gripper_val is not None and (self.control_mode == RIGHT_ARM or self.control_mode == BOTH_ARMS):
                right_gripper_width = MAX_GRIPPER_OPEN * (1 - right_gripper_val)
                right_gripper_width = np.clip(right_gripper_width, 0.0, MAX_GRIPPER_OPEN)
                msg_gripper = Float64()
                msg_gripper.data = float(right_gripper_width)
                if self.right_gripper_publisher:
                    self.right_gripper_publisher.publish(msg_gripper)

        except Exception as exc:
            self.get_logger().error(f'Error publishing joint command: {exc}')
    # ------------------------------------------------------------------ #
    # 1Hz 调试打印
    # ------------------------------------------------------------------ #
    def joint_print_callback(self):
        try:
            if not self.joint_states_received:
                self.get_logger().info('Waiting for joint states to initialize smoothers...')
                return

            left_str = 'N/A'
            if self.latest_left_raw_target is not None:
                left_str = ', '.join(f'{j:.4f}' for j in self.latest_left_raw_target[:7])
            right_str = 'N/A'
            if self.latest_right_raw_target is not None:
                right_str = ', '.join(f'{j:.4f}' for j in self.latest_right_raw_target[:7])

            self.get_logger().info(
                f'[Mode: {self.control_mode}] '
                f'Left raw: [{left_str}] | Right raw: [{right_str}]'
            )
        except Exception as exc:
            self.get_logger().warn(f'joint_print_callback error: {exc}')

    def destroy_node(self):
        if hasattr(self, 'left_gello_agent') and hasattr(self.left_gello_agent, '_robot'):
            try:
                self.left_gello_agent._robot.set_torque_mode(False)
            except Exception:
                pass

        if hasattr(self, 'right_gello_agent') and hasattr(self.right_gello_agent, '_robot'):
            try:
                self.right_gello_agent._robot.set_torque_mode(False)
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        description='GELLO Franka Dual Arm ROS2 Bridge (routed through safety controller)')
    parser.add_argument('--left-config', type=str, default=None,
                        help='Path to left arm GELLO YAML config file')
    parser.add_argument('--right-config', type=str, default=None,
                        help='Path to right arm GELLO YAML config file')
    parser.add_argument('--control-mode', type=int, default=BOTH_ARMS,
                        help='Control mode: 1=LEFT_ARM, 2=RIGHT_ARM, 3=BOTH_ARMS')
    parser.add_argument('--hardware-mode', type=int, default=SIM_MODE,
                        help='Hardware mode: 0=SIM_MODE (simulation), 1=REAL_MODE (real hardware)')
    parser.add_argument('--publish-hz', type=float, default=50.0,
                        help='Command publish rate in Hz (default: 50.0)')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)
    node = None

    try:
        config_kwargs = {}
        if parser_args.control_mode in [LEFT_ARM, RIGHT_ARM, BOTH_ARMS]:
            config_kwargs['control_mode'] = parser_args.control_mode
        if parser_args.hardware_mode in [SIM_MODE, REAL_MODE]:
            config_kwargs['hardware_mode'] = parser_args.hardware_mode
        config_kwargs['publish_hz'] = parser_args.publish_hz
        if parser_args.left_config:
            config_kwargs['left_gello_config_path'] = parser_args.left_config
        if parser_args.right_config:
            config_kwargs['right_gello_config_path'] = parser_args.right_config

        config = GelloFrankaSafetyConfig(**config_kwargs)
        node = GelloFrankaSafetyNode(config=config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except FileNotFoundError as e:
        print(f'Error: {e}')
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()






