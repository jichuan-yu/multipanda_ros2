#!/usr/bin/env python3
"""GELLO 单臂遥操作桥接节点（对接新版 JointImpedanceController）。

数据流：
    GELLO -> 平滑器 -> /panda/joints_desired (sensor_msgs/JointState)
           -> joint_impedance_controller (franka_example_controllers/JointImpedanceController)
    夹爪   -> /panda_gripper/width_desired (std_msgs/Float64)

与旧版脚本的区别：目标话题由 ``/joint_impedance/joints_desired`` 改为
``/panda/joints_desired``；关节反馈话题 ``/panda/joint_states`` 保持不变
（新版 JointImpedanceController 在 ``on_activate`` 后按 publish_rate 发布）。

用法：
    cd src/multipanda_ros2/gello_teleop/scripts
    python3 gello_franka_singlearm.py
    python3 gello_franka_singlearm.py --no-smoother          # 直接控制，不做平滑
    python3 gello_franka_singlearm.py --target-topic /panda/joints_desired
"""

from dataclasses import dataclass, field
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig  # noqa: E402
from gello_smoother import Smoother  # noqa: E402


MAX_GRIPPER_OPEN = 0.08
STARTUP_FEEDBACK_WAIT_SEC = 10.0


def load_gello_config(config_path: str):
    """Load GELLO configuration from YAML file."""
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    agent_config = config.get("agent", {})
    dynamixel_config = agent_config.get("dynamixel_config", {})

    joint_ids = tuple(dynamixel_config.get("joint_ids", (1, 2, 3, 4, 5, 6, 7)))
    gripper_config = dynamixel_config.get("gripper_config", None)
    if gripper_config is not None:
        if not isinstance(gripper_config, (list, tuple)) or len(gripper_config) != 3:
            raise ValueError(
                "Invalid gripper_config in GELLO YAML: expected [gripper_joint_id, open_degrees, closed_degrees]."
            )
        gripper_id = int(gripper_config[0])
        if gripper_id in joint_ids:
            raise ValueError(
                f"Invalid GELLO config: gripper_config joint ID {gripper_id} duplicates arm joint IDs {joint_ids}. "
                "Use a separate gripper ID or remove gripper_config if gripper is not a separate servo."
            )

    return {
        "port": agent_config.get("port", "/dev/ttyUSB0"),
        "joint_ids": joint_ids,
        "joint_offsets": tuple(dynamixel_config.get("joint_offsets", (
            1 * np.pi, 1 * np.pi, 0 * np.pi, 1 * np.pi, 1 * np.pi, 1 * np.pi, 0.25 * np.pi
        ))),
        "joint_signs": tuple(dynamixel_config.get("joint_signs", (1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0))),
        "gripper_config": tuple(gripper_config) if gripper_config is not None else None,
    }


@dataclass
class GelloFrankaSingleArmConfig:
    arm_id: str = 'panda'
    # 新版 JointImpedanceController 在 /<arm_id>/joint_states 上发布反馈
    joint_state_topic: str = '/panda/joint_states'
    gripper_topic: str = '/panda_gripper/width_desired'
    target_joint_topic: str = '/panda/joints_desired'
    # 相对路径时按 <repo>/gello_teleop/config/ 解析
    gello_config_path: str = 'gello_arm1.yaml'
    publish_hz: float = 50.0
    startup_joint_threshold: float = 1.0
    enable_smoother: bool = True
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


class GelloFrankaSingleArmNode(Node):
    def __init__(self, config: GelloFrankaSingleArmConfig = None):
        super().__init__('gello_franka_singlearm')

        self.config = config if config is not None else GelloFrankaSingleArmConfig()
        self.gello_config_path = self.config.gello_config_path
        if not self.gello_config_path:
            raise RuntimeError('gello_config_path is required and cannot be empty')
        if not Path(self.gello_config_path).is_absolute():
            self.gello_config_path = str(SCRIPT_DIR.parent / 'config' / self.gello_config_path)
        self.arm_id = self.config.arm_id
        self.joint_state_topic = self.config.joint_state_topic
        self.target_joint_topic = self.config.target_joint_topic
        self.gripper_topic = self.config.gripper_topic
        self.publish_hz = float(self.config.publish_hz)
        self.startup_joint_threshold = float(self.config.startup_joint_threshold)
        self.joint_limits = np.asarray(self.config.joint_limits, dtype=float)
        self.enable_smoother = self.config.enable_smoother

        self.arm_joint_names = [f'{self.arm_id}_joint{i + 1}' for i in range(7)]
        self.q_current = None
        self.joint_states_received = False
        self._missing_joint_state_logged = False

        # 平滑器及初始化状态
        self.smoother = None
        self.smoother_initialized = False
        self.latest_raw_target = None  # 从 GELLO 获取的最新原始目标关节
        self.startup_gello_joint_target = None

        if self.enable_smoother:
            self.smoother = Smoother(
                publish_hz=self.publish_hz,
                max_velocity=0.15,
                position_tolerance=0.05,
                joint_limits=self.joint_limits,
                logger=self.get_logger()
            )

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.joint_publisher = self.create_publisher(JointState, self.target_joint_topic, 10)
        self.gripper_publisher = self.create_publisher(Float64, self.gripper_topic, 10)

        self.get_logger().info(f"Loading GELLO config from: {self.gello_config_path}")
        gello_config = load_gello_config(self.gello_config_path)

        dynamixel_config = DynamixelRobotConfig(
            joint_ids=gello_config["joint_ids"],
            joint_offsets=gello_config["joint_offsets"],
            joint_signs=gello_config["joint_signs"],
            gripper_config=gello_config["gripper_config"],
        )

        try:
            self.gello_agent = GelloAgent(
                port=gello_config["port"],
                dynamixel_config=dynamixel_config,
                real=True,
            )
        except Exception as exc:
            raise RuntimeError(f'Failed to initialize Gello Agent: {exc}')

        self.get_logger().info(f"GELLO Port: {gello_config['port']}")
        self.get_logger().info(f"Joint offsets: {gello_config['joint_offsets']}")
        self.get_logger().info(f"Joint signs: {gello_config['joint_signs']}")
        self.get_logger().info(f"Smoother enabled: {self.enable_smoother}")

        gello_joints = self.gello_agent.act({})
        if gello_joints is None or len(gello_joints) < 7:
            raise RuntimeError('Failed to read initial Gello joint positions')
        q_first = np.asarray(gello_joints[:7], dtype=float)
        self.startup_gello_joint_target = q_first
        self.get_logger().info(
            'First Gello joint read (after offset correction): ' +
            ', '.join(f'{float(val):.4f}' for val in q_first)
        )

        self._wait_for_startup_joint_alignment()

        self.create_timer(1.0 / max(self.publish_hz, 1.0), self.timer_callback)
        self.joint_print_timer = self.create_timer(1, self.joint_print_callback)
        if not self.enable_smoother:
            self.joint_states_received = True

        mode_info = 'with smoother' if self.enable_smoother else 'direct control'
        self.get_logger().info(
            f'Gello Franka single-arm teleop node initialized ({mode_info} mode).')
        self.get_logger().info(
            f'Publishing to {self.target_joint_topic}, feedback {self.joint_state_topic}, '
            f'gripper {self.gripper_topic}')

    def _wait_for_startup_joint_alignment(self):
        if self.startup_gello_joint_target is None:
            raise RuntimeError('Startup Gello joint target is not available')

        deadline = time.monotonic() + STARTUP_FEEDBACK_WAIT_SEC

        while self.q_current is None and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.q_current is None:
            raise RuntimeError(
                'Timed out waiting for initial joint_state feedback for startup alignment check '
                f'after {STARTUP_FEEDBACK_WAIT_SEC:.1f}s.'
            )

        joint_delta = np.abs(self.q_current - self.startup_gello_joint_target)
        max_delta = float(np.max(joint_delta))
        if max_delta > self.startup_joint_threshold:
            raise RuntimeError(
                'Startup alignment check failed: '
                f'max joint delta {max_delta:.4f} rad exceeds threshold '
                f'{self.startup_joint_threshold:.4f} rad. '
                f'controller={self.q_current.tolist()}, '
                f'gello={self.startup_gello_joint_target.tolist()}'
            )

        self.get_logger().info(
            'Startup alignment check passed: '
            f'max joint delta {max_delta:.4f} rad <= threshold '
            f'{self.startup_joint_threshold:.4f} rad.'
        )

    def joint_print_callback(self):
        try:
            gello_joints = self.gello_agent.act({})
            if gello_joints is None:
                return
            num_joints = len(gello_joints)
            joint_strs = []
            for i, val in enumerate(gello_joints):
                label = f"J{i+1}" if i < 7 else "GRIP"
                joint_strs.append(f"{label}:{float(val):.4f}")
            self.get_logger().info(
                f"GELLO joints ({num_joints}): " + ", ".join(joint_strs))
        except Exception:
            pass

    def joint_state_callback(self, msg: JointState):
        try:
            name_to_pos = dict(zip(msg.name, msg.position))
            if not all(name in name_to_pos for name in self.arm_joint_names):
                if not self._missing_joint_state_logged:
                    missing = [name for name in self.arm_joint_names
                               if name not in name_to_pos]
                    self.get_logger().warn(
                        'JointState names do not match expected Panda joints. '
                        f'Missing: {missing}. Received names: {list(msg.name)}'
                    )
                    self._missing_joint_state_logged = True
                return

            q_arm = np.array([name_to_pos[name] for name in self.arm_joint_names],
                             dtype=float)
            self.q_current = q_arm

            if self.enable_smoother and self.smoother is not None:
                # 将实际关节位置送给平滑器
                self.smoother.set_actual_position(q_arm)

                if not self.joint_states_received:
                    # 第一次接收到实际关节状态时初始化平滑器
                    self.joint_states_received = True
                    self.smoother.set_initial_position(q_arm)
                    # 如果已经有来自 GELLO 的目标，则用目标，否则以当前实际位置为初始目标
                    initial_target = (
                        self.latest_raw_target
                        if self.latest_raw_target is not None
                        else q_arm.copy()
                    )
                    self.smoother.set_target(initial_target)
                    self.smoother.set_follower_enabled(True)
                    self.smoother_initialized = True
                    self.get_logger().info(
                        'Smoother initialized from actual robot joint positions.')
            else:
                # 不使用平滑器时，只要收到关节状态就认为初始化完成
                if not self.joint_states_received:
                    self.joint_states_received = True
                    self.get_logger().info('Joint states received (direct control mode).')

        except Exception as exc:
            self.get_logger().warn(f'Failed to parse joint_states: {exc}')


    def _publish_joint_command(self, q_desired):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.arm_joint_names
        msg.position = q_desired.tolist()
        msg.velocity = [0.0] * len(self.arm_joint_names)
        self.joint_publisher.publish(msg)

    def timer_callback(self):
        try:
            gello_joints = self.gello_agent.act({})
            if gello_joints is None or len(gello_joints) < 7:
                self.get_logger().warn('Invalid Gello joint reading, skipping publish.')
                return

            q_target_raw = np.asarray(gello_joints[:7], dtype=float)
            q_target_raw = np.clip(
                q_target_raw, self.joint_limits[:, 0], self.joint_limits[:, 1])
            self.latest_raw_target = q_target_raw

            # 只有在初始化完成（已收到实际关节反馈）后才发布指令
            if self.joint_states_received:
                if (self.enable_smoother and self.smoother is not None
                        and self.smoother_initialized):
                    self.smoother.set_target(q_target_raw)
                    smoothed_joints = self.smoother.update()
                    self._publish_joint_command(smoothed_joints)
                else:
                    # 直接控制模式，不使用平滑器
                    self._publish_joint_command(q_target_raw)

            if not hasattr(self, '_debug_counter'):
                self._debug_counter = 0
            self._debug_counter += 1

            gripper_info = ""
            if len(gello_joints) >= 8:
                gripper_value = gello_joints[7]
                gripper_width = MAX_GRIPPER_OPEN * (1 - gripper_value)
                gripper_width = np.clip(gripper_width, 0.0, MAX_GRIPPER_OPEN)
                gripper_info = f" | Gripper: {gripper_value:.4f} ({gripper_width:.3f}m)"

                msg = Float64()
                msg.data = float(gripper_width)
                self.gripper_publisher.publish(msg)

            if self._debug_counter % 50 == 0:
                joints_str = ', '.join(f'{j:.4f}' for j in q_target_raw)
                self.get_logger().info(f'Joints: [{joints_str}]{gripper_info}')
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
    parser = argparse.ArgumentParser(
        description='GELLO single-arm teleoperation bridge (JointImpedanceController)')
    parser.add_argument('--config', type=str, default=None,
                        help='Path to GELLO YAML config (default: config/gello_arm1.yaml)')
    parser.add_argument('--arm-id', type=str, default=None,
                        help='Arm id used in joint names (default: panda)')
    parser.add_argument('--target-topic', type=str, default=None,
                        help='Joint command topic (default: /panda/joints_desired)')
    parser.add_argument('--joint-state-topic', type=str, default=None,
                        help='Joint feedback topic (default: /panda/joint_states)')
    parser.add_argument('--gripper-topic', type=str, default=None,
                        help='Gripper width topic (default: /panda_gripper/width_desired)')
    parser.add_argument('--publish-hz', type=float, default=None,
                        help='Command publish rate in Hz (default: 50.0)')
    parser.add_argument('--no-smoother', action='store_true',
                        help='Disable smoothing (direct joint command passthrough)')
    parser_args, _ = parser.parse_known_args(args)

    rclpy.init(args=args)
    node = None
    try:
        config_kwargs = {}
        if parser_args.config:
            config_kwargs['gello_config_path'] = parser_args.config
        if parser_args.arm_id:
            config_kwargs['arm_id'] = parser_args.arm_id
        if parser_args.target_topic:
            config_kwargs['target_joint_topic'] = parser_args.target_topic
        if parser_args.joint_state_topic:
            config_kwargs['joint_state_topic'] = parser_args.joint_state_topic
        if parser_args.gripper_topic:
            config_kwargs['gripper_topic'] = parser_args.gripper_topic
        if parser_args.publish_hz:
            config_kwargs['publish_hz'] = parser_args.publish_hz
        if parser_args.no_smoother:
            config_kwargs['enable_smoother'] = False

        config = GelloFrankaSingleArmConfig(**config_kwargs)
        node = GelloFrankaSingleArmNode(config=config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"Error: {e}")
        sys.exit(1)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

