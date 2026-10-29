#!/usr/bin/env python3
"""GELLO 双臂遥操作桥接节点（对接新版双臂关节阻抗控制器）。

数据流：
    GELLO -> 平滑器 -> /<arm_id>/joints_desired (sensor_msgs/JointState)
           -> dual_joint_impedance_controller (franka_example_controllers/MultiJointImpedanceController)
    夹爪   -> /<arm_id>_gripper/width_desired (std_msgs/Float64)

与旧版脚本（/dual_joint_impedance/joints_desired, Float64MultiArray 14 维）的区别：
    - 新版控制器按臂分别订阅 ``/<arm_id>/joints_desired``，消息类型为
      ``sensor_msgs/msg/JointState``，且必须携带关节名
      （``<arm_id>_joint1`` ... ``<arm_id>_joint7``），不能只靠位置数组；
    - 仿真默认臂名为 ``mj_left`` / ``mj_right``（dual_sim_controllers.yaml），
      真机默认臂名为 ``panda_left`` / ``panda_right``（real/dual_controllers.yaml）；
    - 夹爪仍由点击 bridge 的 ``width_desired`` 话题驱动（单位：米）。

用法：
    cd src/multipanda_ros2/gello_teleop/scripts
    python3 gello_franka_ros2.py --control-mode 3              # 仿真（默认）
    python3 gello_franka_ros2.py --hardware-mode 1             # 真机
    python3 gello_franka_ros2.py --hardware-mode 1 \
        --left-arm-id panda_left --right-arm-id panda_right    # 显式指定臂名
"""

from dataclasses import dataclass, field
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import yaml

# 允许直接以脚本方式运行（否则找不到同目录的 gello_smoother）
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig  # noqa: E402
from gello_smoother import Smoother  # noqa: E402

MAX_GRIPPER_OPEN = 0.08

LEFT_ARM = 1
RIGHT_ARM = 2
BOTH_ARMS = 3

SIM_MODE = 0
REAL_MODE = 1

# 仿真/真机默认臂名（与 franka_bringup 的控制器配置保持一致）
SIM_ARM_IDS = ('mj_left', 'mj_right')
REAL_ARM_IDS = ('panda_left', 'panda_right')


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
        "port": agent_config.get("port", "/dev/ttyUSB1"),
        "joint_ids": joint_ids,
        "joint_offsets": tuple(dynamixel_config.get("joint_offsets", (
            1 * np.pi, 1 * np.pi, 0 * np.pi, 1 * np.pi, 1 * np.pi, 1 * np.pi, 0.25 * np.pi
        ))),
        "joint_signs": tuple(dynamixel_config.get("joint_signs", (1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0))),
        "gripper_config": tuple(gripper_config) if gripper_config is not None else None,
    }


@dataclass
class GelloFrankaDualArmConfig:
    # 臂名：仿真 mj_left/mj_right，真机 panda_left/panda_right
    left_arm_id: str = SIM_ARM_IDS[0]
    right_arm_id: str = SIM_ARM_IDS[1]
    # 仅用于初始化平滑器的实际关节反馈（兼容聚合与分臂两种话题）
    joint_state_topic: str = '/joint_states'
    # 留空表示自动推导为 /<arm_id>/joints_desired
    left_target_joint_topic: str = ''
    right_target_joint_topic: str = ''
    # 留空表示自动推导为 /<arm_id>_gripper/width_desired
    left_gripper_topic: str = ''
    right_gripper_topic: str = ''
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


class GelloFrankaDualArmNode(Node):
    """GELLO 双臂桥接：直接向双臂关节阻抗控制器发布绝对关节位置。"""

    def __init__(self, config: GelloFrankaDualArmConfig = None):
        super().__init__('gello_franka_bridge')

        self.config = config if config is not None else GelloFrankaDualArmConfig()
        self.left_arm_id = self.config.left_arm_id
        self.right_arm_id = self.config.right_arm_id
        self.control_mode = self.config.control_mode
        self.hardware_mode = self.config.hardware_mode
        self.publish_hz = float(self.config.publish_hz)
        self.joint_limits = np.asarray(self.config.joint_limits, dtype=float)
        self.default_joint_target = np.asarray(self.config.default_joint_target, dtype=float)

        # 关节名/话题：新版控制器按 <arm_id>_joint1..7 匹配 JointState
        self.left_joint_names = [f'{self.left_arm_id}_joint{i + 1}' for i in range(7)]
        self.right_joint_names = [f'{self.right_arm_id}_joint{i + 1}' for i in range(7)]
        self.left_target_joint_topic = (
            self.config.left_target_joint_topic or f'/{self.left_arm_id}/joints_desired')
        self.right_target_joint_topic = (
            self.config.right_target_joint_topic or f'/{self.right_arm_id}/joints_desired')
        self.left_gripper_topic = (
            self.config.left_gripper_topic or f'/{self.left_arm_id}_gripper/width_desired')
        self.right_gripper_topic = (
            self.config.right_gripper_topic or f'/{self.right_arm_id}_gripper/width_desired')

        self.q_left_current = None
        self.q_right_current = None
        self.joint_states_received = False
        # 缺帧告警只打印一次（每个聚合话题都会包含另一侧臂缺失的关节名，避免刷屏）
        self._missing_left_joint_state_logged = False
        self._missing_right_joint_state_logged = False

        # 两个独立的平滑器
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
        # 最近一次平滑输出：GELLO 单次读取失败时保持上一拍命令
        self.latest_left_smoothed = None
        self.latest_right_smoothed = None

        # 关节状态反馈：聚合话题 + 各臂控制器自己的 joint_states 话题
        feedback_topics = [self.config.joint_state_topic]
        for arm_id in (self.left_arm_id, self.right_arm_id):
            candidate = f'/{arm_id}/joint_states'
            if candidate not in feedback_topics:
                feedback_topics.append(candidate)
        self.joint_state_subs = [
            self.create_subscription(JointState, topic, self.joint_state_callback, 10)
            for topic in feedback_topics
        ]

        # 目标关节发布器（每臂一个 JointState 话题）
        self.left_joint_publisher = self.create_publisher(
            JointState, self.left_target_joint_topic, 10)
        self.right_joint_publisher = self.create_publisher(
            JointState, self.right_target_joint_topic, 10)

        # 夹爪发布器
        self.left_gripper_publisher = self.create_publisher(
            Float64, self.left_gripper_topic, 10)
        self.right_gripper_publisher = self.create_publisher(
            Float64, self.right_gripper_topic, 10)

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

        # 读取初始位置（仅用于日志，平滑器初始化以实际机器人关节为准）
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
            'Gello Franka dual-arm bridge initialized '
            f'(control_mode={self.control_mode}, publish_hz={self.publish_hz}, '
            f'left={self.left_target_joint_topic}, right={self.right_target_joint_topic})'
        )
        self.get_logger().info(
            f'Gripper topics: {self.left_gripper_topic} | {self.right_gripper_topic}'
        )

    def joint_print_callback(self):
        try:
            left_gello_joints = self.left_gello_agent.act({})
            right_gello_joints = self.right_gello_agent.act({})

            if left_gello_joints is not None:
                left_joint_strs = []
                for i, val in enumerate(left_gello_joints):
                    label = f"J{i + 1}" if i < 7 else "GRIP"
                    left_joint_strs.append(f"{label}:{float(val):.4f}")
                self.get_logger().info("Left arm GELLO joints: " + ", ".join(left_joint_strs))

            if right_gello_joints is not None:
                right_joint_strs = []
                for i, val in enumerate(right_gello_joints):
                    label = f"J{i + 1}" if i < 7 else "GRIP"
                    right_joint_strs.append(f"{label}:{float(val):.4f}")
                self.get_logger().info("Right arm GELLO joints: " + ", ".join(right_joint_strs))
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # 关节状态回调：初始化平滑器（起点锚定到实际机器人关节位置）
    # ------------------------------------------------------------------ #
    def joint_state_callback(self, msg: JointState):
        try:
            name_to_pos = dict(zip(msg.name, msg.position))

            left_positions = [name_to_pos.get(name) for name in self.left_joint_names]
            right_positions = [name_to_pos.get(name) for name in self.right_joint_names]

            # 更新左臂状态
            if all(pos is not None for pos in left_positions):
                self.q_left_current = np.array(left_positions, dtype=float)
                self._missing_left_joint_state_logged = False

                if not self.left_smoother_initialized:
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
            elif not self._missing_left_joint_state_logged:
                missing = [
                    self.left_joint_names[i]
                    for i, pos in enumerate(left_positions) if pos is None
                ]
                self.get_logger().warn(f'Left arm joint state missing: {missing}')
                self._missing_left_joint_state_logged = True

            # 更新右臂状态
            if all(pos is not None for pos in right_positions):
                self.q_right_current = np.array(right_positions, dtype=float)
                self._missing_right_joint_state_logged = False

                if not self.right_smoother_initialized:
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
            elif not self._missing_right_joint_state_logged:
                missing = [
                    self.right_joint_names[i]
                    for i, pos in enumerate(right_positions) if pos is None
                ]
                self.get_logger().warn(f'Right arm joint state missing: {missing}')
                self._missing_right_joint_state_logged = True

            # 设置初始化完成标志（按控制模式决定需要哪只臂的反馈）
            if not self.joint_states_received:
                if ((self.control_mode == LEFT_ARM and self.q_left_current is not None) or
                        (self.control_mode == RIGHT_ARM and self.q_right_current is not None) or
                        (self.control_mode == BOTH_ARMS and
                         self.q_left_current is not None and self.q_right_current is not None)):
                    self.joint_states_received = True

        except Exception as exc:
            self.get_logger().warn(f'Failed to parse joint_states: {exc}')

    def _publish_arm_command(self, publisher, joint_names, positions):
        """以 sensor_msgs/JointState 形式发布单臂目标，供新版控制器解析。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joint_names)
        msg.position = [float(val) for val in positions]
        msg.velocity = [0.0] * len(joint_names)
        publisher.publish(msg)


    # ------------------------------------------------------------------ #
    # 主循环：读 GELLO -> 限幅 -> 平滑 -> 分臂发布目标关节
    # ------------------------------------------------------------------ #
    def timer_callback(self):
        try:
            left_gello_joints = self.left_gello_agent.act({})
            right_gello_joints = self.right_gello_agent.act({})

            left_smoothed = None
            right_smoothed = None
            left_gripper_val = None
            right_gripper_val = None

            # 处理左臂数据
            if left_gello_joints is not None and len(left_gello_joints) >= 7:
                left_q_target_raw = np.asarray(left_gello_joints[:7], dtype=float)
                left_q_target_raw = np.clip(
                    left_q_target_raw, self.joint_limits[:, 0], self.joint_limits[:, 1])
                self.latest_left_raw_target = left_q_target_raw

                if self.left_smoother_initialized and self.q_left_current is not None:
                    self.left_smoother.set_actual_position(self.q_left_current)
                    self.left_smoother.set_target(left_q_target_raw)
                    left_smoothed = self.left_smoother.update()

                if len(left_gello_joints) >= 8:
                    left_gripper_val = left_gello_joints[7]

            # 处理右臂数据
            if right_gello_joints is not None and len(right_gello_joints) >= 7:
                right_q_target_raw = np.asarray(right_gello_joints[:7], dtype=float)
                right_q_target_raw = np.clip(
                    right_q_target_raw, self.joint_limits[:, 0], self.joint_limits[:, 1])
                self.latest_right_raw_target = right_q_target_raw

                if self.right_smoother_initialized and self.q_right_current is not None:
                    self.right_smoother.set_actual_position(self.q_right_current)
                    self.right_smoother.set_target(right_q_target_raw)
                    right_smoothed = self.right_smoother.update()

                if len(right_gello_joints) >= 8:
                    right_gripper_val = right_gello_joints[7]

            # 计算各臂最终命令：平滑输出 > 上一拍输出 > 当前实际位置 > 默认位姿
            # （GELLO 单次读取失败时保持上一拍命令，避免控制器超时或目标跳变）
            if left_smoothed is not None:
                self.latest_left_smoothed = left_smoothed
            elif self.latest_left_smoothed is not None:
                left_smoothed = self.latest_left_smoothed
            elif self.q_left_current is not None:
                left_smoothed = self.q_left_current.copy()
            else:
                left_smoothed = self.default_joint_target

            if right_smoothed is not None:
                self.latest_right_smoothed = right_smoothed
            elif self.latest_right_smoothed is not None:
                right_smoothed = self.latest_right_smoothed
            elif self.q_right_current is not None:
                right_smoothed = self.q_right_current.copy()
            else:
                right_smoothed = self.default_joint_target

            # 发布关节指令（新版控制器：每臂一个 JointState 话题）
            # 注意：只发布当前控制模式选中的臂，未选中的臂完全不发指令，
            # 否则会被其控制器当作目标位置执行。
            if self.joint_states_received:
                if self.control_mode in (LEFT_ARM, BOTH_ARMS):
                    self._publish_arm_command(
                        self.left_joint_publisher, self.left_joint_names, left_smoothed)
                if self.control_mode in (RIGHT_ARM, BOTH_ARMS):
                    self._publish_arm_command(
                        self.right_joint_publisher, self.right_joint_names, right_smoothed)


            # 处理夹爪指令（width_desired，单位：米）
            if left_gripper_val is not None and (
                    self.control_mode == LEFT_ARM or self.control_mode == BOTH_ARMS):
                left_gripper_width = MAX_GRIPPER_OPEN * (1 - left_gripper_val)
                left_gripper_width = np.clip(left_gripper_width, 0.0, MAX_GRIPPER_OPEN)
                msg_gripper = Float64()
                msg_gripper.data = float(left_gripper_width)
                self.left_gripper_publisher.publish(msg_gripper)

            if right_gripper_val is not None and (
                    self.control_mode == RIGHT_ARM or self.control_mode == BOTH_ARMS):
                right_gripper_width = MAX_GRIPPER_OPEN * (1 - right_gripper_val)
                right_gripper_width = np.clip(right_gripper_width, 0.0, MAX_GRIPPER_OPEN)
                msg_gripper = Float64()
                msg_gripper.data = float(right_gripper_width)
                self.right_gripper_publisher.publish(msg_gripper)

            # 调试输出
            if not hasattr(self, '_debug_counter'):
                self._debug_counter = 0
            self._debug_counter += 1

            if self._debug_counter % 50 == 0:
                left_str = 'N/A'
                if self.latest_left_raw_target is not None:
                    left_str = ', '.join(f'{j:.4f}' for j in self.latest_left_raw_target)
                    if left_gripper_val is not None:
                        left_str += f' | GRIP:{left_gripper_val:.4f}'

                right_str = 'N/A'
                if self.latest_right_raw_target is not None:
                    right_str = ', '.join(f'{j:.4f}' for j in self.latest_right_raw_target)
                    if right_gripper_val is not None:
                        right_str += f' | GRIP:{right_gripper_val:.4f}'

                self.get_logger().info(f'Left arm: [{left_str}] | Right arm: [{right_str}]')
        except Exception as exc:
            self.get_logger().error(f'Error publishing joint command: {exc}')

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

    parser = argparse.ArgumentParser(description='GELLO Franka Dual Arm ROS2 Bridge')
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
    parser.add_argument('--left-arm-id', type=str, default=None,
                        help='Left arm id used in joint names/topics '
                             '(default: mj_left for sim, panda_left for real)')
    parser.add_argument('--right-arm-id', type=str, default=None,
                        help='Right arm id used in joint names/topics '
                             '(default: mj_right for sim, panda_right for real)')
    parsed_args, _ = parser.parse_known_args(args)

    rclpy.init(args=args)
    node = None

    try:
        hardware_mode = (
            parsed_args.hardware_mode
            if parsed_args.hardware_mode in [SIM_MODE, REAL_MODE]
            else SIM_MODE
        )
        default_arm_ids = REAL_ARM_IDS if hardware_mode == REAL_MODE else SIM_ARM_IDS

        config_kwargs = {
            'hardware_mode': hardware_mode,
            'publish_hz': parsed_args.publish_hz,
            'left_arm_id': parsed_args.left_arm_id or default_arm_ids[0],
            'right_arm_id': parsed_args.right_arm_id or default_arm_ids[1],
        }
        if parsed_args.control_mode in [LEFT_ARM, RIGHT_ARM, BOTH_ARMS]:
            config_kwargs['control_mode'] = parsed_args.control_mode
        if parsed_args.left_config:
            config_kwargs['left_gello_config_path'] = parsed_args.left_config
        if parsed_args.right_config:
            config_kwargs['right_gello_config_path'] = parsed_args.right_config

        config = GelloFrankaDualArmConfig(**config_kwargs)
        node = GelloFrankaDualArmNode(config=config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)
    except RuntimeError as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

