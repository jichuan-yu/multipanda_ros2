#!/usr/bin/env python3
from dataclasses import dataclass, field
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
from rclpy.action import ActionClient
from franka_msgs.action import Move as MoveAction
import sys
import yaml
import warnings
from pathlib import Path

from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig
from gello_smoother import Smoother

MAX_GRIPPER_OPEN = 0.08

LEFT_ARM = 1
RIGHT_ARM = 2
BOTH_ARMS = 3


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


SIM_MODE = 0
REAL_MODE = 1

@dataclass
class GelloFrankaDualArmConfig:
    arm_id: str = 'mj'
    joint_state_topic: str = '/joint_states'
    target_joint_topic: str = '/dual_joint_impedance/joints_desired'
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


class GelloFrankaDualArmNode(Node):
    def __init__(self, config: GelloFrankaDualArmConfig = None):
        super().__init__('gello_franka_bridge')

        self.config = config if config is not None else GelloFrankaDualArmConfig()
        self.arm_id = self.config.arm_id
        self.joint_state_topic = self.config.joint_state_topic
        self.target_joint_topic = self.config.target_joint_topic
        self.left_gripper_topic = self.config.left_gripper_topic
        self.right_gripper_topic = self.config.right_gripper_topic
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

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.joint_publisher = self.create_publisher(Float64MultiArray, self.target_joint_topic, 10)
        
        # 根据硬件模式选择夹爪控制方式（都通过桥接节点）
        if self.hardware_mode == SIM_MODE:
            # 仿真模式：通过桥接节点控制夹爪
            self.get_logger().info("Using simulation mode: Topic publishing via bridge")
            self.left_gripper_publisher = self.create_publisher(Float64, '/mj_left_gripper/width_desired', 10)
            self.right_gripper_publisher = self.create_publisher(Float64, '/mj_right_gripper/width_desired', 10)
            self.left_gripper_action_client = None
            self.right_gripper_action_client = None
        else:
            # 真机模式：通过桥接节点控制夹爪
            self.get_logger().info("Using real hardware mode: Topic publishing via bridge")
            self.left_gripper_publisher = self.create_publisher(Float64, '/left_gripper/width_desired', 10)
            self.right_gripper_publisher = self.create_publisher(Float64, '/right_gripper/width_desired', 10)
            self.left_gripper_action_client = None
            self.right_gripper_action_client = None

        # 加载左臂配置
        script_dir = Path(__file__).resolve().parent
        
        # 左臂配置
        if self.config.left_gello_config_path:
            left_config_path = self.config.left_gello_config_path
        else:
            left_config_path = script_dir.parent / 'config' / 'gello_arm1.yaml'
        
        # 右臂配置
        if self.config.right_gello_config_path:
            right_config_path = self.config.right_gello_config_path
        else:
            right_config_path = script_dir.parent / 'config' / 'gello_arm2.yaml'
        
        # 加载左臂配置
        self.get_logger().info(f"Loading left arm GELLO config from: {left_config_path}")
        left_gello_config = load_gello_config(str(left_config_path))
        
        # 加载右臂配置
        self.get_logger().info(f"Loading right arm GELLO config from: {right_config_path}")
        right_gello_config = load_gello_config(str(right_config_path))

        # 创建左臂 Dynamixel 配置
        left_dynamixel_config = DynamixelRobotConfig(
            joint_ids=left_gello_config["joint_ids"],
            joint_offsets=left_gello_config["joint_offsets"],
            joint_signs=left_gello_config["joint_signs"],
            gripper_config=left_gello_config["gripper_config"],
        )
        
        # 创建右臂 Dynamixel 配置
        right_dynamixel_config = DynamixelRobotConfig(
            joint_ids=right_gello_config["joint_ids"],
            joint_offsets=right_gello_config["joint_offsets"],
            joint_signs=right_gello_config["joint_signs"],
            gripper_config=right_gello_config["gripper_config"],
        )

        # 初始化左臂 GELLO Agent
        try:
            self.left_gello_agent = GelloAgent(
                port=left_gello_config["port"],
                dynamixel_config=left_dynamixel_config,
                real=True,
            )
            self.get_logger().info(f"Left arm GELLO Port: {left_gello_config['port']}")
        except Exception as exc:
            raise RuntimeError(f'Failed to initialize left arm Gello Agent: {exc}')
        
        # 初始化右臂 GELLO Agent
        try:
            self.right_gello_agent = GelloAgent(
                port=right_gello_config["port"],
                dynamixel_config=right_dynamixel_config,
                real=True,
            )
            self.get_logger().info(f"Right arm GELLO Port: {right_gello_config['port']}")
        except Exception as exc:
            raise RuntimeError(f'Failed to initialize right arm Gello Agent: {exc}')

        self.get_logger().info(f"Control mode: {self.control_mode} (1=LEFT, 2=RIGHT, 3=BOTH)")

        # 读取左臂初始数据
        left_gello_joints = self.left_gello_agent.act({})
        if left_gello_joints is None or len(left_gello_joints) < 7:
            raise RuntimeError('Failed to read initial left arm Gello joint positions')
        left_q_first = np.asarray(left_gello_joints[:7], dtype=float)
        self.get_logger().info(
            'Left arm first Gello joint read (after offset correction): ' +
            ', '.join(f'{float(val):.4f}' for val in left_q_first)
        )
        
        # 读取右臂初始数据
        right_gello_joints = self.right_gello_agent.act({})
        if right_gello_joints is None or len(right_gello_joints) < 7:
            raise RuntimeError('Failed to read initial right arm Gello joint positions')
        right_q_first = np.asarray(right_gello_joints[:7], dtype=float)
        self.get_logger().info(
            'Right arm first Gello joint read (after offset correction): ' +
            ', '.join(f'{float(val):.4f}' for val in right_q_first)
        )

        self.create_timer(1.0 / max(self.publish_hz, 1.0), self.timer_callback)
        self.joint_print_timer = self.create_timer(1, self.joint_print_callback)
        self.get_logger().info('Gello Franka dual-arm teleop node initialized (direct control mode with smoother).')

    def joint_print_callback(self):
        try:
            # 读取左臂数据
            left_gello_joints = self.left_gello_agent.act({})
            # 读取右臂数据
            right_gello_joints = self.right_gello_agent.act({})
            
            if left_gello_joints is not None:
                left_joint_strs = []
                for i, val in enumerate(left_gello_joints):
                    label = f"J{i+1}" if i < 7 else "GRIP"
                    left_joint_strs.append(f"{label}:{float(val):.4f}")
                self.get_logger().info(f"Left arm GELLO joints: " + ", ".join(left_joint_strs))
            
            if right_gello_joints is not None:
                right_joint_strs = []
                for i, val in enumerate(right_gello_joints):
                    label = f"J{i+1}" if i < 7 else "GRIP"
                    right_joint_strs.append(f"{label}:{float(val):.4f}")
                self.get_logger().info(f"Right arm GELLO joints: " + ", ".join(right_joint_strs))
        except Exception:
            pass

    def joint_state_callback(self, msg: JointState):
        try:
            left_indices = [msg.name.index(name) if name in msg.name else None for name in self.left_joint_names]
            right_indices = [msg.name.index(name) if name in msg.name else None for name in self.right_joint_names]

            # 更新左臂状态
            if all(idx is not None for idx in left_indices):
                self.q_left_current = np.array([msg.position[idx] for idx in left_indices], dtype=float)
                
                if self.q_left_current is not None and not self.left_smoother_initialized:
                    self.left_smoother.set_actual_position(self.q_left_current)
                    self.left_smoother.set_initial_position(self.q_left_current)
                    initial_target = self.latest_left_raw_target if self.latest_left_raw_target is not None else self.q_left_current.copy()
                    self.left_smoother.set_target(initial_target)
                    self.left_smoother.set_follower_enabled(True)
                    self.left_smoother_initialized = True
                    self.get_logger().info('Left arm smoother initialized from actual robot joint positions.')
            elif not self._missing_joint_state_logged:
                missing = [self.left_joint_names[i] for i, idx in enumerate(left_indices) if idx is None]
                self.get_logger().warn(f'Left arm joint state missing: {missing}')

            # 更新右臂状态
            if all(idx is not None for idx in right_indices):
                self.q_right_current = np.array([msg.position[idx] for idx in right_indices], dtype=float)
                
                if self.q_right_current is not None and not self.right_smoother_initialized:
                    self.right_smoother.set_actual_position(self.q_right_current)
                    self.right_smoother.set_initial_position(self.q_right_current)
                    initial_target = self.latest_right_raw_target if self.latest_right_raw_target is not None else self.q_right_current.copy()
                    self.right_smoother.set_target(initial_target)
                    self.right_smoother.set_follower_enabled(True)
                    self.right_smoother_initialized = True
                    self.get_logger().info('Right arm smoother initialized from actual robot joint positions.')
            elif not self._missing_joint_state_logged:
                missing = [self.right_joint_names[i] for i, idx in enumerate(right_indices) if idx is None]
                self.get_logger().warn(f'Right arm joint state missing: {missing}')

            # 设置初始化完成标志
            if not self.joint_states_received:
                if ((self.control_mode == LEFT_ARM and self.q_left_current is not None) or
                    (self.control_mode == RIGHT_ARM and self.q_right_current is not None) or
                    (self.control_mode == BOTH_ARMS and self.q_left_current is not None and self.q_right_current is not None)):
                    self.joint_states_received = True

        except Exception as exc:
            self.get_logger().warn(f'Failed to parse joint_states: {exc}')

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
                left_q_target_raw = np.asarray(left_gello_joints[:7], dtype=float)
                left_q_target_raw = np.clip(left_q_target_raw, self.joint_limits[:, 0], self.joint_limits[:, 1])
                self.latest_left_raw_target = left_q_target_raw
                
                if self.left_smoother_initialized and self.q_left_current is not None:
                    self.left_smoother.set_actual_position(self.q_left_current)
                    self.left_smoother.set_target(left_q_target_raw)
                    left_smoothed = self.left_smoother.update()
                
                # 处理夹爪数据
                if len(left_gello_joints) >= 8:
                    left_gripper_val = left_gello_joints[7]
            
            # 处理右臂数据
            if right_gello_joints is not None and len(right_gello_joints) >= 7:
                right_q_target_raw = np.asarray(right_gello_joints[:7], dtype=float)
                right_q_target_raw = np.clip(right_q_target_raw, self.joint_limits[:, 0], self.joint_limits[:, 1])
                self.latest_right_raw_target = right_q_target_raw
                
                if self.right_smoother_initialized and self.q_right_current is not None:
                    self.right_smoother.set_actual_position(self.q_right_current)
                    self.right_smoother.set_target(right_q_target_raw)
                    right_smoothed = self.right_smoother.update()
                
                # 处理夹爪数据
                if len(right_gello_joints) >= 8:
                    right_gripper_val = right_gello_joints[7]
            
            # 构建双臂关节数据
            dual_joints = []
            if self.control_mode == LEFT_ARM:
                left_joints = left_smoothed if left_smoothed is not None else self.default_joint_target
                dual_joints = list(left_joints) + list(self.default_joint_target)
            elif self.control_mode == RIGHT_ARM:
                right_joints = right_smoothed if right_smoothed is not None else self.default_joint_target
                dual_joints = list(self.default_joint_target) + list(right_joints)
            elif self.control_mode == BOTH_ARMS:
                left_joints = left_smoothed if left_smoothed is not None else self.default_joint_target
                right_joints = right_smoothed if right_smoothed is not None else self.default_joint_target
                dual_joints = list(left_joints) + list(right_joints)
            
            # 发布关节指令
            if dual_joints and self.joint_states_received:
                msg = Float64MultiArray()
                msg.data = dual_joints
                self.joint_publisher.publish(msg)
            
            # 处理夹爪指令（通过桥接节点）
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
            
            # 调试输出
            if not hasattr(self, '_debug_counter'):
                self._debug_counter = 0
            self._debug_counter += 1
            
            if self._debug_counter % 50 == 0:
                left_str = 'N/A'
                if left_gello_joints is not None and len(left_gello_joints) >= 7:
                    left_str = ', '.join(f'{j:.4f}' for j in left_gello_joints[:7])
                    if len(left_gello_joints) >= 8:
                        left_str += f' | GRIP:{left_gello_joints[7]:.4f}'
                
                right_str = 'N/A'
                if right_gello_joints is not None and len(right_gello_joints) >= 7:
                    right_str = ', '.join(f'{j:.4f}' for j in right_gello_joints[:7])
                    if len(right_gello_joints) >= 8:
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
    rclpy.init(args=args)
    node = None
    
    import argparse
    parser = argparse.ArgumentParser(description='GELLO Franka Dual Arm ROS2 Bridge')
    parser.add_argument('--left-config', type=str, default=None, help='Path to left arm GELLO YAML config file')
    parser.add_argument('--right-config', type=str, default=None, help='Path to right arm GELLO YAML config file')
    parser.add_argument('--control-mode', type=int, default=BOTH_ARMS, 
                        help='Control mode: 1=LEFT_ARM, 2=RIGHT_ARM, 3=BOTH_ARMS')
    parser.add_argument('--hardware-mode', type=int, default=SIM_MODE, 
                        help='Hardware mode: 0=SIM_MODE (simulation), 1=REAL_MODE (real hardware)')
    parsed_args, _ = parser.parse_known_args(args)

    try:
        config_kwargs = {}
        if parsed_args.control_mode in [LEFT_ARM, RIGHT_ARM, BOTH_ARMS]:
            config_kwargs['control_mode'] = parsed_args.control_mode
        if parsed_args.hardware_mode in [SIM_MODE, REAL_MODE]:
            config_kwargs['hardware_mode'] = parsed_args.hardware_mode
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
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()