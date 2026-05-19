#!/usr/bin/env python3
from dataclasses import dataclass, field
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import sys
import yaml
import warnings
from pathlib import Path

from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig
from gello_smoother import Smoother  # 新增导入平滑器


MAX_GRIPPER_OPEN = 0.08


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
    joint_state_topic: str = '/joint_states'
    target_joint_topic: str = '/joint_impedance/joints_desired'
    gripper_topic: str = '/panda_gripper/width_desired'
    gello_port: str = '/dev/ttyUSB1'
    publish_hz: float = 50.0
    startup_joint_threshold: float = 1.0
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


class GelloFrankaSingleArmNode(Node):
    def __init__(self, config: GelloFrankaSingleArmConfig = None, gello_config_path: str = None):
        super().__init__('gello_franka_singlearm')

        self.config = config if config is not None else GelloFrankaSingleArmConfig()
        self.arm_id = self.config.arm_id
        self.joint_state_topic = self.config.joint_state_topic
        self.target_joint_topic = self.config.target_joint_topic
        self.gripper_topic = self.config.gripper_topic
        self.publish_hz = float(self.config.publish_hz)
        self.joint_limits = np.asarray(self.config.joint_limits, dtype=float)
        self.default_joint_target = np.asarray(self.config.default_joint_target, dtype=float)

        self.arm_joint_names = [f'{self.arm_id}_joint{i + 1}' for i in range(7)]
        self.q_current = None
        self.joint_states_received = False
        self._missing_joint_state_logged = False

        # 新增：平滑器及初始化状态
        self.smoother = Smoother(
            publish_hz=self.publish_hz,
            max_velocity=0.3,
            position_tolerance=0.05,
            joint_limits=self.joint_limits,
            logger=self.get_logger()
        )
        self.smoother_initialized = False
        self.latest_raw_target = None  # 从 GELLO 获取的最新原始目标关节

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.joint_publisher = self.create_publisher(JointState, self.target_joint_topic, 10)
        self.gripper_publisher = self.create_publisher(Float64, self.gripper_topic, 10)

        # Load GELLO configuration from YAML or use defaults
        if gello_config_path:
            self.get_logger().info(f"Loading GELLO config from: {gello_config_path}")
            gello_config = load_gello_config(gello_config_path)
        else:
            self.get_logger().info("Using default GELLO configuration")
            gello_config = {
                "port": "/dev/ttyUSB1",
                "joint_ids": (1, 2, 3, 4, 5, 6, 7),
                "joint_offsets": (1 * np.pi, 1 * np.pi, 0 * np.pi, 1 * np.pi, 1 * np.pi, 1 * np.pi, 0.25 * np.pi),
                "joint_signs": (1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0),
                "gripper_config": (8, 195, 152),
            }

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

        # Log the loaded configuration
        self.get_logger().info(f"GELLO Port: {gello_config['port']}")
        self.get_logger().info(f"Joint offsets: {gello_config['joint_offsets']}")
        self.get_logger().info(f"Joint signs: {gello_config['joint_signs']}")

        gello_joints = self.gello_agent.act({})
        if gello_joints is None or len(gello_joints) < 7:
            raise RuntimeError('Failed to read initial Gello joint positions')
        q_first = np.asarray(gello_joints[:7], dtype=float)
        self.get_logger().info(
            'First Gello joint read (after offset correction): ' +
            ', '.join(f'{float(val):.4f}' for val in q_first)
        )

        self.create_timer(1.0 / max(self.publish_hz, 1.0), self.timer_callback)
        self.joint_print_timer = self.create_timer(1, self.joint_print_callback)
        self.get_logger().info('Gello Franka single-arm teleop node initialized (direct control mode with smoother).')

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
            self.get_logger().info(f"GELLO joints ({num_joints}): " + ", ".join(joint_strs))
        except Exception:
            pass

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

            # 将实际关节位置送给平滑器
            self.smoother.set_actual_position(q_arm)

            if not self.joint_states_received:
                # 第一次接收到实际关节状态时初始化平滑器
                self.joint_states_received = True
                # 设置平滑器初始位置为当前实际位置
                self.smoother.set_initial_position(q_arm)
                # 如果已经有来自 GELLO 的目标，则用目标，否则以当前实际位置为初始目标
                initial_target = self.latest_raw_target if self.latest_raw_target is not None else q_arm.copy()
                self.smoother.set_target(initial_target)
                self.smoother.set_follower_enabled(True)
                self.smoother_initialized = True
                self.get_logger().info(
                    'Smoother initialized from actual robot joint positions.'
                )

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
            q_target_raw = np.clip(q_target_raw, self.joint_limits[:, 0], self.joint_limits[:, 1])
            self.latest_raw_target = q_target_raw

            # 只有在平滑器初始化完成（已收到实际关节反馈）后才发布平滑后的指令
            if self.smoother_initialized and self.q_current is not None:
                self.smoother.set_target(q_target_raw)
                smoothed_joints = self.smoother.update()
                self._publish_joint_command(smoothed_joints)
            else:
                # 平滑器未初始化时不发布关节指令（等待实际关节反馈）
                pass

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
                # 打印的仍然是原始目标（或平滑后？这里保持打印原始目标以观察 GELLO 输出）
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
    rclpy.init(args=args)
    node = None
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='GELLO Franka Single Arm ROS2 Bridge')
    parser.add_argument('--config', type=str, default=None, help='Path to GELLO YAML config file')
    parser.add_argument('--gripper-topic', type=str, default=None, help='ROS2 topic for gripper control')
    parsed_args, _ = parser.parse_known_args(args)

    # Set default config path if not provided
    config_path = parsed_args.config
    if config_path is None:
        # Only read from package config directory
        script_dir = Path(__file__).resolve().parent
        package_config = script_dir.parent / 'config' / 'yam_auto_generated.yaml'
        
        if package_config.exists():
            config_path = str(package_config)
            print(f"Using package config: {config_path}")
        else:
            print(f"Config not found: {package_config}")
            print("Using hardcoded default configuration")

    try:
        # Create config with optional gripper topic override
        config_kwargs = {}
        if parsed_args.gripper_topic:
            config_kwargs['gripper_topic'] = parsed_args.gripper_topic
        
        config = GelloFrankaSingleArmConfig(**config_kwargs)
        node = GelloFrankaSingleArmNode(config=config, gello_config_path=config_path)
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