#!/usr/bin/env python3
from dataclasses import dataclass, field
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import yaml
from pathlib import Path

from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig


def load_gello_config(config_path: str):
    """Load GELLO configuration from YAML file."""
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    
    agent_config = config.get("agent", {})
    dynamixel_config = agent_config.get("dynamixel_config", {})
    
    return {
        "port": agent_config.get("port", "/dev/ttyUSB0"),
        "joint_ids": tuple(dynamixel_config.get("joint_ids", (1, 2, 3, 4, 5, 6, 7))),
        "joint_offsets": tuple(dynamixel_config.get("joint_offsets", (
            1 * np.pi, 1 * np.pi, 0 * np.pi, 1 * np.pi, 1 * np.pi, 1 * np.pi, 0.25 * np.pi
        ))),
        "joint_signs": tuple(dynamixel_config.get("joint_signs", (1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0))),
        "gripper_config": dynamixel_config.get("gripper_config", None),
    }


@dataclass
class GelloFrankaSingleArmConfig:
    arm_id: str = 'panda'
    joint_state_topic: str = '/panda/joint_states'
    target_joint_topic: str = '/joint_impedance/joints_desired'
    gello_port: str = '/dev/ttyUSB0'
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
        self.publish_hz = float(self.config.publish_hz)
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

        # Load GELLO configuration from YAML or use defaults
        if gello_config_path:
            self.get_logger().info(f"Loading GELLO config from: {gello_config_path}")
            gello_config = load_gello_config(gello_config_path)
        else:
            self.get_logger().info("Using default GELLO configuration")
            gello_config = {
                "port": "/dev/ttyUSB0",
                "joint_ids": (1, 2, 3, 4, 5, 6, 7),
                "joint_offsets": (1 * np.pi, 1 * np.pi, 0 * np.pi, 1 * np.pi, 1 * np.pi, 1 * np.pi, 0.25 * np.pi),
                "joint_signs": (1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0),
                "gripper_config": None,
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
        self.get_logger().info('Gello Franka single-arm teleop node initialized (direct control mode).')

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

            q_target = np.asarray(gello_joints[:7], dtype=float)
            q_target = np.clip(q_target, self.joint_limits[:, 0], self.joint_limits[:, 1])

            self._publish_joint_command(q_target)
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
        node = GelloFrankaSingleArmNode(gello_config_path=config_path)
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