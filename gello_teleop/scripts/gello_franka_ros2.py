#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import sys
import yaml
from pathlib import Path

sys.path.insert(0, '/home/botao/dual_panda_ws/src/multipanda_ros2/gello_teleop/scripts')
from gello_smoother import Smoother

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

LEFT_ARM = 1
RIGHT_ARM = 2
BOTH_ARMS = 3

CONTROL_MODE = RIGHT_ARM

DEFAULT_JOINTS = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])


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
            1 * np.pi, 1 * np.pi, 0 * np.pi, 1 * np.pi, 1 * np.pi, 1 * np.pi, 1.25 * np.pi
        ))),
        "joint_signs": tuple(dynamixel_config.get("joint_signs", (1, -1, 1, 1, 1, -1, 1))),
        "gripper_config": dynamixel_config.get("gripper_config", None),
    }


class GelloFrankaBridge(Node):
    def __init__(self, config_path: str = None):
        super().__init__('gello_franka_bridge')
        
        # Load configuration from YAML or use defaults
        if config_path:
            self.get_logger().info(f"Loading GELLO config from: {config_path}")
            gello_config = load_gello_config(config_path)
        else:
            self.get_logger().info("Using default GELLO configuration")
            gello_config = {
                "port": "/dev/ttyUSB0",
                "joint_ids": (1, 2, 3, 4, 5, 6, 7),
                "joint_offsets": (1 * np.pi, 1 * np.pi, 0 * np.pi, 1 * np.pi, 1 * np.pi, 1 * np.pi, 1.25 * np.pi),
                "joint_signs": (1, -1, 1, 1, 1, -1, 1),
                "gripper_config": None,
            }
        
        dynamixel_config = DynamixelRobotConfig(
            joint_ids=gello_config["joint_ids"],
            joint_offsets=gello_config["joint_offsets"],
            joint_signs=gello_config["joint_signs"],
            gripper_config=gello_config["gripper_config"],
        )

        port_path = gello_config["port"]

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
        self.get_logger().info("GELLO configuration loaded successfully")
        
        # Log the loaded configuration
        self.get_logger().info(f"Port: {port_path}")
        self.get_logger().info(f"Joint offsets: {gello_config['joint_offsets']}")
        self.get_logger().info(f"Joint signs: {gello_config['joint_signs']}")

        self.smoother = Smoother(
            publish_hz=50.0,
            max_velocity=0.3,
            max_acceleration=1.0,
            smooth_threshold=0.5,
            joint_limits=JOINT_LIMITS
        )
        self.smoother.set_initial_position(DEFAULT_JOINTS.copy())
        self.smoother.set_follower_enabled(True)

        self.joint_publisher = self.create_publisher(
            Float64MultiArray,
            '/dual_joint_impedance/joints_desired',
            10
        )

        self.timer = self.create_timer(0.02, self.timer_callback)

        gello_joints = self.gello_agent.act({})
        if gello_joints is not None and len(gello_joints) >= 7:
            q_first = np.asarray(gello_joints[:7], dtype=float)
            self.get_logger().info(
                'First Gello joint read (after offset correction): ' +
                ', '.join(f'{float(val):.4f}' for val in q_first)
            )

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

            arm_joints = np.asarray(arm_joints, dtype=float)
            arm_joints = np.clip(arm_joints, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
            arm_joints = arm_joints[:7]

            self.smoother.set_target(arm_joints)
            smoothed_joints = self.smoother.update()

            if CONTROL_MODE == LEFT_ARM:
                dual_joints = list(smoothed_joints) + list(DEFAULT_JOINTS)
            elif CONTROL_MODE == RIGHT_ARM:
                dual_joints = list(DEFAULT_JOINTS) + list(smoothed_joints)
            else:
                dual_joints = list(smoothed_joints) + list(smoothed_joints)

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
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='GELLO Franka ROS2 Bridge')
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
        gello_bridge = GelloFrankaBridge(config_path=config_path)
        rclpy.spin(gello_bridge)
    except KeyboardInterrupt:
        pass
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        if gello_bridge is not None:
            gello_bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()