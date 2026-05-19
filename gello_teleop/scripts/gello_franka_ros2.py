#!/usr/bin/env python3
from dataclasses import dataclass, field
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
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


@dataclass
class GelloFrankaDualArmConfig:
    arm_id: str = 'mj'
    joint_state_topic: str = '/joint_states'
    target_joint_topic: str = '/dual_joint_impedance/joints_desired'
    left_gripper_topic: str = '/mj_left_gripper/width_desired'
    right_gripper_topic: str = '/mj_right_gripper/width_desired'
    gello_port: str = '/dev/ttyUSB1'
    publish_hz: float = 50.0
    control_mode: int = RIGHT_ARM
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
    def __init__(self, config: GelloFrankaDualArmConfig = None, gello_config_path: str = None):
        super().__init__('gello_franka_bridge')

        self.config = config if config is not None else GelloFrankaDualArmConfig()
        self.arm_id = self.config.arm_id
        self.joint_state_topic = self.config.joint_state_topic
        self.target_joint_topic = self.config.target_joint_topic
        self.left_gripper_topic = self.config.left_gripper_topic
        self.right_gripper_topic = self.config.right_gripper_topic
        self.control_mode = self.config.control_mode
        self.publish_hz = float(self.config.publish_hz)
        self.joint_limits = np.asarray(self.config.joint_limits, dtype=float)
        self.default_joint_target = np.asarray(self.config.default_joint_target, dtype=float)

        self.left_joint_names = [f'{self.arm_id}_left_joint{i + 1}' for i in range(7)]
        self.right_joint_names = [f'{self.arm_id}_right_joint{i + 1}' for i in range(7)]
        
        self.q_left_current = None
        self.q_right_current = None
        self.joint_states_received = False
        self._missing_joint_state_logged = False

        self.smoother = Smoother(
            publish_hz=self.publish_hz,
            max_velocity=0.3,
            position_tolerance=0.05,
            joint_limits=self.joint_limits,
            logger=self.get_logger()
        )
        self.smoother_initialized = False
        self.latest_raw_target = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.joint_publisher = self.create_publisher(Float64MultiArray, self.target_joint_topic, 10)
        self.left_gripper_publisher = self.create_publisher(Float64, self.left_gripper_topic, 10)
        self.right_gripper_publisher = self.create_publisher(Float64, self.right_gripper_topic, 10)

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

        self.get_logger().info(f"GELLO Port: {gello_config['port']}")
        self.get_logger().info(f"Joint offsets: {gello_config['joint_offsets']}")
        self.get_logger().info(f"Joint signs: {gello_config['joint_signs']}")
        self.get_logger().info(f"Control mode: {self.control_mode} (1=LEFT, 2=RIGHT, 3=BOTH)")

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
        self.get_logger().info('Gello Franka dual-arm teleop node initialized (direct control mode with smoother).')

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
            left_indices = [msg.name.index(name) if name in msg.name else None for name in self.left_joint_names]
            right_indices = [msg.name.index(name) if name in msg.name else None for name in self.right_joint_names]

            if all(idx is not None for idx in left_indices):
                self.q_left_current = np.array([msg.position[idx] for idx in left_indices], dtype=float)
            elif not self._missing_joint_state_logged:
                missing = [self.left_joint_names[i] for i, idx in enumerate(left_indices) if idx is None]
                self.get_logger().warn(f'Left arm joint state missing: {missing}')

            if all(idx is not None for idx in right_indices):
                self.q_right_current = np.array([msg.position[idx] for idx in right_indices], dtype=float)
            elif not self._missing_joint_state_logged:
                missing = [self.right_joint_names[i] for i, idx in enumerate(right_indices) if idx is None]
                self.get_logger().warn(f'Right arm joint state missing: {missing}')

            if ((self.control_mode == LEFT_ARM and self.q_left_current is not None) or
                (self.control_mode == RIGHT_ARM and self.q_right_current is not None) or
                (self.control_mode == BOTH_ARMS and self.q_left_current is not None and self.q_right_current is not None)):
                
                if self.control_mode == LEFT_ARM:
                    current_pos = self.q_left_current.copy()
                elif self.control_mode == RIGHT_ARM:
                    current_pos = self.q_right_current.copy()
                else:
                    current_pos = self.q_left_current.copy()

                self.smoother.set_actual_position(current_pos)

                if not self.joint_states_received:
                    self.joint_states_received = True
                    self.smoother.set_initial_position(current_pos)
                    initial_target = self.latest_raw_target if self.latest_raw_target is not None else current_pos.copy()
                    self.smoother.set_target(initial_target)
                    self.smoother.set_follower_enabled(True)
                    self.smoother_initialized = True
                    self.get_logger().info('Smoother initialized from actual robot joint positions.')

        except Exception as exc:
            self.get_logger().warn(f'Failed to parse joint_states: {exc}')

    def timer_callback(self):
        try:
            gello_joints = self.gello_agent.act({})
            if gello_joints is None or len(gello_joints) < 7:
                self.get_logger().warn('Invalid Gello joint reading, skipping publish.')
                return
            
            q_target_raw = np.asarray(gello_joints[:7], dtype=float)
            q_target_raw = np.clip(q_target_raw, self.joint_limits[:, 0], self.joint_limits[:, 1])
            self.latest_raw_target = q_target_raw

            if self.smoother_initialized:
                if self.control_mode == LEFT_ARM and self.q_left_current is not None:
                    self.smoother.set_actual_position(self.q_left_current)
                elif self.control_mode == RIGHT_ARM and self.q_right_current is not None:
                    self.smoother.set_actual_position(self.q_right_current)
                elif self.control_mode == BOTH_ARMS and self.q_left_current is not None:
                    self.smoother.set_actual_position(self.q_left_current)
                else:
                    return

                self.smoother.set_target(q_target_raw)
                smoothed_joints = self.smoother.update()

                if self.control_mode == LEFT_ARM:
                    dual_joints = list(smoothed_joints) + list(self.default_joint_target)
                elif self.control_mode == RIGHT_ARM:
                    dual_joints = list(self.default_joint_target) + list(smoothed_joints)
                else:
                    dual_joints = list(smoothed_joints) + list(smoothed_joints)

                msg = Float64MultiArray()
                msg.data = dual_joints
                self.joint_publisher.publish(msg)
            else:
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
                
                msg_gripper = Float64()
                msg_gripper.data = float(gripper_width)

                if self.control_mode == LEFT_ARM:
                    self.left_gripper_publisher.publish(msg_gripper)
                elif self.control_mode == RIGHT_ARM:
                    self.right_gripper_publisher.publish(msg_gripper)
                else:
                    self.left_gripper_publisher.publish(msg_gripper)
                    self.right_gripper_publisher.publish(msg_gripper)
            
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
    rclpy.init(args=args)
    node = None
    
    import argparse
    parser = argparse.ArgumentParser(description='GELLO Franka Dual Arm ROS2 Bridge')
    parser.add_argument('--config', type=str, default=None, help='Path to GELLO YAML config file')
    parser.add_argument('--control-mode', type=int, default=RIGHT_ARM, 
                        help='Control mode: 1=LEFT_ARM, 2=RIGHT_ARM, 3=BOTH_ARMS')
    parsed_args, _ = parser.parse_known_args(args)

    config_path = parsed_args.config
    if config_path is None:
        script_dir = Path(__file__).resolve().parent
        package_config = script_dir.parent / 'config' / 'yam_auto_generated.yaml'
        
        if package_config.exists():
            config_path = str(package_config)
            print(f"Using package config: {config_path}")
        else:
            print(f"Config not found: {package_config}")
            print("Using hardcoded default configuration")

    try:
        config_kwargs = {}
        if parsed_args.control_mode in [LEFT_ARM, RIGHT_ARM, BOTH_ARMS]:
            config_kwargs['control_mode'] = parsed_args.control_mode
        
        config = GelloFrankaDualArmConfig(**config_kwargs)
        node = GelloFrankaDualArmNode(config=config, gello_config_path=config_path)
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