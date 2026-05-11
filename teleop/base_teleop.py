"""
Base class for all teleoperation nodes.
Provides common functionality for ROS2 initialization, command publishing, and safety features.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from dualarm_reactive_control.msg import TeleopCommand
import numpy as np
from typing import Optional


class BaseTeleopNode(Node):
    """Base class for all teleoperation nodes."""

    # Arm selector constants (from TeleopCommand.msg)
    LEFT_ARM = TeleopCommand.LEFT_ARM
    RIGHT_ARM = TeleopCommand.RIGHT_ARM
    BOTH_ARMS = TeleopCommand.BOTH_ARMS

    # Command type constants
    JOINT_POSITION_INCREMENT = TeleopCommand.JOINT_POSITION_INCREMENT
    JOINT_VELOCITY = TeleopCommand.JOINT_VELOCITY
    TASK_SPACE_INCREMENT = TeleopCommand.TASK_SPACE_INCREMENT
    TASK_SPACE_VELOCITY = TeleopCommand.TASK_SPACE_VELOCITY

    # Panda joint limits
    JOINT_LIMITS_LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    JOINT_LIMITS_UPPER = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    # Default home position
    HOME_POSITION = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

    def __init__(
        self,
        node_name: str,
        publish_rate: float = 100.0,
        use_sim_time: bool = True
    ):
        """
        Initialize base teleop node.

        Args:
            node_name: Name of the ROS2 node
            publish_rate: Publishing rate in Hz
            use_sim_time: Whether to use simulation time
        """
        super().__init__(node_name)

        # Set use_sim_time parameter
        self.set_parameters([
            rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, use_sim_time)
        ])

        # Create publisher with reliable QoS for teleop commands
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.teleop_pub = self.create_publisher(
            TeleopCommand,
            '/dualarm_teleop_cmd',
            qos
        )

        self.publish_rate = publish_rate
        self.publish_period = 1.0 / publish_rate

        # Current joint states (left and right arms)
        self.current_joint_states = {
            'left': self.HOME_POSITION.copy(),
            'right': self.HOME_POSITION.copy()
        }

        # Arm selection
        self.selected_arm = 'left'  # 'left', 'right', or 'both'

        # Safety parameters
        self.max_joint_increment = 0.002  # rad
        self.max_position_increment = 0.002  # m
        self.max_rotation_increment = 0.01  # rad

        self.get_logger().info(f'{node_name} initialized')

    def create_teleop_command(
        self,
        arm_selector: int,
        command_type: int,
        left_joint_delta: Optional[np.ndarray] = None,
        right_joint_delta: Optional[np.ndarray] = None,
        left_ee_delta: Optional[Pose] = None,
        right_ee_delta: Optional[Pose] = None,
        allow_safety_violation: bool = False
    ) -> TeleopCommand:
        """
        Create a TeleopCommand message.

        Args:
            arm_selector: LEFT_ARM, RIGHT_ARM, or BOTH_ARMS
            command_type: JOINT_POSITION_INCREMENT, TASK_SPACE_INCREMENT, etc.
            left_joint_delta: 7-element array for left arm joint increments
            right_joint_delta: 7-element array for right arm joint increments
            left_ee_delta: Pose message for left arm end-effector increment
            right_ee_delta: Pose message for right arm end-effector increment
            allow_safety_violation: Whether to disable collision avoidance

        Returns:
            TeleopCommand message
        """
        msg = TeleopCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.arm_selector = arm_selector
        msg.command_type = command_type
        msg.allow_safety_violation = allow_safety_violation

        # Set joint deltas
        if left_joint_delta is not None:
            msg.left_joint_delta = left_joint_delta.tolist()
        else:
            msg.left_joint_delta = [0.0] * 7

        if right_joint_delta is not None:
            msg.right_joint_delta = right_joint_delta.tolist()
        else:
            msg.right_joint_delta = [0.0] * 7

        # Set end-effector deltas
        if left_ee_delta is not None:
            msg.left_ee_delta = left_ee_delta
        else:
            msg.left_ee_delta = Pose()

        if right_ee_delta is not None:
            msg.right_ee_delta = right_ee_delta
        else:
            msg.right_ee_delta = Pose()

        return msg

    def clamp_joint_delta(self, delta: np.ndarray) -> np.ndarray:
        """
        Clamp joint increment to safety limits.

        Args:
            delta: 7-element joint increment array

        Returns:
            Clamped joint increment
        """
        max_delta = np.full(7, self.max_joint_increment)
        return np.clip(delta, -max_delta, max_delta)

    def clamp_joint_position(self, q: np.ndarray) -> np.ndarray:
        """
        Clamp joint position to Panda limits.

        Args:
            q: 7-element joint position array

        Returns:
            Clamped joint position
        """
        return np.clip(q, self.JOINT_LIMITS_LOWER, self.JOINT_LIMITS_UPPER)

    def get_arm_selector(self) -> int:
        """Get arm selector constant based on current selection."""
        if self.selected_arm == 'left':
            return self.LEFT_ARM
        elif self.selected_arm == 'right':
            return self.RIGHT_ARM
        else:
            return self.BOTH_ARMS

    def set_selected_arm(self, arm: str):
        """
        Set the currently selected arm.

        Args:
            arm: 'left', 'right', or 'both'
        """
        if arm in ['left', 'right', 'both']:
            self.selected_arm = arm
        else:
            self.get_logger().warn(f"Invalid arm selection: {arm}")

    def print_usage(self, usage_text: str):
        """Print usage instructions."""
        print(usage_text, flush=True)
