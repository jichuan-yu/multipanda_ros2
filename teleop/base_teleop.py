"""
Base class for all teleoperation nodes.
Provides common functionality for ROS2 initialization, command publishing, and safety features.

Uses standard ROS2 Float64MultiArray message format for teleoperation commands.
Message format: [data14, command_type, arm_selector, allow_safety_violation]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
import numpy as np
from typing import Optional

# Command type constants
JOINT_POSITION_INCREMENT = 0
JOINT_VELOCITY = 1
TASK_SPACE_INCREMENT = 2
TASK_SPACE_VELOCITY = 3

# Arm selector constants
LEFT_ARM = 1
RIGHT_ARM = 2
BOTH_ARMS = 3


class BaseTeleopNode(Node):
    """Base class for all teleoperation nodes."""

    # Arm selector constants
    LEFT_ARM = LEFT_ARM
    RIGHT_ARM = RIGHT_ARM
    BOTH_ARMS = BOTH_ARMS

    # Command type constants
    JOINT_POSITION_INCREMENT = JOINT_POSITION_INCREMENT
    JOINT_VELOCITY = JOINT_VELOCITY
    TASK_SPACE_INCREMENT = TASK_SPACE_INCREMENT
    TASK_SPACE_VELOCITY = TASK_SPACE_VELOCITY

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
            Float64MultiArray,
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
        left_ee_delta: Optional[np.ndarray] = None,
        right_ee_delta: Optional[np.ndarray] = None,
        allow_safety_violation: bool = False
    ) -> Float64MultiArray:
        """
        Create a Float64MultiArray teleop command message.

        Message format: [data14, command_type, arm_selector, allow_safety_violation]
        - data14: 14 data elements (format depends on command_type)
          * Joint space: [left_joint1-7, right_joint1-7]
          * Task space: [left_x,y,z,qx,qy,qz, right_x,y,z,qx,qy,qz, 0, 0]
        - command_type: 0=JOINT_POSITION_INCREMENT, 1=JOINT_VELOCITY,
                       2=TASK_SPACE_INCREMENT, 3=TASK_SPACE_VELOCITY
        - arm_selector: 1=LEFT_ARM, 2=RIGHT_ARM, 3=BOTH_ARMS
        - allow_safety_violation: 0.0=false, 1.0=true

        Args:
            arm_selector: LEFT_ARM, RIGHT_ARM, or BOTH_ARMS
            command_type: JOINT_POSITION_INCREMENT, TASK_SPACE_INCREMENT, etc.
            left_joint_delta: 7-element array for left arm joint increments
            right_joint_delta: 7-element array for right arm joint increments
            left_ee_delta: 6-element array [x,y,z,qx,qy,qz] for left arm EE increment
            right_ee_delta: 6-element array [x,y,z,qx,qy,qz] for right arm EE increment
            allow_safety_violation: Whether to disable collision avoidance

        Returns:
            Float64MultiArray message with teleop command
        """
        msg = Float64MultiArray()

        if command_type in [JOINT_POSITION_INCREMENT, JOINT_VELOCITY]:
            # Joint space command: 14 joint values
            data = np.zeros(14)

            if left_joint_delta is not None:
                data[0:7] = left_joint_delta
            else:
                data[0:7] = 0.0

            if right_joint_delta is not None:
                data[7:14] = right_joint_delta
            else:
                data[7:14] = 0.0

        elif command_type in [TASK_SPACE_INCREMENT, TASK_SPACE_VELOCITY]:
            # Task space command: 12 task space values + 2 padding
            data = np.zeros(14)

            if left_ee_delta is not None:
                data[0:6] = left_ee_delta
            # else: already zeros

            if right_ee_delta is not None:
                data[6:12] = right_ee_delta
            # else: already zeros

            # Indices 12-13 are padding (unused) for task space commands
            data[12:14] = 0.0
        else:
            self.get_logger().warn(f"Unknown command type: {command_type}")
            data = np.zeros(14)

        # Assemble full message: [data14, command_type, arm_selector, safety_flag]
        msg.data = np.concatenate([
            data,
            [command_type],
            [arm_selector],
            [1.0 if allow_safety_violation else 0.0]
        ]).tolist()

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
