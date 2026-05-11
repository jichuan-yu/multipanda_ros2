#!/usr/bin/env python3
"""
Single-arm joint teleop for real Panda robot.

Controls:
  1-7 : Select joint 1..7
  w / Up Arrow   : Increase selected joint by step (default 0.03 rad)
  s / Down Arrow : Decrease selected joint by step
  Ctrl-C         : Quit

Publishes `sensor_msgs.msg.JointState` to `/joint_impedance/joints_desired` (controller input).
"""

import sys
import select
import termios
import tty
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


@dataclass
class KeyPubConfig:
    arm_id: str = 'panda'
    topic: str = '/joint_impedance/joints_desired'
    publish_hz: float = 10.0  # rate used to compute per-press increment
    scale: float = 0.05       # rad/s when key held down, used to compute per-press increment
    use_sim_time: bool = True


class KeyJointSinglearmTeleop(Node):
    def __init__(self, config: KeyPubConfig = None):
        super().__init__('key_joint_singlearm_teleop')

        self.config = config if config is not None else KeyPubConfig()
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, self.config.use_sim_time)])

        # Publisher to controller input
        self.pub = self.create_publisher(JointState, self.config.topic, 10)

        # Subscribe to /joint_states to read current joint values
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        self.arm_id = self.config.arm_id
        self.joint_names = [f'{self.arm_id}_joint{i+1}' for i in range(7)]
        self.state_lock = threading.Lock()

        # Current joint positions (initialized to zeros until /joint_states arrives)
        self.q_current = np.zeros(7, dtype=float)
        self.q_cmd = None
        self.joint_states_received = False
        self.error_log_period = 2.0
        self._last_error_log_time = 0.0

        # Teleop state
        self.selected_joint = 0
        self.publish_hz = float(self.config.publish_hz)
        self.scale = float(self.config.scale)
        # per-press increment computed from scale/publish_hz for configurable behaviour
        self.step = float(self.scale) / max(self.publish_hz, 1.0)

        self.get_logger().info('Single-arm joint teleop initialized')
        self.print_usage()

        # Held-key detection and continuous send state
        self.held_action = None  # 'inc' or 'dec' when in hold-mode
        self.last_key_char = None
        self.last_key_time = 0.0
        self.repeat_count = 0
        # windows (s)
        self.repeat_detection_window = 0.1
        self.repeat_timeout = 0.1

    def held_timer_callback(self):
        """Timer callback to publish continuous commands while a key is held."""
        now = time.monotonic()
        if self.held_action is not None:
            if now - self.last_key_time > self.repeat_timeout:
                self.held_action = None
            else:
                # call update_joint to apply one step
                self.update_joint((self.held_action, None))

        self.log_joint_error()

    def print_usage(self):
        msg = f"""
            Single-arm Joint Teleop (arm: {self.arm_id})
            Select joint: keys 1..7
            Increase selected joint: w or Up Arrow
            Decrease selected joint: s or Down Arrow
            Ctrl-C to quit
            Current selected joint: {self.selected_joint+1}
        """
        print(msg, flush=True)

    def joint_state_callback(self, msg: JointState):
        try:
            indices = []
            for i in range(7):
                name = self.joint_names[i]
                if name in msg.name:
                    indices.append(msg.name.index(name))
                else:
                    indices.append(None)

            # If all joints found, update q_current
            if all(idx is not None for idx in indices):
                with self.state_lock:
                    self.q_current = np.array([msg.position[idx] for idx in indices], dtype=float)
                if not self.joint_states_received:
                    self.joint_states_received = True
                    print('\nJoint states received. Ready for control!', flush=True)
        except Exception as e:
            # ignore parsing errors
            pass

    def publish_command(self, q_cmd):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q_cmd.tolist()
        msg.velocity = [0.0] * 7
        self.pub.publish(msg)
        with self.state_lock:
            self.q_cmd = q_cmd.copy()

    def log_joint_error(self):
        now = time.monotonic()
        if now - self._last_error_log_time < self.error_log_period:
            return
        if not self.joint_states_received:
            return

        with self.state_lock:
            if self.q_cmd is None:
                return
            q_current = self.q_current.copy()
            q_cmd = self.q_cmd.copy()

        q_err = q_cmd - q_current
        err_norm = float(np.linalg.norm(q_err))
        self.get_logger().info(
            f'Joint error (q_cmd - q): {np.array2string(q_err, precision=5, suppress_small=True)} '
            f'| norm={err_norm:.6f} rad'
        )
        self._last_error_log_time = now

    def update_joint(self, action):
        """action can be: ('select', idx) or ('inc', None) or ('dec', None)"""
        if action[0] == 'select':
            idx = action[1]
            if 0 <= idx < 7:
                self.selected_joint = idx
                print(f"\nSelected joint: {self.selected_joint+1}", flush=True)
            return

        if not self.joint_states_received:
            print('No joint_states yet; cannot send command', flush=True)
            return

        with self.state_lock:
            q_target = self.q_current.copy()
        if action[0] == 'inc':
            q_target[self.selected_joint] += self.step
        elif action[0] == 'dec':
            q_target[self.selected_joint] -= self.step
        else:
            return

        # Publish the desired joint positions as controller input
        self.publish_command(q_target)
        print(f"Published command. joint {self.selected_joint+1}: {q_target[self.selected_joint]:.4f}", flush=True)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = ''
    if rlist:
        ch1 = sys.stdin.read(1)
        if ch1 == '\x1b':
            # possible arrow key; attempt to read two more chars
            rlist2, _, _ = select.select([sys.stdin], [], [], 0.01)
            if rlist2:
                ch2 = sys.stdin.read(1)
                rlist3, _, _ = select.select([sys.stdin], [], [], 0.01)
                if rlist3:
                    ch3 = sys.stdin.read(1)
                    seq = ch1 + ch2 + ch3
                    if seq == '\x1b[A':
                        key = 'UP'
                    elif seq == '\x1b[B':
                        key = 'DOWN'
                    else:
                        key = seq
                else:
                    key = ch1 + ch2
            else:
                key = ch1
        else:
            key = ch1
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    config = KeyPubConfig()
    node = KeyJointSinglearmTeleop(config=config)

    settings = termios.tcgetattr(sys.stdin)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Timer to publish continuous commands when a key is held (uses node.held_timer_callback)
    node.create_timer(1.0 / max(node.publish_hz, 1.0), node.held_timer_callback)

    try:
        while rclpy.ok():
            key = get_key(settings)
            if not key:
                time.sleep(0.01)
                continue

            if key == '\x03':  # Ctrl-C
                break

            key = key.lower()
            # Numeric joint select
            if key in [str(i) for i in range(1, 8)]:
                idx = int(key) - 1
                # selecting a joint cancels any hold
                node.held_action = None
                node.repeat_count = 0
                node.last_key_char = None
                node.update_joint(('select', idx))
                continue

            now = time.monotonic()

            # w / s or arrows -> increment/decrement
            if key in ('w', 'up', 'UP'):
                # immediate single-step
                node.update_joint(('inc', None))
                # detect repeats to enter hold-mode
                if node.last_key_char == key and (now - node.last_key_time) < node.repeat_detection_window:
                    node.repeat_count += 1
                else:
                    node.repeat_count = 1
                node.last_key_char = key
                node.last_key_time = now
                if node.repeat_count >= 2:
                    node.held_action = 'inc'
                continue

            if key in ('s', 'down', 'DOWN'):
                node.update_joint(('dec', None))
                if node.last_key_char == key and (now - node.last_key_time) < node.repeat_detection_window:
                    node.repeat_count += 1
                else:
                    node.repeat_count = 1
                node.last_key_char = key
                node.last_key_time = now
                if node.repeat_count >= 2:
                    node.held_action = 'dec'
                continue

            # Any other key cancels hold
            node.held_action = None
            node.repeat_count = 0
            node.last_key_char = None

    except Exception as e:
        print(f'Error: {e}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
