#!/usr/bin/env python3
"""Forward-motion teleop sender for time-axis alignment experiments.

Publishes type=5 (TASK_SPACE_POSE) commands to ``/dualarm_teleop_cmd`` on a fixed
timer, moving both end-effectors forward (+X, world frame) by a fixed step each
tick. Every publish is time-stamped (wall clock) to a CSV so the send grid can be
aligned against the MPC node's callback timestamps.

Usage:
    python3 src/multipanda_ros2/teleop/teleop_sender_forward.py \
        --interval-ms 5 --step-m 0.001 --max-sends 200
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import threading
import time

import numpy as np
import rclpy

# Allow importing teleop.base_teleop when run directly.
if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

# Import base teleop node (mirrors csv_teleop_cartesian_absolute.py)
try:
    from teleop.base_teleop import BaseTeleopNode
except ImportError:
    # Fallback for direct execution: the script's own dir is on sys.path.
    from base_teleop import BaseTeleopNode


def _parse_pose(s: str) -> np.ndarray:
    """Parse a comma-separated ``[x,y,z,qx,qy,qz,qw]`` string into a float array."""
    vals = [float(x) for x in s.split(",")]
    if len(vals) != 7:
        raise ValueError(f"Expected 7 comma-separated values, got {len(vals)}: {s!r}")
    return np.array(vals, dtype=np.float64)


class ForwardTeleopSender(BaseTeleopNode):
    """Publish forward-moving type=5 absolute poses on a fixed timer."""

    def __init__(
        self,
        interval_ms: float = 5.0,
        step_m: float = 0.001,
        left_init: np.ndarray = None,
        right_init: np.ndarray = None,
        command_type: int = 5,
        arm_selector: int = 3,
        allow_safety_violation: bool = False,
        max_sends: int = 0,
        out_dir: str = None,
    ):
        super().__init__("forward_teleop_sender")

        self.interval_ms = interval_ms
        self.step_m = step_m
        self.command_type = command_type
        self.arm_selector = arm_selector
        self.allow_safety_violation = allow_safety_violation
        self.max_sends = max_sends

        if left_init is None:
            left_init = np.array([0.307, 0.26, 0.487, 1.0, 0.0, 0.0, 0.0])
        if right_init is None:
            right_init = np.array([0.307, -0.26, 0.487, 1.0, 0.0, 0.0, 0.0])
        self.left_pose = np.array(left_init, dtype=np.float64).copy()
        self.right_pose = np.array(right_init, dtype=np.float64).copy()

        # Send-timestamp CSV
        if out_dir is None:
            out_dir = "/home/csc/csc_ws/work_file/8.11-8.18/mpc_experiment_record"
        os.makedirs(out_dir, exist_ok=True)
        self._csv_path = os.path.join(
            out_dir, f"sender_timestamps_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(["seq", "send_time_ns", "send_time_s", "left_x", "right_x"])

        self.seq = 0
        self._done = False

        self.control_timer = self.create_timer(interval_ms / 1000.0, self.send_callback)

        self.get_logger().info(
            f"Forward sender: interval={interval_ms}ms step={step_m}m "
            f"command_type={command_type} arm={arm_selector} max_sends={max_sends}"
        )
        print(f"[Sender] writing send timestamps to {self._csv_path}")

    @property
    def done(self) -> bool:
        return self._done

    def send_callback(self):
        """Publish one forward-stepped type=5 command and time-stamp it."""
        if self._done:
            return

        self.seq += 1
        self.left_pose[0] += self.step_m
        self.right_pose[0] += self.step_m

        msg = self.create_teleop_command(
            arm_selector=self.arm_selector,
            command_type=self.command_type,
            left_ee_delta=self.left_pose,
            right_ee_delta=self.right_pose,
            allow_safety_violation=self.allow_safety_violation,
        )
        self.teleop_pub.publish(msg)

        send_ns = time.time_ns()
        self._csv_writer.writerow(
            [self.seq, send_ns, send_ns / 1e9, self.left_pose[0], self.right_pose[0]]
        )

        if self.max_sends > 0 and self.seq >= self.max_sends:
            self._done = True
            self.control_timer.cancel()
            self._csv_file.flush()
            self.get_logger().info(f"Reached max_sends={self.max_sends}, stopping.")
            print(f"[Sender] done: {self.seq} commands sent -> {self._csv_path}")

    def shutdown(self):
        """Close the send-timestamp CSV."""
        if not self._csv_file.closed:
            self._csv_file.close()


def main(args=None):
    parser = argparse.ArgumentParser(description="Forward-motion type=5 teleop sender")
    parser.add_argument("--interval-ms", type=float, default=5.0)
    parser.add_argument("--step-m", type=float, default=0.001)
    parser.add_argument("--left-init", type=str, default="0.307,0.26,0.487,1,0,0,0")
    parser.add_argument("--right-init", type=str, default="0.307,-0.26,0.487,1,0,0,0")
    parser.add_argument("--command-type", type=int, default=5)
    parser.add_argument("--arm-selector", type=int, default=3)
    parser.add_argument("--allow-safety", action="store_true")
    parser.add_argument("--max-sends", type=int, default=0)
    parser.add_argument(
        "--out-dir",
        type=str,
        default="/home/csc/csc_ws/work_file/8.11-8.18/mpc_experiment_record",
    )
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)

    try:
        node = ForwardTeleopSender(
            interval_ms=parser_args.interval_ms,
            step_m=parser_args.step_m,
            left_init=_parse_pose(parser_args.left_init),
            right_init=_parse_pose(parser_args.right_init),
            command_type=parser_args.command_type,
            arm_selector=parser_args.arm_selector,
            allow_safety_violation=parser_args.allow_safety,
            max_sends=parser_args.max_sends,
            out_dir=parser_args.out_dir,
        )

        thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        thread.start()

        while rclpy.ok() and not node.done:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Sender] interrupted, shutting down...")
    except Exception as e:
        print(f"\n[Sender] error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if "node" in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()
        print("[Sender] stopped.")


if __name__ == "__main__":
    main()
