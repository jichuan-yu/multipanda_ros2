#!/usr/bin/env python3
"""Offline scheduling check for gripper_action_bridge_real (panda-style bridge).

Starts fake grasp/homing action servers and a stop service for arm_id=test, runs
the real bridge binary, streams <test>_gripper/width_desired at 50 Hz the way
gello_franka_ros2.py does, and verifies that the bridge

  * homes the hand before the first command,
  * never has two goals in flight (panda.py's blocking goto() semantics),
  * collapses the 50 Hz stream (latest wins + deadband),
  * ends on the last published width.

Usage:
  python3 gripper_bridge_panda_style_test.py [path/to/gripper_action_bridge_real]
"""

import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger

from franka_msgs.action import Grasp, Homing, Move

ARM = "test"
MOTION_TIME = 0.15  # simulated traversal time of one gripper command [s]
PUBLISH_HZ = 50.0
RAMP_TIME = 2.0  # duration of the teleop-like stream [s]
SETTLE_TIME = 0.6  # extra wait so the final command can finish [s]

DEFAULT_BRIDGE = (
    "/home/botao/dual_panda_ws/install/franka_example_controllers/lib/"
    "franka_example_controllers/gripper_action_bridge_real"
)


class FakeGripper(Node):
    def __init__(self):
        super().__init__("fake_gripper")
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.in_flight = 0
        self.max_in_flight = 0
        self.goals = []  # [(start_monotonic, width)] grasp or move goals, whichever runs
        self.homing_count = 0
        self.stop_calls = 0

        self.grasp_server = ActionServer(
            self, Grasp, f"{ARM}_gripper/grasp", self.execute_grasp, callback_group=self.group
        )
        self.move_server = ActionServer(
            self, Move, f"{ARM}_gripper/move", self.execute_move, callback_group=self.group
        )
        self.homing_server = ActionServer(
            self, Homing, f"{ARM}_gripper/homing", self.execute_homing, callback_group=self.group
        )
        self.stop_srv = self.create_service(
            Trigger, f"{ARM}_gripper/stop", self.on_stop, callback_group=self.group
        )
        self.width_pub = self.create_publisher(Float64, f"{ARM}_gripper/width_desired", 10)
        self.stop_pub = self.create_publisher(Bool, f"{ARM}_gripper/stop_desired", 10)

    def _begin_goal(self, width):
        with self.lock:
            self.in_flight += 1
            self.max_in_flight = max(self.max_in_flight, self.in_flight)
            self.goals.append((time.monotonic(), width))
        # Simulated finger travel; a second goal arriving in this window would
        # preempt it on the real robot, which is exactly what we must not do.
        time.sleep(MOTION_TIME)
        with self.lock:
            self.in_flight -= 1

    def execute_grasp(self, goal_handle):
        self._begin_goal(goal_handle.request.width)
        goal_handle.succeed()
        return Grasp.Result(success=True, error="")

    def execute_move(self, goal_handle):
        self._begin_goal(goal_handle.request.width)
        goal_handle.succeed()
        return Move.Result(success=True, error="")

    def execute_homing(self, goal_handle):
        with self.lock:
            self.homing_count += 1
        goal_handle.succeed()
        return Homing.Result(success=True, error="")

    def on_stop(self, request, response):
        with self.lock:
            self.stop_calls += 1
        response.success = True
        response.message = "stopped"
        return response


def main() -> int:
    positional = [a for a in sys.argv[1:] if not a.startswith("--")]
    use_move = "--use-move" in sys.argv[1:]  # exercise the use_grasp:=false path
    bridge_bin = positional[0] if positional else DEFAULT_BRIDGE
    if not os.path.isfile(bridge_bin):
        print(f"[FAIL] bridge binary not found: {bridge_bin}")
        return 2
    print(f"testing {'Move (use_grasp:=false)' if use_move else 'Grasp'} path")

    rclpy.init()
    fake = FakeGripper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fake)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(0.5)

    log_path = "/tmp/gripper_bridge_panda_style_test.log"
    log_file = open(log_path, "w")
    params = [
        "--ros-args",
        "-r",
        "__node:=test_gripper_bridge",
        "-p",
        f"arm_id:={ARM}",
        "-p",
        "speed:=1.0",
        "-p",
        "force:=10.0",
    ]
    if use_move:
        params += ["-p", "use_grasp:=false"]
    bridge = subprocess.Popen(
        [bridge_bin] + params,
        stdout=log_file,
        stderr=subprocess.STDOUT,
    )

    # Teleop-like stream: 50 Hz, one float per sample (what GELLO publishes).
    published = []
    t0 = time.monotonic()
    period = 1.0 / PUBLISH_HZ
    while time.monotonic() - t0 < RAMP_TIME:
        phase = min(1.0, (time.monotonic() - t0) / RAMP_TIME)
        width = 0.060 * phase
        fake.width_pub.publish(Float64(data=width))
        published.append(width)
        time.sleep(period)

    final_width = 0.0423  # hold a distinct target so "latest wins" is visible
    hold_deadline = time.monotonic() + SETTLE_TIME
    while time.monotonic() < hold_deadline:
        fake.width_pub.publish(Float64(data=final_width))
        published.append(final_width)
        time.sleep(period)

    # stop_desired must reach the /stop service without waiting for the goal that
    # is currently running.
    fake.stop_pub.publish(Bool(data=True))
    time.sleep(0.3)

    time.sleep(0.3)
    with fake.lock:
        goals = list(fake.goals)
        max_in_flight = fake.max_in_flight
        homing_count = fake.homing_count
        stop_calls = fake.stop_calls

    bridge.terminate()
    try:
        bridge.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        bridge.kill()
    log_file.close()
    executor.shutdown()
    spin_thread.join(timeout=2.0)
    fake.destroy_node()
    rclpy.shutdown()

    ok = True

    def check(condition, message):
        nonlocal ok
        print(("[PASS] " if condition else "[FAIL] ") + message)
        ok = ok and condition

    check(homing_count >= 1, f"hand homed before the first command (homing goals: {homing_count})")
    check(max_in_flight == 1, f"goals never overlapped (max in flight: {max_in_flight})")
    check(stop_calls >= 1, f"stop_desired forwarded to the /stop service (calls: {stop_calls})")
    check(
        len(goals) > 0,
        f"bridge forwarded commands ({len(goals)} {'move' if use_move else 'grasp'} goals)",
    )
    check(
        len(published) > 5 * len(goals),
        f"50 Hz stream collapsed: {len(goals)} goals for {len(published)} samples",
    )

    if goals:
        gaps = [b[0] - a[0] for a, b in zip(goals, goals[1:])]
        min_gap = min(gaps) if gaps else float("nan")
        check(
            not gaps or min_gap >= 0.9 * MOTION_TIME,
            f"consecutive goals spaced by the motion time (min gap: {min_gap:.3f}s)",
        )
        check(
            math.isclose(goals[-1][1], final_width, abs_tol=1e-6),
            f"last commanded width {goals[-1][1]:.4f} m == last published {final_width:.4f} m",
        )

    print(f"\nbridge log: {log_path}")
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
