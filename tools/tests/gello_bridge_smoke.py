"""Smoke test the GELLO bridge scripts without Dynamixel hardware.

The GELLO agent is replaced by a fake that always reports the same joint vector,
so this exercises the ROS-facing half of the bridges:

  * ``gello_franka_ros2.py``        -> ``/<arm>/joints_desired`` + gripper width
  * ``gello_franka_safety_ros2.py`` -> ``/dualarm_teleop_cmd`` (17-element, type=4)

Run it in an isolated DDS domain so the fake ``/joint_states`` publisher cannot be
confused with a running robot/sim:

    ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 python3 \
        src/multipanda_ros2/tools/tests/gello_bridge_smoke.py
"""
import os
if os.environ.get('ROS_DOMAIN_ID') != '221' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('Use ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1')

from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

SCRIPTS_DIR = Path(__file__).resolve().parents[2] / 'gello_teleop' / 'scripts'
CONFIG_DIR = SCRIPTS_DIR.parent / 'config'
sys.path.insert(0, str(SCRIPTS_DIR))

import gello_franka_ros2 as direct_bridge  # noqa: E402
import gello_franka_safety_ros2 as safety_bridge  # noqa: E402
import gello.agents.gello_agent as gello_agent_module  # noqa: E402

# 7 arm joints + gripper, all inside the Franka limits
FAKE_JOINTS = np.array([0.1, -0.6, 0.2, -2.0, 0.1, 1.5, 0.7], dtype=float)
FAKE_GRIPPER = 0.4
EXPECTED_WIDTH = 0.08 * (1.0 - FAKE_GRIPPER)


class FakeGelloAgent:
    """Stand-in for ``gello.agents.gello_agent.GelloAgent`` (no serial port)."""

    def __init__(self, *args, **kwargs):
        self.q = np.concatenate([FAKE_JOINTS, [FAKE_GRIPPER]])

    def act(self, obs):
        return self.q.copy()


def install_fake_gello():
    # The safety bridge imports GelloAgent lazily inside __init__, so patch the
    # module itself instead of only the already-imported names.
    gello_agent_module.GelloAgent = FakeGelloAgent
    gello_agent_module.DynamixelRobotConfig = lambda **kwargs: None
    for module in (direct_bridge, safety_bridge):
        module.GelloAgent = FakeGelloAgent
        module.DynamixelRobotConfig = lambda **kwargs: None


def collect(node_factory, joint_names, subscriptions, timeout=8.0):
    """Spin ``node_factory()`` against a fake /joint_states publisher."""
    bridge = node_factory()
    helper = Node('gello_bridge_smoke_helper')
    received = {topic: [] for topic in subscriptions}

    for topic, msg_type in subscriptions.items():
        helper.create_subscription(
            msg_type, topic, lambda msg, t=topic: received[t].append(msg), 10)
    state_pub = helper.create_publisher(JointState, '/joint_states', 10)

    state_msg = JointState()
    state_msg.name = list(joint_names)
    state_msg.position = [float(v) for v in FAKE_JOINTS] * (len(joint_names) // 7)

    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(helper)
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            state_pub.publish(state_msg)
            executor.spin_once(timeout_sec=0.01)
            if all(received.values()):
                break
    finally:
        executor.remove_node(helper)
        executor.remove_node(bridge)
        bridge.destroy_node()
        helper.destroy_node()
    return received


def joint_names_for(arm_ids):
    return [f'{arm}_joint{i}' for arm in arm_ids for i in range(1, 8)]


def check_direct(arm_ids):
    left_arm, right_arm = arm_ids
    config = direct_bridge.GelloFrankaDualArmConfig(
        left_arm_id=left_arm,
        right_arm_id=right_arm,
        control_mode=direct_bridge.BOTH_ARMS,
        left_gello_config_path=str(CONFIG_DIR / 'gello_arm1.yaml'),
        right_gello_config_path=str(CONFIG_DIR / 'gello_arm2.yaml'),
    )
    received = collect(
        lambda: direct_bridge.GelloFrankaDualArmNode(config=config),
        joint_names_for(arm_ids),
        {
            f'/{left_arm}/joints_desired': JointState,
            f'/{right_arm}/joints_desired': JointState,
            f'/{left_arm}_gripper/width_desired': Float64,
            f'/{right_arm}_gripper/width_desired': Float64,
        },
    )

    for arm in arm_ids:
        msg = received[f'/{arm}/joints_desired'][-1]
        assert list(msg.name) == [f'{arm}_joint{i}' for i in range(1, 8)], msg.name
        assert len(msg.position) == 7, msg.position
        np.testing.assert_allclose(msg.position, FAKE_JOINTS, atol=1e-6)
        width = received[f'/{arm}_gripper/width_desired'][-1].data
        assert abs(width - EXPECTED_WIDTH) < 1e-6, width
    print(f'PASS direct bridge ({left_arm}/{right_arm}): joints_desired + gripper width')


def check_safety(arm_id='panda'):
    config = safety_bridge.GelloFrankaSafetyConfig(
        arm_id=arm_id,
        hardware_mode=safety_bridge.REAL_MODE,  # system time, no /clock needed
        control_mode=safety_bridge.BOTH_ARMS,
        left_gello_config_path=str(CONFIG_DIR / 'gello_arm1.yaml'),
        right_gello_config_path=str(CONFIG_DIR / 'gello_arm2.yaml'),
    )
    received = collect(
        lambda: safety_bridge.GelloFrankaSafetyNode(config=config),
        [f'{arm_id}_left_joint{i}' for i in range(1, 8)]
        + [f'{arm_id}_right_joint{i}' for i in range(1, 8)],
        {
            '/dualarm_teleop_cmd': Float64MultiArray,
            f'/{arm_id}_left_gripper/width_desired': Float64,
            f'/{arm_id}_right_gripper/width_desired': Float64,
        },
    )

    data = list(received['/dualarm_teleop_cmd'][-1].data)
    assert len(data) == 17, len(data)
    assert int(data[14]) == 4, data[14]               # JOINT_POSITION
    assert int(data[15]) == safety_bridge.BOTH_ARMS, data[15]
    assert int(data[16]) == 0, data[16]               # allow_safety_violation
    np.testing.assert_allclose(data[0:7], FAKE_JOINTS, atol=1e-6)
    np.testing.assert_allclose(data[7:14], FAKE_JOINTS, atol=1e-6)
    for side in ('left', 'right'):
        width = received[f'/{arm_id}_{side}_gripper/width_desired'][-1].data
        assert abs(width - EXPECTED_WIDTH) < 1e-6, width
    print(f'PASS safety bridge (arm_id={arm_id}): /dualarm_teleop_cmd type=4 + gripper width')


def main():
    install_fake_gello()
    rclpy.init()
    try:
        check_direct(direct_bridge.SIM_ARM_IDS)
        check_direct(direct_bridge.REAL_ARM_IDS)
        check_safety()
    finally:
        rclpy.shutdown()
    print('PASS: GELLO bridge smoke test finished')


if __name__ == '__main__':
    main()
