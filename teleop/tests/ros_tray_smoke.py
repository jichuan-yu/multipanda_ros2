"""Isolated ROS protocol/state-machine smoke test; no physics or real actuators.

Run explicitly with ROS_DOMAIN_ID=219 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST.
"""
import os
if os.environ.get('ROS_DOMAIN_ID') != '219' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
    raise RuntimeError('Use isolated test domain 219 and localhost discovery')

import argparse
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tray_transport_plan import home_poses, HOME_POSITION, build_plan


class Fixture(Node):
    def __init__(self, reject_close=False, idle_no_ee=False, grasp_stop_offset=0., competitor=None, detour_release_distance=0., carry_endpoint_offsets=False):
        super().__init__('dual_arm_mprc_node')
        self.declare_parameter('relative_pose_constraint_enabled', False)
        self.add_on_set_parameters_callback(self.switch)
        self.cooperative = False
        self.reject_close = reject_close
        self.idle_no_ee = idle_no_ee
        self.command_count = 0
        self.carry_endpoint_offsets = carry_endpoint_offsets
        self.placement_started = False
        self.nominal_transport = next(s.target for s in build_plan().stages if s.name == "transport")
        self.detour_release_distance = detour_release_distance
        self.detour_released = not bool(detour_release_distance)
        self.max_reference_lead = 0.
        self.competitor = competitor
        self.competitor_sent = False
        self.foreign_pub = (self.create_publisher(Float64MultiArray, '/dualarm_teleop_cmd', 10)
                            if competitor else None)
        self.grasp_stop_offset = grasp_stop_offset
        self.nominal_grasp = next(s.target for s in build_plan().stages if s.name == 'close')
        self.closed_poses = {}
        self.events = []
        self.pose = home_poses()
        self.widths = {s: .08 for s in ['left', 'right']}
        self.forces = {s: 0. for s in ['left', 'right']}
        self.clock = self.create_publisher(Clock, '/clock', 10)
        self.ee = self.create_publisher(Float64MultiArray, '/ee_pose', 10)
        self.js = self.create_publisher(JointState, '/joint_states', 10)
        self.gs, self.status = {}, {}
        self.subs = [self.create_subscription(Float64MultiArray, '/dualarm_teleop_cmd', self.cmd, 10)]
        for side in ['left', 'right']:
            self.gs[side] = self.create_publisher(JointState, f'/mj_{side}_gripper_sim_node/joint_states', 10)
            self.status[side] = self.create_publisher(Float64MultiArray, f'/mj_{side}_gripper/grasp_status', 10)
            self.subs.append(self.create_subscription(Float64MultiArray,
                f'/mj_{side}_gripper/grasp_desired', lambda msg, s=side: self.grip(s, msg), 10))
        self.t = 1.
        self.timer = self.create_timer(.002, self.tick)

    def switch(self, params):
        for p in params:
            if p.name == 'relative_pose_constraint_enabled':
                self.cooperative = p.value
                self.events.append(('cooperation', p.value))
        return SetParametersResult(successful=True)

    def cmd(self, msg):
        assert len(msg.data) == 17 and list(msg.data[-3:]) == [5., 3., 0.]
        self.command_count += 1
        commanded = np.array(msg.data[:14]).reshape(2, 7)
        lead = float(max(np.linalg.norm(commanded[:, :3]-self.pose[:, :3], axis=1)))
        self.max_reference_lead = max(self.max_reference_lead, lead)
        if not self.detour_released:
            if lead < self.detour_release_distance:
                return  # Actual robot is constrained until the reference moves past it.
            self.detour_released = True
        self.pose = commanded.copy()
        if self.carry_endpoint_offsets and self.cooperative and min(self.widths.values()) < .02:
            plateau = self.nominal_transport[:, 2]+self.grasp_stop_offset-.012
            at_destination = np.max(np.abs(commanded[:, 1]-self.nominal_transport[:, 1])) < .03
            if at_destination and np.all(commanded[:, 2] < plateau-.002):
                self.placement_started = True
            if not self.placement_started:
                self.pose[:, 2] = np.minimum(self.pose[:, 2], plateau)
                self.pose[:, 1] = np.maximum(self.pose[:, 1], self.nominal_transport[:, 1]+.015)
        if self.grasp_stop_offset and not any(e[1] == 'close' for e in self.events):
            self.pose[:, 2] = np.maximum(self.pose[:, 2],
                                         self.nominal_grasp[:, 2]+self.grasp_stop_offset)

    def grip(self, side, msg):
        close = msg.data[0] == 0
        self.events.append((side, 'close' if close else 'open', self.cooperative))
        if close:
            self.closed_poses[side] = self.pose.copy()
        if close and self.reject_close:
            self.status[side].publish(Float64MultiArray(data=[msg.data[5], -2.]))
            return
        self.widths[side] = .01 if close else .08
        self.forces[side] = 30. if close else 0.
        self.status[side].publish(Float64MultiArray(data=[msg.data[5], 2.]))

    def tick(self):
        if self.competitor == 'active' or (self.competitor == 'late' and
                                          self.command_count > 4 and not self.competitor_sent):
            self.foreign_pub.publish(Float64MultiArray(data=self.pose.ravel().tolist()+[5., 3., 0.]))
            self.competitor_sent = True
        self.t += .01
        c = Clock(); c.clock.sec = int(self.t); c.clock.nanosec = int((self.t-int(self.t))*1e9)
        self.clock.publish(c)
        if not self.idle_no_ee or self.command_count:
            self.ee.publish(Float64MultiArray(data=self.pose.ravel().tolist()))
        msg = JointState()
        msg.name = [f'mj_{s}_joint{i}' for s in ['left', 'right'] for i in range(1, 8)]
        msg.position = list(HOME_POSITION)*2
        self.js.publish(msg)
        for side in ['left', 'right']:
            g = JointState(); g.name = [f'mj_{side}_finger_joint{i}' for i in [1, 2]]
            g.position = [self.widths[side]/2]*2; g.effort = [self.forces[side]/2]*2
            self.gs[side].publish(g)


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--reject-close', action='store_true')
    parser.add_argument('--idle-no-ee', action='store_true')
    parser.add_argument('--check-feedback', action='store_true')
    parser.add_argument('--grasp-stop-offset', type=float, default=0.)
    parser.add_argument('--competitor', choices=['idle', 'active', 'late'])
    parser.add_argument('--detour-release-distance', type=float, default=0.)
    parser.add_argument('--carry-endpoint-offsets', action='store_true')
    args = parser.parse_args()
    rclpy.init(); node = Fixture(args.reject_close, args.idle_no_ee, args.grasp_stop_offset, args.competitor, args.detour_release_distance, args.carry_endpoint_offsets)
    path = Path(__file__).resolve().parents[1]/'tray_transport_input.py'
    log_path = Path('/tmp/tray_ros_reject.log' if args.reject_close else '/tmp/tray_ros_smoke.log')
    with log_path.open('w') as log:
        process = subprocess.Popen([sys.executable, str(path),
                                    '--check-feedback' if args.check_feedback else '--execute',
                                    '--gripper-timeout', '5', '--stage-timeout', '3'],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+100
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.002)
            assert process.poll() is not None, f'Test timeout; see {log_path}'
            if args.competitor in ['active', 'late']:
                assert process.returncode != 0, log_path.read_text()
                assert 'External command received' in log_path.read_text(), log_path.read_text()
                assert 'candidate publishers:' in log_path.read_text()
                assert not any(e[1] in ['open', 'close'] for e in node.events)
                if args.competitor == 'active':
                    assert not node.events  # No parameter/gripper commands before startup rejection.
            elif args.check_feedback:
                assert process.returncode == 0, log_path.read_text()
                assert node.command_count == 0 and not node.events
                if args.idle_no_ee:
                    assert 'pose source: /joint_states FK' in log_path.read_text()
            elif args.reject_close:
                assert process.returncode != 0
                assert ('cooperation', True) not in node.events
                assert sum(e[1] == 'open' for e in node.events) == 2
            else:
                assert process.returncode == 0, log_path.read_text()
                assert [e[1] for e in node.events if e[0] == 'cooperation'] == [False, True, False]
                for side in ['left', 'right']:
                    assert [e[1:] for e in node.events if e[0] == side] == [
                        ('open', False), ('close', False), ('open', True)]
                assert not node.cooperative
                if args.carry_endpoint_offsets:
                    assert node.placement_started
                    assert "lift: transit gate accepted" in log_path.read_text()
                    assert "transport: transit gate accepted" in log_path.read_text()
            if args.detour_release_distance:
                assert node.detour_released
                assert node.max_reference_lead >= args.detour_release_distance
            if args.grasp_stop_offset and node.closed_poses:
                for pose in node.closed_poses.values():
                    np.testing.assert_allclose(pose[:, 2]-node.nominal_grasp[:, 2],
                                               args.grasp_stop_offset, atol=.0002)
                assert 'Grasp region accepted' in log_path.read_text()
                if process.returncode == 0 and not args.check_feedback:
                    expected = next(s.target for s in build_plan().stages if s.name == 'retreat')
                    np.testing.assert_allclose(node.pose[:, 2],
                                               expected[:, 2]+args.grasp_stop_offset, atol=.003)
            print('PASS:', f'external command {args.competitor} rejected' if args.competitor in ['active', 'late'] else
                  'read-only feedback preflight' if args.check_feedback else
                  'failed grasp stops progression' if args.reject_close else 'complete task protocol', node.events)
        finally:
            if process.poll() is None:
                process.terminate(); process.wait(timeout=5)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
