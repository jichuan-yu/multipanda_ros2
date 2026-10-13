#!/usr/bin/env python3
"""Tray task reference input; same safety-controller interface as keyboard teleop.

Default is offline preview. --execute enables ROS reference publishing.
"""
import argparse
import json
import sys
import threading
import time
import uuid
from pathlib import Path

import numpy as np
from tray_transport_plan import (
    DEFAULT_SCENE, DEFAULT_MODEL, build_plan, home_poses, validate_reachability,
    HOME_POSITION, PandaDHKinematics, SegmentPlayer, errors, GraspArrival, grasp_region_status, transit_region_status,
)


def plan_summary(plan):
    return dict(plan.metadata, walls=plan.walls, open_width=plan.open_width,
                stages=[dict(name=s.name, poses_xyzw=s.target.tolist(), speed=s.speed,
                             action=s.action, cooperative=s.cooperative) for s in plan.stages])


def execute(plan, args):
    # Offline geometry/preflight needs neither ROS nor a running simulation.
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray, Bool, MultiArrayDimension
    from sensor_msgs.msg import JointState
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from base_teleop import BaseTeleopNode
    from key_teleop_cartesian_absolute import KeyCartesianAbsoluteTeleop

    class TrayInput(BaseTeleopNode):
        def __init__(self):
            super().__init__('tray_transport_input_' + uuid.uuid4().hex[:8], publish_rate=100.)
            self.controller_node = args.controller_node
            self.lock = threading.RLock()
            self.actual = None
            self.joints = None
            self.ee_stamp = self.joint_stamp = self.pose_stamp = 0.
            self.pose_source = 'none'
            self.kinematics = [PandaDHKinematics(side) for side in ['left', 'right']]
            self.target = None
            self.grippers = {}
            self.results = {}
            self.request_id = int(time.time()*1000)
            self.request_active = False
            self.carrying = False
            self.fault = None
            self.cooperative = False
            self.grip_loss_since = {}
            self.command_token = 'tray_input:' + uuid.uuid4().hex
            self.subs = [self.create_subscription(Float64MultiArray, '/dualarm_teleop_cmd',
                                                  self.command_observed, qos_profile_sensor_data),
                         self.create_subscription(Float64MultiArray, '/ee_pose', self.ee_cb, 10),
                         self.create_subscription(JointState, '/joint_states', self.joints_cb,
                                                  qos_profile_sensor_data)]
            self.grip_pubs, self.stop_pubs = {}, {}
            for side in ['left', 'right']:
                self.grip_pubs[side] = self.create_publisher(
                    Float64MultiArray, f'/mj_{side}_gripper/grasp_desired', 10)
                self.stop_pubs[side] = self.create_publisher(
                    Bool, f'/mj_{side}_gripper/stop_desired', 10)
                self.subs.append(self.create_subscription(
                    Float64MultiArray, f'/mj_{side}_gripper/grasp_status',
                    lambda msg, s=side: self.status_cb(s, msg), 10))
                self.subs.append(self.create_subscription(
                    JointState, f'/mj_{side}_gripper_sim_node/joint_states',
                    lambda msg, s=side: self.gripper_cb(s, msg), qos_profile_sensor_data))
            # Wall timer republishes the held reference during simulation pauses.
            self.timer = self.create_timer(.01, self.publish_target,
                                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        def command_observed(self, msg):
            # Humble's rclpy callbacks do not expose publisher GIDs. Tag only
            # our layout metadata; the controller's 17 data values stay intact.
            # Legacy keyboard/CSV messages have no tag and are external commands.
            if len(msg.layout.dim) == 1 and msg.layout.dim[0].label == self.command_token:
                return
            with self.lock:
                endpoints = [e for e in self.get_publishers_info_by_topic('/dualarm_teleop_cmd')
                             if e.node_name != self.get_name() or e.node_namespace != self.get_namespace()]
                candidates = '; '.join(
                    f'{e.node_namespace.rstrip("/")}/{e.node_name} (GID={bytes(e.endpoint_gid).hex()})'
                    for e in endpoints) or 'unknown publisher'
                self.fault = ('External command received on /dualarm_teleop_cmd; '
                              'candidate publishers: ' + candidates +
                              '; stopped this input to avoid competing commands')

        def report_command_endpoints(self):
            endpoints = [e for e in self.get_publishers_info_by_topic('/dualarm_teleop_cmd')
                         if e.node_name != self.get_name() or e.node_namespace != self.get_namespace()]
            if endpoints:
                labels = [f'{e.node_namespace.rstrip("/")}/{e.node_name} '
                          f'(GID={bytes(e.endpoint_gid).hex()})' for e in endpoints]
                self.get_logger().warning('Other command endpoints exist: ' + '; '.join(labels) +
                    '. Endpoint presence alone does not block startup; actual external commands will stop this task.')

        def ee_cb(self, msg):
            if len(msg.data) != 14:
                return
            a = np.asarray(msg.data).reshape(2, 7)
            lengths = np.linalg.norm(a[:, 3:], axis=1)
            if not np.isfinite(a).all() or np.any(lengths < .5):
                return
            a[:, 3:] /= lengths[:, None]
            with self.lock:
                self.actual, self.ee_stamp = a, time.monotonic()
                self.pose_stamp = self.ee_stamp
                self.pose_source = '/ee_pose'

        def joints_cb(self, msg):
            values = dict(zip(msg.name, msg.position))
            names = [[f'mj_{side}_joint{i}' for i in range(1, 8)] for side in ['left', 'right']]
            if all(n in values for row in names for n in row):
                a = np.array([[values[n] for n in row] for row in names])
                if np.isfinite(a).all():
                    with self.lock:
                        self.joints, self.joint_stamp = a, time.monotonic()
                        # STOPPING/idle publishes joint states but may publish
                        # /ee_pose only once at controller startup. Never send
                        # a motion command just to obtain pose feedback.
                        if self.joint_stamp-self.ee_stamp > .5:
                            poses = []
                            for model, q in zip(self.kinematics, a):
                                position, quat = model.forward_position_quat(q)
                                poses.append(np.r_[position, np.roll(quat, -1)])
                            self.actual = np.array(poses)
                            self.pose_stamp = self.joint_stamp
                            self.pose_source = '/joint_states FK'

        def gripper_cb(self, side, msg):
            names = [f'mj_{side}_finger_joint{i}' for i in [1, 2]]
            if not all(n in msg.name for n in names):
                return
            indices = [msg.name.index(n) for n in names]
            if any(i >= len(msg.position) or i >= len(msg.effort) for i in indices):
                return
            width = sum(msg.position[i] for i in indices)
            force = abs(sum(msg.effort[i] for i in indices))
            if np.isfinite([width, force]).all():
                with self.lock:
                    self.grippers[side] = (width, force, time.monotonic())

        def status_cb(self, side, msg):
            if len(msg.data) == 2:
                with self.lock:
                    if msg.data[0] == self.request_id:
                        self.results[side] = int(msg.data[1])

        def publish_target(self):
            with self.lock:
                if (self.target is None or self.fault
                        or time.monotonic()-self.pose_stamp > .5):
                    return
                msg = self.create_teleop_command(
                    arm_selector=3, command_type=5,
                    left_ee_delta=self.target[0], right_ee_delta=self.target[1],
                    allow_safety_violation=False)
                msg.layout.dim = [MultiArrayDimension(
                    label=self.command_token, size=len(msg.data), stride=len(msg.data))]
                self.teleop_pub.publish(msg)

        def snapshot(self):
            with self.lock:
                if self.fault:
                    raise RuntimeError(self.fault)
                if self.actual is None or time.monotonic()-self.pose_stamp > .5:
                    raise RuntimeError('Neither /ee_pose nor /joint_states FK is fresh; reference publishing stopped')
                return self.actual.copy()

        def check_health(self):
            if not rclpy.ok():
                raise RuntimeError('ROS shutdown')
            self.snapshot()
            if self.carrying:
                for side in ['left', 'right']:
                    state = self.grippers.get(side)
                    if state is None or time.monotonic()-state[2] > .5:
                        raise RuntimeError(f'{side}: gripper feedback stale during carry')
                    if state[0] > .035 or state[0] < .001:
                        raise RuntimeError(f'{side}: unexpected gripper width during carry')
                    if state[1] < 2.:
                        since = self.grip_loss_since.setdefault(side, time.monotonic())
                        if time.monotonic()-since > .5:
                            raise RuntimeError(f'{side}: sustained loss of gripping force')
                    else:
                        self.grip_loss_since.pop(side, None)

        def wait_sim(self, seconds, timeout=15.):
            start = self.get_clock().now().nanoseconds/1e9
            wall = time.monotonic()
            while self.get_clock().now().nanoseconds/1e9-start < seconds:
                self.check_health()
                if time.monotonic()-wall > timeout:
                    raise RuntimeError('Simulation clock did not progress within timeout')
                time.sleep(.01)

        def switch(self, enabled):
            self.check_health()
            client = self.create_client(SetParameters, self.controller_node+'/set_parameters')
            try:
                request = SetParameters.Request()
                request.parameters = [Parameter(name='relative_pose_constraint_enabled',
                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled))]
                response = KeyCartesianAbsoluteTeleop._wait_parameter_response(client, request)
                if len(response.results) != 1 or not response.results[0].successful:
                    raise RuntimeError('Controller rejected cooperative-constraint switch')
                self.cooperative = enabled
                # Controller captures the current relative pose on its next update.
                self.wait_sim(plan.metadata['configuration']['cooperation_settle_time'])
            finally:
                self.destroy_client(client)

        def grip(self, close):
            self.check_health()
            if close and not grasp_region_status(plan, self.snapshot())[0]:
                raise RuntimeError('Hands left the feasible grasp region before closing')
            self.request_id += 1
            self.results = {}
            self.request_active = True
            width = 0. if close else plan.open_width
            for publisher in self.grip_pubs.values():
                publisher.publish(Float64MultiArray(data=[width, .05, 30., .005, .005,
                                                         float(self.request_id)]))
            deadline = time.monotonic()+args.gripper_timeout
            stable = None
            # Action success alone does not prove contact: the simulation action
            # also returns success when it closes on empty space.
            while time.monotonic() < deadline:
                self.check_health()
                if any(result < 0 for result in self.results.values()):
                    raise RuntimeError(f'Gripper action failed: {self.results}')
                valid = all(self.results.get(side) == 2 for side in ['left', 'right'])
                for index, side in enumerate(['left', 'right']):
                    state = self.grippers.get(side)
                    valid &= state is not None and time.monotonic()-state[2] < .5
                    if state is not None:
                        w, force, _ = state
                        valid &= (.001 < w < plan.wall_widths[index]+.015 and force >= 3.
                                  if close else w >= plan.open_width-.004)
                now = self.get_clock().now().nanoseconds/1e9
                stable = (now if stable is None else stable) if valid else None
                if stable is not None and now-stable >= plan.metadata['configuration']['gripper_settle_time']:
                    self.request_active = False
                    return
                time.sleep(.01)
            raise RuntimeError('Gripper completion/contact timeout; no automatic advance')

        def move(self, stage):
            player = SegmentPlayer(self.target, stage.target, stage.speed,
                min_playback_scale=float(plan.metadata['configuration'].get('min_playback_scale', .2)),
                ramp_time=plan.metadata['configuration']['motion_ramp_time'])
            grasp_arrival = GraspArrival(plan) if stage.name == 'descend_to_grasp' else None
            transit = stage.name in ('lift', 'transport')
            previous = self.get_clock().now().nanoseconds/1e9
            endpoint_wait_since = None
            stable = None
            report = 0.
            while rclpy.ok():
                self.check_health()
                now = self.get_clock().now().nanoseconds/1e9
                dt, previous = now-previous, now
                if dt < 0 or dt > .5:
                    raise RuntimeError('Simulation clock jumped; restart from actual state')
                actual = self.snapshot()
                with self.lock:
                    self.target = player.advance(actual, min(.05, dt))
                pe, re = errors(actual, stage.target)
                if grasp_arrival is not None and grasp_arrival.update(actual, now):
                    # Hold the achievable pose while closing; never continue
                    # requesting the lower nominal point after accepting it.
                    with self.lock:
                        self.target = actual.copy()
                    _, overlap = grasp_region_status(plan, actual)
                    self.get_logger().info(
                        'Grasp region accepted: height above nominal=' +
                        str(np.round(actual[:, 2]-stage.target[:, 2], 4).tolist()) +
                        'm, pad/wall overlap=' + str(np.round(overlap, 4).tolist()) + 'm')
                    return
                transit_detail = ''
                if transit:
                    actual_ready, transit_detail = transit_region_status(plan, stage, actual)
                    reference_ready, _ = transit_region_status(plan, stage, self.target)
                    arrived = actual_ready and reference_ready
                else:
                    arrived = grasp_arrival is None and player.done and pe < .003 and re < .025
                stable = (now if stable is None else stable) if arrived else None
                if stable is not None and now-stable >= plan.metadata['configuration']['arrival_settle_time']:
                    if transit:
                        self.get_logger().info(f'{stage.name}: transit gate accepted; {transit_detail}')
                    return
                # Never exhaust an error-dependent wall deadline before the
                # reference has played to its end. Allow extra time for the
                # real arms to reach the endpoint, measured in simulation time.
                if player.done:
                    if endpoint_wait_since is None:
                        endpoint_wait_since = now
                    elif now-endpoint_wait_since >= args.stage_timeout:
                        raise RuntimeError(f'{stage.name}: reference completed, but actual endpoint '
                                           f'not reached within {args.stage_timeout:g}s of simulation time; '
                                           f'error={pe:.4f}m/{re:.3f}rad; {transit_detail}')
                if time.monotonic()-report > 2:
                    grasp_detail = ''
                    if grasp_arrival is not None:
                        feasible, overlaps = grasp_region_status(plan, actual)
                        grasp_detail = (f', grasp_region={feasible}, overlap='
                                        f'{np.round(overlaps, 4).tolist()}m')
                    tracking_pe, tracking_re = errors(actual, self.target)
                    self.get_logger().info(f'{stage.name}: progress={player.elapsed/player.duration if player.duration else 1.:.0%}, '
                        f'scale={player.scale:.2f}, tracking_error={tracking_pe:.4f}m/{tracking_re:.3f}rad, '
                        f'endpoint_error={pe:.4f}m/{re:.3f}rad{grasp_detail}'
                        + (f', transit_gate: {transit_detail}' if transit else ''))
                    report = time.monotonic()
                time.sleep(.01)
            raise RuntimeError('ROS shutdown during reference playback')

        def missing_feedback(self):
            with self.lock:
                now = time.monotonic()
                missing = []
                if self.actual is None or now-self.pose_stamp > .5:
                    missing.append('/ee_pose or /joint_states FK (missing/stale)')
                if self.joints is None or now-self.joint_stamp > .5:
                    missing.append('/joint_states (14 named arm joints, missing/stale)')
                for side in ['left', 'right']:
                    state = self.grippers.get(side)
                    if state is None or now-state[2] > .5:
                        missing.append(f'/mj_{side}_gripper_sim_node/joint_states '
                                       '(two named fingers with position/effort, missing/stale)')
                    if self.count_publishers(f'/mj_{side}_gripper/grasp_status') == 0:
                        missing.append(f'/mj_{side}_gripper/grasp_status publisher '
                                       '(bridge not discovered; ensure updated bridge is running)')
                    if self.grip_pubs[side].get_subscription_count() == 0:
                        missing.append(f'/mj_{side}_gripper/grasp_desired subscriber '
                                       '(bridge not discovered)')
                return missing

        def run(self):
            nonlocal plan
            deadline = time.monotonic()+15.
            while time.monotonic() < deadline:
                if self.fault:
                    raise RuntimeError(self.fault)
                if not self.missing_feedback():
                    break
                time.sleep(.05)
            else:
                raise RuntimeError('Startup dependencies not ready after 15s: ' + '; '.join(self.missing_feedback()))
            self.get_logger().info(f'Feedback ready; pose source: {self.pose_source}')
            self.check_health()
            self.controller_node = KeyCartesianAbsoluteTeleop._find_cooperative_controller(self)
            start, joints = self.snapshot(), self.joints.copy()
            plan = build_plan(args.scene, args.model, start=start)
            fk = []
            for side, q in zip(['left', 'right'], joints):
                p, quat = PandaDHKinematics(side).forward_position_quat(q)
                fk.append(np.r_[p, np.roll(quat, -1)])
            pe, re = errors(start, np.array(fk))
            if pe > .01 or re > .05:
                raise RuntimeError('Feedback frame differs from Panda DH/model; refuse preflight')
            self.get_logger().info('Checking all reference samples from the actual initial state...')
            collision_check = None
            if args.check_collisions:
                from tray_transport_collision import ReferenceCollisionCheck
                collision_check = ReferenceCollisionCheck(
                    plan, args.scene, args.model,
                    [self.grippers[s][0] for s in ['left', 'right']])
            validate_reachability(plan, start, joints, collision_check=collision_check)
            if max(errors(self.snapshot(), start)) > .01:
                raise RuntimeError('Robot moved during preflight; restart')
            self.check_health()
            self.report_command_endpoints()
            if args.check_feedback:
                self.get_logger().info('Live feedback and reference preflight passed; '
                                       'no motion, gripper, or parameter commands sent')
                return
            self.target = start.copy()
            self.switch(False)
            for stage in plan.stages:
                self.get_logger().info(f'Stage: {stage.name}')
                if not stage.action:
                    self.move(stage)
                elif stage.action == 'open':
                    # Keep cooperative constraint through release completion.
                    was_carrying = self.carrying
                    self.carrying = False
                    try:
                        self.grip(False)
                    except Exception:
                        self.carrying = was_carrying
                        raise
                elif stage.action == 'close':
                    self.grip(True)
                    self.carrying = True
                elif stage.action == 'enable':
                    # Match the relative transform the controller captures,
                    # rather than fighting it with two independently rounded
                    # nominal endpoints. Apply one translation to both hands.
                    captured = self.snapshot()
                    nominal = stage.target.copy()
                    with self.lock:
                        self.target = captured.copy()
                    for following in plan.stages[next(i for i, s in enumerate(plan.stages) if s.name == stage.name)+1:]:
                        delta = following.target[:, :3]-nominal[:, :3]
                        following.target = captured.copy()
                        following.target[:, :3] += delta
                    self.switch(True)
                elif stage.action == 'disable':
                    self.switch(False)
            self.get_logger().info('Tray task completed; cooperation disabled, arms at retreat pose')

    rclpy.init(args=[])
    node = TrayInput()
    def spin_node():
        try:
            rclpy.spin(node)
        except rclpy.executors.ExternalShutdownException:
            pass
        except Exception as exc:
            node.fault = f'ROS executor failed: {exc}'

    thread = threading.Thread(target=spin_node, daemon=True)
    thread.start()
    try:
        node.run()
    except (Exception, KeyboardInterrupt):
        with node.lock:
            node.fault = 'Task stopped'
        if node.request_active:
            for publisher in node.stop_pubs.values():
                publisher.publish(Bool(data=True))
            time.sleep(.1)
        node.get_logger().error('Stopped reference input. No release or constraint reset was sent; '
                                'safety controller retains motion authority.')
        raise
    finally:
        node.timer.cancel()
        rclpy.shutdown()
        thread.join(timeout=2.)
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, default=DEFAULT_SCENE)
    parser.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--check-collisions', action='store_true',
                        help='Check sampled nominal IK mesh contacts with offline MuJoCo')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='Publish to the running safety controller')
    mode.add_argument('--check-feedback', action='store_true',
                      help='Read live feedback and preflight only; never issue commands')
    parser.add_argument('--output', type=Path, help='Write derived task waypoints to JSON')
    parser.add_argument('--controller-node', default=None)
    parser.add_argument('--stage-timeout', type=float, default=120.,
                        help='Simulation seconds to wait for actual arrival after reference finishes')
    parser.add_argument('--gripper-timeout', type=float, default=15.)
    args = parser.parse_args()
    if args.stage_timeout <= 0 or args.gripper_timeout <= 0:
        parser.error('Timeouts must be positive')
    plan = build_plan(args.scene, args.model)
    if not (args.execute or args.check_feedback):
        collision_check = None
        if args.check_collisions:
            from tray_transport_collision import ReferenceCollisionCheck
            collision_check = ReferenceCollisionCheck(plan, args.scene, args.model)
        count = validate_reachability(plan, home_poses(), np.tile(HOME_POSITION, (2, 1)),
                                      collision_check=collision_check)
        print(f'Offline IK preflight passed: {count} samples; '
              f'mesh collision check={args.check_collisions}. No ROS commands sent.', file=sys.stderr)
    text = json.dumps(plan_summary(plan), ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(text+'\n')
    if args.execute or args.check_feedback:
        execute(plan, args)
    else:
        print(text)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError) as exc:
        print(f'Tray input stopped: {exc}', file=sys.stderr)
        sys.exit(1)
