#!/usr/bin/env python3
import numpy as np


class Smoother:
    def __init__(
        self,
        publish_hz: float = 100.0,
        max_velocity: float = 0.3,
        max_acceleration: float = 1.0,
        position_tolerance: float = 0.001,
        smooth_threshold: float = 0.5,
        joint_limits: np.ndarray = None,
        logger=None
    ):
        if joint_limits is None:
            joint_limits = np.array([
                [-2.8973, 2.8973],
                [-1.7628, 1.7628],
                [-2.8973, 2.8973],
                [-3.0718, -0.0698],
                [-2.8973, 2.8973],
                [-0.0175, 3.7525],
                [-2.8973, 2.8973],
            ], dtype=float)

        self.publish_period = 1.0 / publish_hz
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.position_tolerance = position_tolerance
        self.smooth_threshold = smooth_threshold
        self.joint_limits = np.asarray(joint_limits, dtype=float)
        self.logger = logger

        self.q_target_raw = np.zeros(7, dtype=float)
        self.q_target_smooth = np.zeros(7, dtype=float)
        self.q_velocity = np.zeros(7, dtype=float)

        self.s_curve_phase = np.zeros(7, dtype=int)
        self.s_curve_start_pos = np.zeros(7, dtype=float)
        self.s_curve_end_pos = np.zeros(7, dtype=float)
        self.s_curve_total_dist = np.zeros(7, dtype=float)
        self.s_curve_accel_time = np.zeros(7, dtype=float)
        self.s_curve_decel_time = np.zeros(7, dtype=float)
        self.s_curve_const_time = np.zeros(7, dtype=float)
        self.s_curve_elapsed_time = np.zeros(7, dtype=float)

        self.follower_enabled = False
        self.target_received = False
        self.has_started = False
        self.in_full_tracking = False

    def set_initial_position(self, initial_pos: np.ndarray):
        self.q_target_smooth = np.asarray(initial_pos, dtype=float).copy()
        self.q_target_raw = np.asarray(initial_pos, dtype=float).copy()

    def set_follower_enabled(self, enabled: bool):
        was_enabled = self.follower_enabled
        self.follower_enabled = enabled

        if enabled and not was_enabled:
            self.has_started = True
            self._start_smooth()

    def set_target(self, target: np.ndarray):
        self.q_target_raw = np.asarray(target, dtype=float)
        self.target_received = True

    def update(self):
        if not self.target_received:
            return self.q_target_smooth.copy()

        if not self.follower_enabled:
            return self.q_target_smooth.copy()

        if self.in_full_tracking:
            self.q_target_smooth = self.q_target_raw.copy()
            self.q_velocity = np.zeros(7)
            return self.q_target_smooth.copy()

        all_joints_completed = True
        for i in range(7):
            completed = self._update_s_curve_joint(i)
            if not completed:
                all_joints_completed = False

        if all_joints_completed and not self.in_full_tracking:
            self.in_full_tracking = True
            if self.logger:
                self.logger.info('Tracking state: Switched to full tracking mode')

        for i in range(7):
            self.q_target_smooth[i] = np.clip(
                self.q_target_smooth[i],
                self.joint_limits[i, 0],
                self.joint_limits[i, 1]
            )

        return self.q_target_smooth.copy()

    def _start_smooth(self):
        for i in range(7):
            self._start_joint_smooth(i)

    def _start_joint_smooth(self, joint_idx):
        self.s_curve_start_pos[joint_idx] = self.q_target_smooth[joint_idx]
        self.s_curve_end_pos[joint_idx] = self.q_target_raw[joint_idx]
        self.s_curve_total_dist[joint_idx] = abs(self.s_curve_end_pos[joint_idx] - self.s_curve_start_pos[joint_idx])
        self.s_curve_elapsed_time[joint_idx] = 0.0
        self._calculate_s_curve_params(joint_idx)
        self.s_curve_phase[joint_idx] = 0

    def _calculate_s_curve_params(self, joint_idx):
        distance = self.s_curve_total_dist[joint_idx]

        if distance < 0.0001:
            self.s_curve_phase[joint_idx] = 3
            return

        accel_distance = 0.5 * self.max_acceleration * (self.max_velocity / self.max_acceleration) ** 2

        if distance <= 2 * accel_distance:
            self.s_curve_accel_time[joint_idx] = np.sqrt(distance / self.max_acceleration)
            self.s_curve_decel_time[joint_idx] = self.s_curve_accel_time[joint_idx]
            self.s_curve_const_time[joint_idx] = 0.0
        else:
            self.s_curve_accel_time[joint_idx] = self.max_velocity / self.max_acceleration
            self.s_curve_decel_time[joint_idx] = self.s_curve_accel_time[joint_idx]
            remaining_distance = distance - 2 * accel_distance
            self.s_curve_const_time[joint_idx] = remaining_distance / self.max_velocity

    def _update_s_curve_joint(self, joint_idx):
        if self.s_curve_phase[joint_idx] == 3:
            return True

        self.s_curve_elapsed_time[joint_idx] += self.publish_period
        elapsed = self.s_curve_elapsed_time[joint_idx]
        start_pos = self.s_curve_start_pos[joint_idx]
        end_pos = self.s_curve_end_pos[joint_idx]
        accel_time = self.s_curve_accel_time[joint_idx]
        const_time = self.s_curve_const_time[joint_idx]
        decel_time = self.s_curve_decel_time[joint_idx]

        direction = 1.0 if end_pos > start_pos else -1.0

        if elapsed < accel_time:
            t = elapsed / accel_time
            normalized_vel = np.sin(np.pi / 2 * t)
            vel = direction * self.max_velocity * normalized_vel
            pos = start_pos + direction * self.max_velocity * accel_time * (1 - np.cos(np.pi / 2 * t)) / (np.pi / 2)
            self.s_curve_phase[joint_idx] = 0

        elif elapsed < accel_time + const_time:
            t = elapsed - accel_time
            vel = direction * self.max_velocity
            accel_dist = direction * self.max_velocity * accel_time * (2 / np.pi)
            pos = start_pos + accel_dist + vel * t
            self.s_curve_phase[joint_idx] = 1

        elif elapsed < accel_time + const_time + decel_time:
            t = (elapsed - accel_time - const_time) / decel_time
            normalized_vel = np.cos(np.pi / 2 * t)
            vel = direction * self.max_velocity * normalized_vel
            accel_dist = direction * self.max_velocity * accel_time * (2 / np.pi)
            const_dist = direction * self.max_velocity * const_time
            decel_dist = direction * self.max_velocity * decel_time * (np.sin(np.pi / 2 * t) / (np.pi / 2))
            pos = start_pos + accel_dist + const_dist + decel_dist
            self.s_curve_phase[joint_idx] = 2

        else:
            pos = end_pos
            vel = 0.0
            self.s_curve_phase[joint_idx] = 3

        self.q_target_smooth[joint_idx] = pos
        self.q_velocity[joint_idx] = vel

        return self.s_curve_phase[joint_idx] == 3