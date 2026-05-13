#!/usr/bin/env python3
from dataclasses import dataclass, field
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class GelloSmootherConfig:
    arm_id: str = 'panda'
    source_joint_topic: str = '/gello/joints_desired'
    target_joint_topic: str = '/joint_impedance/joints_desired'
    joint_state_topic: str = '/panda/joint_states'
    publish_hz: float = 100.0
    smooth_threshold: float = 0.5  
    full_tracking_threshold: float = 0.05  
    max_velocity: float = 0.3  
    max_acceleration: float = 1.0  
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

    def __post_init__(self):
        if self.joint_limits.shape != (7, 2):
            raise ValueError('joint_limits must be shape (7, 2)')


class JointSmoothTrajectory:
    def __init__(self, start_pos, end_pos, max_vel, max_acc):
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.distance = abs(end_pos - start_pos)
        
        if self.distance < 0.0001:
            self._is_finished = True
            self.duration = 0.0
            return
        
        self._is_finished = False
        self.calculate_trajectory()
    
    def calculate_trajectory(self):
        d = self.distance
        v_max = self.max_vel
        a_max = self.max_acc
        
        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc ** 2
        
        if 2 * d_acc >= d:
            t_acc = np.sqrt(d / a_max)
            self.t_jerk = t_acc
            self.t_accel = t_acc
            self.t_const = 0.0
            self.t_decel = t_acc
            self.duration = 2 * t_acc
            self.v_peak = a_max * t_acc
        else:
            self.t_jerk = t_acc
            self.t_accel = t_acc
            self.t_const = (d - 2 * d_acc) / v_max
            self.t_decel = t_acc
            self.duration = 2 * t_acc + self.t_const
            self.v_peak = v_max
        
        self.sign = 1.0 if (self.end_pos - self.start_pos) >= 0 else -1.0
    
    def get_position(self, t):
        if self._is_finished or self.distance < 0.0001:
            return self.end_pos
        
        t = max(0.0, min(t, self.duration))
        
        if t <= self.t_jerk:
            s = self.s_curve_accel(t)
        elif t <= self.t_jerk + self.t_accel:
            s = self.const_accel(t - self.t_jerk)
        elif t <= self.t_jerk + self.t_accel + self.t_const:
            s = self.const_vel(t - self.t_jerk - self.t_accel)
        elif t <= self.duration - self.t_jerk:
            s = self.const_decel(t - self.duration + self.t_jerk + self.t_decel)
        else:
            s = self.s_curve_decel(t - self.duration + self.t_jerk)
        
        return self.start_pos + self.sign * s
    
    def s_curve_accel(self, t):
        T = self.t_jerk
        a_max = self.max_acc
        return (a_max / (6 * T ** 3)) * t ** 4 * (3 * T - t)
    
    def const_accel(self, t):
        v_peak = self.v_peak
        T_jerk = self.t_jerk
        a_max = self.max_acc
        s_jerk = (a_max / 24) * T_jerk ** 2
        v_jerk = (a_max / 6) * T_jerk ** 2 / T_jerk
        return s_jerk + v_jerk * t + 0.5 * a_max * t ** 2
    
    def const_vel(self, t):
        v_peak = self.v_peak
        T_jerk = self.t_jerk
        T_acc = self.t_accel
        a_max = self.max_acc
        s_jerk = (a_max / 24) * T_jerk ** 2
        v_jerk = (a_max / 6) * T_jerk
        s_acc = s_jerk + v_jerk * T_acc + 0.5 * a_max * T_acc ** 2
        v_acc = v_jerk + a_max * T_acc
        return s_acc + v_acc * t
    
    def const_decel(self, t):
        d = self.distance
        v_peak = self.v_peak
        T_jerk = self.t_jerk
        a_max = self.max_acc
        s_jerk = (a_max / 24) * T_jerk ** 2
        v_jerk = (a_max / 6) * T_jerk
        s_acc = s_jerk + v_jerk * self.t_accel + 0.5 * a_max * self.t_accel ** 2
        s_const = s_acc + v_peak * self.t_const
        v_current = v_peak - a_max * t
        return s_const + v_peak * t - 0.5 * a_max * t ** 2
    
    def s_curve_decel(self, t):
        d = self.distance
        T = self.t_jerk
        a_max = self.max_acc
        return d - (a_max / (6 * T ** 3)) * t ** 4 * (3 * T - t)
    
    @property
    def is_finished(self):
        return self._is_finished


class GelloSmootherNode(Node):
    def __init__(self, config: GelloSmootherConfig = None):
        super().__init__('gello_smoother')

        self.config = config if config is not None else GelloSmootherConfig()
        self.arm_id = self.config.arm_id
        self.smooth_threshold = float(self.config.smooth_threshold)
        self.full_tracking_threshold = float(self.config.full_tracking_threshold)
        self.max_velocity = float(self.config.max_velocity)
        self.max_acceleration = float(self.config.max_acceleration)
        self.publish_period = 1.0 / float(self.config.publish_hz)
        self.joint_limits = np.asarray(self.config.joint_limits, dtype=float)

        self.arm_joint_names = [f'{self.arm_id}_joint{i + 1}' for i in range(7)]
        
        self.q_current = None
        self.q_target_raw = None
        self.q_target_smooth = None
        
        self.joint_states_received = False
        self.raw_target_received = False
        self.in_full_tracking = False
        self.trajectories = None
        self.trajectory_start_time = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.config.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.raw_target_sub = self.create_subscription(
            JointState,
            self.config.source_joint_topic,
            self.raw_target_callback,
            10,
        )

        self.smooth_target_pub = self.create_publisher(
            JointState,
            self.config.target_joint_topic,
            10
        )

        self.timer = self.create_timer(self.publish_period, self.timer_callback)

        self.get_logger().info('Gello Smoother Node initialized.')
        self.get_logger().info(f'Smooth threshold: {self.smooth_threshold:.3f} rad')
        self.get_logger().info(f'Full tracking threshold: {self.full_tracking_threshold:.3f} rad')
        self.get_logger().info(f'Max velocity: {self.max_velocity:.3f} rad/s')

    def joint_state_callback(self, msg: JointState):
        try:
            indices = [msg.name.index(name) if name in msg.name else None 
                       for name in self.arm_joint_names]
            if not all(idx is not None for idx in indices):
                return

            q_arm = np.array([msg.position[idx] for idx in indices], dtype=float)
            self.q_current = q_arm
            self.joint_states_received = True
            
            if self.q_target_smooth is None:
                self.q_target_smooth = q_arm.copy()
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse joint_states: {exc}')

    def raw_target_callback(self, msg: JointState):
        try:
            if len(msg.position) >= 7:
                self.q_target_raw = np.array(msg.position[:7], dtype=float)
                self.raw_target_received = True
                
                if not self.in_full_tracking and self.q_current is not None:
                    self.check_and_start_smooth()
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse raw target: {exc}')

    def check_and_start_smooth(self):
        errors = np.abs(self.q_target_raw - self.q_current)
        max_error = np.max(errors)
        
        if max_error > self.smooth_threshold:
            self.start_smooth_trajectory()
        else:
            self.in_full_tracking = True
            self.get_logger().info(f'Direct full tracking (max error: {max_error:.3f} rad)')

    def start_smooth_trajectory(self):
        self.trajectories = []
        for i in range(7):
            start_pos = float(self.q_current[i])
            end_pos = float(self.q_target_raw[i])
            traj = JointSmoothTrajectory(start_pos, end_pos, self.max_velocity, self.max_acceleration)
            self.trajectories.append(traj)
        
        max_duration = max(t.duration for t in self.trajectories)
        self.trajectory_start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(f'Starting S-curve smooth trajectory')
        self.get_logger().info(f'Max error: {np.max(np.abs(self.q_target_raw - self.q_current)):.3f} rad')
        self.get_logger().info(f'Trajectory duration: {max_duration:.2f} s')

    def timer_callback(self):
        if not self.joint_states_received or not self.raw_target_received:
            return

        if self.q_current is None or self.q_target_raw is None:
            return

        if self.in_full_tracking:
            self.q_target_smooth = self.q_target_raw.copy()
            self.publish_target()
            return

        if self.trajectories is not None and self.trajectory_start_time is not None:
            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed = current_time - self.trajectory_start_time
            
            all_finished = True
            for i in range(7):
                self.q_target_smooth[i] = self.trajectories[i].get_position(elapsed)
                if not self.trajectories[i].is_finished and elapsed < self.trajectories[i].duration:
                    all_finished = False
            
            self.q_target_smooth = np.clip(
                self.q_target_smooth, 
                self.joint_limits[:, 0], 
                self.joint_limits[:, 1]
            )
            
            self.publish_target()
            
            if all_finished:
                self.in_full_tracking = True
                self.trajectories = None
                self.trajectory_start_time = None
                self.get_logger().info('Smooth trajectory completed, switching to full tracking')
        else:
            self.q_target_smooth = self.q_current.copy()
            self.publish_target()

    def publish_target(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.arm_joint_names
        msg.position = self.q_target_smooth.tolist()
        msg.velocity = [0.0] * len(self.arm_joint_names)
        self.smooth_target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GelloSmootherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()