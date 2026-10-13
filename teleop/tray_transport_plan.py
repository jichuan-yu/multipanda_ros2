"""Scene-derived Cartesian reference plan. No ROS or robot commands in this module."""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from utils.panda_dh_kinematics import (
    PandaDHKinematics, HOME_POSITION, matrix_to_quaternion_wxyz,
    quaternion_wxyz_to_matrix,
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = ROOT / 'tools/env_config/tray_transport.yaml'
DEFAULT_MODEL = ROOT / 'franka_description/mujoco/franka/mj_dual.xml'


def rotation(pose):
    q = np.asarray(pose.get('orientation', [0, 0, 0, 1]), dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid scene quaternion')
    return quaternion_wxyz_to_matrix(np.roll(q / np.linalg.norm(q), 1))


def transform(obj):
    pose = obj.get('pose', {})
    return np.asarray(pose.get('position', [0, 0, 0]), dtype=float), rotation(pose)


def angle(q1, q2):
    return 2 * np.arccos(np.clip(abs(np.dot(q1, q2)), 0, 1))


def interpolate(a, b, u):
    """Two world poses [xyz, xyzw]; shortest-arc, constant-progress SLERP."""
    out = (1-u)*a + u*b
    for arm in range(2):
        qa, qb = a[arm, 3:].copy(), b[arm, 3:].copy()
        dot = np.dot(qa, qb)
        if dot < 0:
            qb, dot = -qb, -dot
        if dot > .9995:
            q = (1-u)*qa + u*qb
        else:
            theta = np.arccos(np.clip(dot, -1, 1))
            q = (np.sin((1-u)*theta)*qa + np.sin(u*theta)*qb)/np.sin(theta)
        out[arm, 3:] = q / np.linalg.norm(q)
    return out


def errors(actual, target):
    return (max(np.linalg.norm(actual[:, :3]-target[:, :3], axis=1)),
            max(angle(a[3:], b[3:]) for a, b in zip(actual, target)))


@dataclass
class Stage:
    name: str
    target: np.ndarray
    speed: float
    action: str = ''
    cooperative: bool = False


@dataclass
class Plan:
    stages: list
    walls: list
    wall_widths: list
    open_width: float
    metadata: dict


def build_plan(scene_path=DEFAULT_SCENE, model_path=DEFAULT_MODEL, start=None):
    """Derive contacts from world +/-Y walls and Panda pad/TCP geometry.

    This deliberately generates a task reference, not an OMPL/cuRobo global
    collision-free path. Safety controller remains the motion authority.
    """
    with open(scene_path) as stream:
        scene = yaml.safe_load(stream)
    cfg = scene.get('tray_transport_input', {})
    transition_defaults = {'motion_ramp_time': .5, 'arrival_settle_time': .1,
                           'grasp_settle_time': .25, 'gripper_settle_time': .15,
                           'cooperation_settle_time': .05}
    for key, default in transition_defaults.items():
        value = float(cfg.get(key, default))
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
        cfg[key] = value
    objects = {o['id']: o for o in scene['collision_objects']}
    tray = objects[cfg.get('tray', 'tray')]
    stool = objects[cfg.get('destination', 'right_stool')]
    tp, tr = transform(tray)
    sp, sr = transform(stool)
    if not np.allclose(tr[:, 2], [0, 0, 1], atol=1e-6) or not np.allclose(sr[:, 2], [0, 0, 1], atol=1e-6):
        raise ValueError('Only horizontal tray/support with arbitrary yaw are supported')
    parts = {p['id']: p for p in tray['parts']}
    walls = [p for p in parts.values() if p['id'].endswith('_wall')]
    if len(walls) != 4:
        raise ValueError('Expected four tray walls')
    centers = [tp + tr @ transform(w)[0] for w in walls]
    selected = [walls[int(np.argmax(np.array(centers)[:, 1]))],
                walls[int(np.argmin(np.array(centers)[:, 1]))]]
    if selected[0]['id'] == selected[1]['id']:
        raise ValueError('Cannot identify opposite world-Y walls')

    model = ET.parse(model_path).getroot()
    # Existing DH helpers define the same TCP as these model sites.
    initial = home_poses() if start is None else np.asarray(start, dtype=float)
    poses, widths, opening, contacts = [], [], [], []
    tilt_degrees = float(cfg.get('grasp_tilt_degrees', 10.))
    if not np.isfinite(tilt_degrees) or not 0 <= tilt_degrees <= 10:
        raise ValueError('grasp_tilt_degrees must be in [0, 10] to retain pad clearance')
    tilt = np.deg2rad(tilt_degrees)
    min_overlap = float(cfg.get('grasp_min_overlap', .008))
    if not np.isfinite(min_overlap) or min_overlap <= 0:
        raise ValueError('grasp_min_overlap must be positive')
    for side, wall in zip(['left', 'right'], selected):
        hand = model.find(f".//body[@name='mj_{side}_hand']")
        finger = hand.find(f"body[@name='mj_{side}_left_finger']")
        tcp_body = next(b for b in hand.findall('body')
                        if b.find(f"site[@name='mj_{side}_ee_site']") is not None)
        tcp = np.fromstring(tcp_body.get('pos'), sep=' ')
        pad = model.find(".//default[@class='fingertip_pad_collision_1']/geom")
        pad_center = np.fromstring(finger.get('pos'), sep=' ') + np.fromstring(pad.get('pos'), sep=' ')
        pad_lateral_offset = abs(pad_center[1])
        # Opposing pads mirror their lateral offsets.
        pad_center[:2] = 0.
        pad_half = np.fromstring(pad.get('size'), sep=' ')
        limits = np.fromstring(model.find(".//default[@class='finger']/joint").get('range'), sep=' ')
        opening.append(2*limits[1])
        wp, wr = transform(wall)
        dims = np.asarray(wall['dimensions'], dtype=float)
        long_axis = int(np.argmax(dims[:2]))
        thin_axis = 1-long_axis
        tangent = tr @ wr[:, long_axis]
        normal = tr @ wr[:, thin_axis]
        if not np.allclose((tr @ wr)[:, 2], [0, 0, 1], atol=1e-6):
            raise ValueError('Grasp walls must be vertical with horizontal top edges')
        # Either sign of the closing axis gives the same symmetric grasp.
        if normal[1] < 0:
            normal = -normal
        center = tp + tr @ wp
        base = PandaDHKinematics(side).T_base[:3, 3]
        # Lean the fingers away from the base along the edge, keeping the wrist nearer.
        lean = (-1. if np.dot(base-center, tangent) > 0 else 1.)*tangent
        z = np.sin(tilt)*lean + np.cos(tilt)*np.array([0., 0., -1.])
        if abs(np.dot(normal, z)) > 1e-6:
            raise ValueError('Selected walls do not support the configured grasp lean')
        candidates = [np.column_stack([np.cross(sign*normal, z), sign*normal, z])
                      for sign in [1., -1.]]
        # Parallel fingers are symmetric under a half-turn about tool Z.
        # Select each arm's equivalent orientation nearest its actual start.
        arm = len(poses)
        rot = min(candidates, key=lambda r: angle(
            np.roll(matrix_to_quaternion_wxyz(r), -1), initial[arm, 3:]))
        if dims[long_axis] <= 2*pad_half[0] or dims[thin_axis] + .01 >= opening[-1]:
            raise ValueError('Wall does not fit Panda fingers')
        # Center the main finger pads on the wall, both along its edge and height.
        ee = center + rot @ (tcp-pad_center)
        quat = np.roll(matrix_to_quaternion_wxyz(rot), -1)
        poses.append(np.r_[ee, quat])
        widths.append(float(dims[thin_axis]))
        if min_overlap > min(dims[2], 2*pad_half[2]):
            raise ValueError('grasp_min_overlap exceeds wall/pad height')
        contacts.append({
            'wall_z_min': float(center[2]-dims[2]/2),
            'wall_z_max': float(center[2]+dims[2]/2),
            'pad_offset_tcp': (pad_center-tcp).tolist(),
            'pad_half_size': pad_half.tolist(),
            # Bound opposing pad height differences for small tool tilt.
            'finger_lateral_reach': float(opening[-1]/2+pad_lateral_offset+pad_half[1]),
        })
    grasp = np.array(poses)

    seat = next(p for p in stool['parts'] if p['id'] == 'seat')
    seat_p, seat_r = transform(seat)
    if not np.allclose((sr @ seat_r)[:, 2], [0, 0, 1], atol=1e-6):
        raise ValueError('Tilted seat is unsupported')
    seat_center = sp + sr @ seat_p
    seat_top = seat_center[2] + seat['dimensions'][2]/2
    base_p, base_r = transform(parts['base'])
    if not np.allclose(base_r, np.eye(3)):
        raise ValueError('Rotated tray base is unsupported')
    bottom_local = base_p[2] - parts['base']['dimensions'][2]/2
    destination = np.r_[seat_center[:2], seat_top-bottom_local]
    displacement = destination-tp
    place = grasp.copy()
    place[:, :3] += displacement
    approach = float(cfg.get('approach_clearance', .08))
    clearance = float(cfg.get('transport_clearance', .08))
    retreat = float(cfg.get('retreat_clearance', .08))
    free_speed = float(cfg.get('free_speed', .04))
    carry_speed = float(cfg.get('carry_speed', .025))
    contact_speed = float(cfg.get('contact_speed', .008))
    min_scale = float(cfg.get('min_playback_scale', .2))
    if not np.isfinite(min_scale) or not 0 < min_scale <= 1:
        raise ValueError('min_playback_scale must be in (0, 1]')
    values = [approach, clearance, retreat, free_speed, carry_speed, contact_speed]
    if not np.isfinite(values).all() or min(values) <= 0:
        raise ValueError('Clearances and speeds must be positive')
    ready = grasp.copy(); ready[:, 2] += approach
    lift = grasp.copy(); lift[:, 2] += max(0, displacement[2])+clearance
    above = place.copy(); above[:, 2] += max(0, -displacement[2])+clearance
    end = place.copy(); end[:, 2] += retreat
    transit_clearance = float(cfg.get('transit_min_clearance', .025))
    transit_xy = float(cfg.get('transport_xy_tolerance', .03))
    transit_rotation = float(cfg.get('transit_orientation_tolerance', .05))
    if (not np.isfinite([transit_clearance, transit_xy, transit_rotation]).all()
            or not 0 < transit_clearance <= clearance or transit_xy <= 0
            or not 0 < transit_rotation <= .1):
        raise ValueError('Invalid transit clearance/tolerances')
    stages = [
        Stage('approach', ready, free_speed),
        Stage('open_before_pick', ready, contact_speed, 'open'),
        Stage('descend_to_grasp', grasp, contact_speed),
        Stage('close', grasp, contact_speed, 'close'),
        Stage('enable_cooperation', grasp, contact_speed, 'enable', True),
        Stage('lift', lift, contact_speed, cooperative=True),
        Stage('transport', above, carry_speed, cooperative=True),
        Stage('place', place, contact_speed, cooperative=True),
        Stage('release', place, contact_speed, 'open', True),
        Stage('disable_cooperation', place, contact_speed, 'disable'),
        Stage('retreat', end, free_speed),
    ]
    return Plan(stages, [w['id'] for w in selected], widths, min(opening), {
        'scene': str(Path(scene_path).resolve()), 'model': str(Path(model_path).resolve()),
        'tray_initial_position': tp.tolist(), 'tray_destination_position': destination.tolist(),
        'seat_top': float(seat_top), 'configuration': cfg,
        'transit_acceptance': {
            'minimum_clearance': transit_clearance, 'xy_tolerance': transit_xy,
            'orientation_tolerance': transit_rotation,
            'seat_rotation': (sr @ seat_r).tolist(),
            'seat_half_size': (np.array(seat['dimensions'][:2])/2).tolist(),
            'tray_radius': float(np.linalg.norm(parts['base']['dimensions'][:2])/2),
        },
        'grasp_acceptance': {
            'minimum_overlap': min_overlap, 'xy_tolerance': .003,
            'orientation_tolerance': .025, 'lower_z_tolerance': .003,
            'settle_seconds': cfg['grasp_settle_time'], 'settle_position': .0008, 'settle_rotation': .008,
            'contacts': contacts,
        },
        'planning_kind': 'Cartesian task reference; collision avoidance by safety controller',
    })


def transit_region_status(plan, stage, actual):
    """Intermediate carry waypoints use clearance/placement corridor, not exact XYZ.

    The captured grasp offset is already included in the place target. Estimate
    tray bottom conservatively from both hands and subtract the corner drop
    caused by orientation error. This is a geometric gate, not collision proof.
    """
    if stage.name not in ('lift', 'transport') or not np.isfinite(actual).all():
        return False, 'not a valid carry transition'
    cfg = plan.metadata['transit_acceptance']
    place = next(s.target for s in plan.stages if s.name == 'place')
    delta = actual[:, :3]-place[:, :3]
    re = errors(actual, place)[1]
    clearance = float(min(delta[:, 2])-cfg['tray_radius']*np.sin(re))
    xy = max(np.linalg.norm(delta[:, :2], axis=1))
    valid = clearance >= cfg['minimum_clearance'] and re <= cfg['orientation_tolerance']
    if stage.name == 'transport':
        # Both inferred tray centers must be near the seat center and inside it.
        local = delta @ np.array(cfg['seat_rotation'])
        margin = np.array(cfg['seat_half_size'])-.03
        valid &= xy <= cfg['xy_tolerance'] and bool(np.all(np.abs(local[:, :2]) <= margin))
    return bool(valid), f'clearance={clearance:.4f}m, destination_xy={xy:.4f}m, rotation={re:.3f}rad'


def grasp_region_status(plan, actual):
    """Both hands must cover enough wall height; no blanket Z error relaxation.

    Use a conservative common vertical span of both pads under tool tilt.
    The nominal lower limit is retained to avoid accepting an overshoot below
    the tray wall. XY/rotation bounds remain as strict as normal arrival.
    """
    cfg = plan.metadata['grasp_acceptance']
    nominal = next(s.target for s in plan.stages if s.name == 'descend_to_grasp')
    overlaps = []
    valid = bool(np.isfinite(actual).all())
    for arm, contact in enumerate(cfg['contacts']):
        rotation_actual = quaternion_wxyz_to_matrix(np.roll(actual[arm, 3:], 1))
        pad_center = actual[arm, :3]+rotation_actual @ np.array(contact['pad_offset_tcp'])
        half = contact['pad_half_size']
        half_height = max(0., abs(rotation_actual[2, 2])*half[2]
                          - abs(rotation_actual[2, 0])*half[0]
                          - abs(rotation_actual[2, 1])*contact['finger_lateral_reach'])
        overlap = max(0., min(contact['wall_z_max'], pad_center[2]+half_height)
                      - max(contact['wall_z_min'], pad_center[2]-half_height))
        overlaps.append(overlap)
        valid &= (overlap+1e-9 >= cfg['minimum_overlap']
                  and np.linalg.norm(actual[arm, :2]-nominal[arm, :2]) <= cfg['xy_tolerance']
                  and angle(actual[arm, 3:], nominal[arm, 3:]) <= cfg['orientation_tolerance']
                  and actual[arm, 2] >= nominal[arm, 2]-cfg['lower_z_tolerance'])
    return bool(valid), overlaps


class GraspArrival:
    """Accept a viable and stationary grasp even before the virtual clock ends."""
    def __init__(self, plan):
        self.plan = plan
        self.anchor = None
        self.since = None

    def update(self, actual, now):
        valid, _ = grasp_region_status(self.plan, actual)
        if not valid:
            self.anchor = self.since = None
            return False
        cfg = self.plan.metadata['grasp_acceptance']
        if self.anchor is None or now < self.since:
            self.anchor, self.since = actual.copy(), now
        pe, re = errors(actual, self.anchor)
        if pe > cfg['settle_position'] or re > cfg['settle_rotation']:
            self.anchor, self.since = actual.copy(), now
        return now-self.since >= cfg['settle_seconds']


def home_poses():
    poses = []
    for side in ['left', 'right']:
        p, q = PandaDHKinematics(side).forward_position_quat(HOME_POSITION)
        poses.append(np.r_[p, np.roll(q, -1)])
    return np.array(poses)


def validate_reachability(plan, start, joints, spacing=.005, collision_check=None):
    """Sample the entire reference; reject unreachable segments before publishing."""
    seeds = np.array(joints, dtype=float).copy()
    models = [PandaDHKinematics(side) for side in ['left', 'right']]
    previous = np.array(start, dtype=float)
    count = 0
    for stage in plan.stages:
        pos, rot = errors(previous, stage.target)
        steps = max(1, int(np.ceil(max(pos/spacing, rot/.04))))
        for i in range(steps+1):
            pose = interpolate(previous, stage.target, i/steps)
            for arm, model in enumerate(models):
                q, pe, re, ok = model.inverse_kinematics(
                    pose[arm, :3], np.roll(pose[arm, 3:], 1), seeds[arm],
                    tol=.001, max_iter=600)
                if not ok:
                    raise ValueError(f'{stage.name}: {model.side} unreachable at {i}/{steps}; '
                                     f'position error={pe:.4f}m, rotation error={re:.4f}rad')
                seeds[arm] = q
            if collision_check is not None:
                collision_check(stage, pose, seeds)
            count += 1
        previous = stage.target
    return count


class SegmentPlayer:
    """Shared clock: fixed-duration smooth velocity ramps with a cruise interval."""
    def __init__(self, start, goal, speed, angular_speed=.25, min_playback_scale=.2, ramp_time=.5):
        if not np.isfinite(min_playback_scale) or not 0 < min_playback_scale <= 1:
            raise ValueError('min_playback_scale must be in (0, 1]')
        if not np.isfinite([speed, angular_speed, ramp_time]).all() or min(speed, angular_speed, ramp_time) <= 0:
            raise ValueError('Speeds and ramp_time must be finite and positive')
        self.min_playback_scale = float(min_playback_scale)
        self.start, self.goal = start.copy(), goal.copy()
        distance, rotation_error = errors(start, goal)
        self.ramp_time = float(ramp_time)
        travel_time = max(distance/speed, rotation_error/angular_speed)
        # Integral of each smoothstep velocity ramp is half its duration.
        # Short segments lower peak speed instead of exceeding acceleration.
        self.duration = max(2*ramp_time, travel_time+ramp_time) if travel_time > 1e-9 else 0.
        self.progress_rate = 1/(self.duration-ramp_time) if self.duration else 0.
        self.elapsed = 0.
        self.target = start.copy()
        # The geometric profile already starts at zero velocity; avoid a
        # second time-scale ramp at every stage boundary.
        self.scale = 1.

    def progress_at(self, elapsed):
        if self.duration == 0 or elapsed >= self.duration:
            return 1.
        t = max(0., elapsed)
        r = self.ramp_time
        if t < r:
            u = t/r
            return self.progress_rate*r*(u**3-.5*u**4)
        if t > self.duration-r:
            u = (self.duration-t)/r
            return 1-self.progress_rate*r*(u**3-.5*u**4)
        return self.progress_rate*(t-.5*r)

    def governor(self, actual, target):
        p, r = errors(actual, target)
        return float(max(self.min_playback_scale,
                         min(np.clip((.025-p)/.020, 0, 1),
                             np.clip((.15-r)/.12, 0, 1))))

    def advance(self, actual, dt):
        desired = self.governor(actual, self.target)
        # Tracking error may slow the virtual clock but cannot freeze it.
        # Smoothly recover the time scale after tracking-error slowdown.
        self.scale = min(desired, self.scale + 2*dt)
        self.elapsed = min(self.duration, self.elapsed + max(0, dt)*self.scale)
        self.target = interpolate(self.start, self.goal, self.progress_at(self.elapsed))
        return self.target.copy()

    @property
    def done(self):
        return self.elapsed >= self.duration
