"""Check the actual centered, tilted contact with free tray dynamics (no ROS)."""
import copy
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tray_transport_plan import build_plan, DEFAULT_MODEL, DEFAULT_SCENE, quaternion_wxyz_to_matrix


class CenterGripPhysicsTests(unittest.TestCase):
    def test_center_grasp_lifts_free_tray_after_force_threshold(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest('MuJoCo unavailable')
        sys.path.insert(0, str(DEFAULT_SCENE.parents[1]))
        from env_generator import EnvGenerator, load_config
        robot = ET.parse(DEFAULT_MODEL).getroot()
        root = ET.Element('mujoco')
        ET.SubElement(root, 'compiler', angle='radian', meshdir=str(DEFAULT_MODEL.parent/'assets'))
        for tag in ['option', 'default', 'asset']:
            root.append(copy.deepcopy(robot.find(tag)))
        world = ET.SubElement(root, 'worldbody')
        ET.SubElement(world, 'geom', type='plane', size='2 2 .05')
        world.extend(ET.fromstring(EnvGenerator(load_config(DEFAULT_SCENE)).generate_mujoco_xml()).find('worldbody'))
        tendons = ET.SubElement(root, 'tendon')
        equality = ET.SubElement(root, 'equality')
        actuators = ET.SubElement(root, 'actuator')
        grasp = next(s.target for s in build_plan().stages if s.name == 'close')
        origins = []
        for arm, side in enumerate(['left', 'right']):
            prefix = f'mj_{side}'
            hand = copy.deepcopy(robot.find(f".//body[@name='{prefix}_hand']"))
            tcp = next(b for b in hand.findall('body') if b.find(f"site[@name='{prefix}_ee_site']") is not None)
            quat = np.roll(grasp[arm, 3:], 1)
            origin = grasp[arm, :3]-quaternion_wxyz_to_matrix(quat) @ np.fromstring(tcp.get('pos'), sep=' ')
            origins.append(origin)
            hand.set('childclass', 'panda')
            hand.set('pos', ' '.join(map(str, origin)))
            hand.set('quat', ' '.join(map(str, quat)))
            ET.SubElement(hand, 'freejoint')
            world.append(hand)
            tendons.append(copy.deepcopy(robot.find(f".//fixed[@name='{prefix}_split']")))
            equality.append(copy.deepcopy(robot.find(f"equality/joint[@joint1='{prefix}_finger_joint1']")))
            actuators.append(copy.deepcopy(robot.find(f"actuator/general[@name='{prefix}_act_gripper']")))
            ET.SubElement(world, 'body', name=prefix+'_target', mocap='true', pos=hand.get('pos'), quat=hand.get('quat'))
            ET.SubElement(equality, 'weld', body1=prefix+'_hand', body2=prefix+'_target', solref='.004 1', solimp='.95 .99 .001')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        data = mujoco.MjData(model)
        for side in ['left', 'right']:
            for finger in [1, 2]:
                data.qpos[model.jnt_qposadr[model.joint(f'mj_{side}_finger_joint{finger}').id]] = .04
        command = np.full(2, .08)
        stopped = np.zeros(2, dtype=bool)
        heights = []
        for step in range(round(15/model.opt.timestep)):
            now = step*model.opt.timestep
            if step % 5 == 0 and now >= .5:
                stopped |= np.abs(data.actuator_force) >= 30.
                command[~stopped] = np.maximum(0., command[~stopped]-.0005)
            data.ctrl[:] = command/.08*255
            data.mocap_pos[:] = origins
            data.mocap_pos[:, 2] += min(.09, .03*max(0., now-3))
            mujoco.mj_step(model, data)
            if now > 7:
                heights.append(data.xpos[model.body('tray').id, 2])
        self.assertTrue(stopped.all())
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertFalse(any(w.number for w in data.warning))
        self.assertGreater(min(heights), .27)
        self.assertLess(max(heights)-min(heights), .003)
