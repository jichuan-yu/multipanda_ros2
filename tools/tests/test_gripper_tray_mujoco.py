"""Physical grip regression; mocap drives hands, never the free tray or cubes."""
import copy
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from env_generator import EnvGenerator, load_config


@unittest.skipIf(mujoco is None, 'MuJoCo Python package is not installed')
class TrayGripTests(unittest.TestCase):
    def test_two_grippers_lift_and_hold_free_tray(self):
        self.check_lift_and_hold(stop_at_force=False)

    def test_hold_after_action_force_threshold(self):
        self.check_lift_and_hold(stop_at_force=True)

    def check_lift_and_hold(self, stop_at_force):
        assets = TOOLS.parent / 'franka_description/mujoco/franka'
        robot = ET.parse(assets / 'mj_dual.xml').getroot()
        root = ET.Element('mujoco')
        ET.SubElement(root, 'compiler', angle='radian', meshdir=str(assets / 'assets'))
        for tag in ['option', 'default', 'asset']:
            root.append(copy.deepcopy(robot.find(tag)))
        world = ET.SubElement(root, 'worldbody')
        ET.SubElement(world, 'geom', type='plane', size='2 2 .05')
        scene = EnvGenerator(load_config(TOOLS / 'env_config/tray_transport.yaml'))
        world.extend(ET.fromstring(scene.generate_mujoco_xml()).find('worldbody'))
        tendons = ET.SubElement(root, 'tendon')
        equality = ET.SubElement(root, 'equality')
        actuators = ET.SubElement(root, 'actuator')
        for side, x in [('left', .695), ('right', .505)]:
            prefix = f'mj_{side}'
            hand = copy.deepcopy(robot.find(f".//body[@name='{prefix}_hand']"))
            hand.set('childclass', 'panda')
            hand.set('pos', f'{x} .35 .3229')
            hand.set('quat', '0 .7071067811865476 .7071067811865476 0')
            ET.SubElement(hand, 'freejoint')
            world.append(hand)
            tendons.append(copy.deepcopy(robot.find(f".//fixed[@name='{prefix}_split']")))
            equality.append(copy.deepcopy(robot.find(f"equality/joint[@joint1='{prefix}_finger_joint1']")))
            actuators.append(copy.deepcopy(robot.find(f"actuator/general[@name='{prefix}_act_gripper']")))
            ET.SubElement(world, 'body', name=prefix + '_target', mocap='true',
                          pos=hand.get('pos'), quat=hand.get('quat'))
            ET.SubElement(equality, 'weld', body1=prefix + '_hand', body2=prefix + '_target',
                          solref='.004 1', solimp='.95 .99 .001')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        data = mujoco.MjData(model)
        for side in ['left', 'right']:
            for finger in [1, 2]:
                joint = model.joint(f'mj_{side}_finger_joint{finger}').id
                data.qpos[model.jnt_qposadr[joint]] = .02
        tray = model.body('tray').id
        held_heights = []
        command_width = np.full(2, .04)
        stopped = np.zeros(2, dtype=bool)
        for step in range(round(127 / model.opt.timestep)):
            time = step * model.opt.timestep
            width = max(0, .04 - .02 * max(0, time - .5))
            if stop_at_force:
                # Replay the action server: 100 Hz, 0.05 m/s, freeze each
                # command when its measured actuator force reaches 30 N.
                if step % 5 == 0 and time >= .5:
                    stopped |= np.abs(data.actuator_force) >= 30
                    command_width[~stopped] = np.maximum(
                        0, command_width[~stopped] - .0005)
                data.ctrl[:] = command_width / .08 * 255
            else:
                data.ctrl[:] = width / .08 * 255
            data.mocap_pos[:, 2] = .3229 + min(.12, .03 * max(0, time - 3))
            mujoco.mj_step(model, data)
            if time >= 7:
                held_heights.append(data.xpos[tray, 2])
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertFalse(any(w.number for w in data.warning))
        self.assertGreater(min(held_heights), .29)  # Lift >=9 cm and hold for 120 s.
        self.assertLess(max(held_heights) - min(held_heights), .002)
        self.assertLess(np.linalg.norm(data.xpos[tray, :2] - [.6, .35]), .02)


if __name__ == '__main__':
    unittest.main()
