"""Optional physics checks; skipped when the MuJoCo Python package is absent."""
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from env_generator import EnvGenerator, load_config

CONFIG_DIR = Path(__file__).resolve().parents[1] / 'env_config'


def model_xml(path):
    generator = EnvGenerator(load_config(path))
    valid, errors = generator.validate()
    if not valid:
        raise AssertionError(errors)
    root = ET.fromstring(generator.generate_mujoco_xml())
    root.tag = 'mujoco'
    return root


@unittest.skipIf(mujoco is None, 'MuJoCo Python package is not installed')
class MuJoCoEnvironmentTests(unittest.TestCase):
    def test_all_configs_compile(self):
        for path in CONFIG_DIR.glob('*.yaml'):
            with self.subTest(config=path.name):
                mujoco.MjModel.from_xml_string(ET.tostring(model_xml(path), encoding='unicode'))

    def test_cleaning_scene_supported_tools_stay_stable(self):
        root = model_xml(CONFIG_DIR / 'cleaning_scene.yaml')
        ET.SubElement(root.find('worldbody'), 'geom',
                      name='floor', type='plane', size='2 2 0.05')
        ET.SubElement(root, 'option', timestep='0.001')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.assertEqual(model.njnt, 10)  # Two tools and eight loose cubes.
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertTrue(all(c.dist >= -1e-6 for c in data.contact))
        broom_handle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'broom__handle_geom')
        dustpan_handle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'dustpan__handle_geom')
        rear_wall = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'dustpan__rear_wall_geom')
        support = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'fixed_block')
        # Cylinder axes are local Z; the broom leans 10 degrees, dustpan handle is vertical.
        self.assertAlmostEqual(data.geom_xmat[broom_handle].reshape(3, 3)[2, 2],
                               np.cos(np.deg2rad(10)), places=7)
        np.testing.assert_allclose(data.geom_xmat[dustpan_handle].reshape(3, 3)[:, 2], [0, 0, 1])
        self.assertAlmostEqual(data.geom_xpos[dustpan_handle, 2] - model.geom_size[dustpan_handle, 1],
                               data.geom_xpos[rear_wall, 2] + model.geom_size[rear_wall, 2])
        self.assertAlmostEqual(2 * model.geom_size[broom_handle, 1], 0.55)
        self.assertAlmostEqual(2 * model.geom_size[dustpan_handle, 1], 0.45)
        self.assertAlmostEqual(2 * model.geom_size[support, 2], 0.40)
        self.assertLess(np.linalg.norm(data.geom_xpos[dustpan_handle, :2] - [0, -0.26]), 0.37)
        np.testing.assert_allclose(2 * model.geom_size[support], [0.08, 0.08, 0.40])
        broom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'broom')
        dustpan = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'dustpan')
        self.assertLess(data.xpos[broom, 1], data.geom_xpos[support, 1])
        self.assertGreater(data.geom_xmat[broom_handle].reshape(3, 3)[1, 2], 0)
        # The dustpan's missing wall is local -X, now world +Y.
        np.testing.assert_allclose(-data.xmat[dustpan].reshape(3, 3)[:, 0], [0, 1, 0], atol=1e-8)
        debris = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'debris_{i:02}')
                  for i in range(1, 9)]
        xy = data.xpos[debris, :2]
        self.assertGreater(np.ptp(xy[:, 0]), 0.5)
        self.assertGreaterEqual(np.ptp(xy[:, 1]), 0.5)
        initial = data.xpos.copy()
        for _ in range(30000):
            mujoco.mj_step(model, data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertFalse(any(w.number for w in data.warning))
        self.assertLess(np.max(np.linalg.norm(data.xpos - initial, axis=1)), 0.002)
        self.assertGreater(data.geom_xmat[broom_handle].reshape(3, 3)[2, 2], np.cos(np.deg2rad(12)))
        self.assertTrue(any({c.geom1, c.geom2} == {broom_handle, support} for c in data.contact))
        for name in ['fixed_pillar', 'fixed_barrier']:
            self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name), -1)
        for name in ['fixed_block']:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            self.assertEqual(model.geom_bodyid[geom_id], 0)

    def test_cleaning_tools_clear_robot_home_pose(self):
        assets = CONFIG_DIR.parents[1] / 'franka_description/mujoco/franka'
        root = ET.parse(assets / 'mj_dual.xml').getroot()
        root.find('compiler').set('meshdir', str(assets / 'assets'))
        world = root.find('worldbody')
        world.extend(model_xml(CONFIG_DIR / 'cleaning_scene.yaml').find('worldbody'))
        ET.SubElement(world, 'geom', name='floor', type='plane', size='2 2 0.05')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        data = mujoco.MjData(model)
        # Defaults from my_task_description/launch/my_task_sim.launch.py.
        for side in ['left', 'right']:
            for i, value in enumerate([0, -.785, 0, -2.356, 0, 1.571, .785], 1):
                joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'mj_{side}_joint{i}')
                data.qpos[model.jnt_qposadr[joint]] = value
        mujoco.mj_forward(model, data)
        ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'mj_right_ee_site')
        handle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'dustpan__handle_geom')
        # Compare to the closest point on the vertical handle centreline.
        target = data.geom_xpos[handle].copy()
        half_length = model.geom_size[handle, 1]
        target[2] = np.clip(data.site_xpos[ee, 2], target[2] - half_length, target[2] + half_length)
        self.assertLess(np.linalg.norm(target - data.site_xpos[ee]), 0.11)
        for contact in data.contact:
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
                     for g in (contact.geom1, contact.geom2)]
            if any(name.startswith(('broom', 'dustpan', 'fixed_', 'debris_')) for name in names):
                self.assertGreaterEqual(contact.dist, -1e-5, names)

    def test_tray_transport_stack_stays_supported(self):
        root = model_xml(CONFIG_DIR / 'tray_transport.yaml')
        ET.SubElement(root.find('worldbody'), 'geom', name='floor', type='plane', size='2 2 0.05')
        ET.SubElement(root, 'option', timestep='0.002')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertEqual(model.njnt, 7)  # Tray and six independent cubes.
        self.assertTrue(all(c.dist >= -1e-6 for c in data.contact))
        def geom(name):
            return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name, size, height in [('left_stool', [.25, .15, .03], .2),
                                   ('right_stool', [.25, .2, .03], .35)]:
            seat = geom(name + '__seat_geom')
            np.testing.assert_allclose(2 * model.geom_size[seat], size)
            self.assertAlmostEqual(data.geom_xpos[seat, 2] + model.geom_size[seat, 2], height)
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            self.assertEqual(model.body_jntnum[body], 0)
        for name in ['left_stool__seat_geom', 'right_stool__seat_geom', 'tray__base_geom']:
            np.testing.assert_allclose(data.geom_xmat[geom(name)].reshape(3, 3)[:, 0],
                                       [0, 1, 0], atol=1e-8)
        base = geom('tray__base_geom')
        np.testing.assert_allclose(2 * model.geom_size[base], [.3, .2, .01])
        self.assertAlmostEqual(data.geom_xpos[base, 2] - model.geom_size[base, 2], .2)
        for wall in ['front', 'back', 'left', 'right']:
            self.assertAlmostEqual(2 * model.geom_size[geom('tray__' + wall + '_wall_geom'), 2], .02)
        for i in range(1, 7):
            cube = geom(f'cube_{i:02}_geom')
            np.testing.assert_allclose(2 * model.geom_size[cube], [.03, .03, .03])
            np.testing.assert_allclose(data.geom_xpos[cube], [.6, .35, .225 + (i - 1) * .03])
        initial = data.xpos.copy()
        # Match SimPausedPhysics: pause-time forward calls must not warm-start
        # the first integrated step with their constraint solution (MuJoCo 3.2).
        for _ in range(100):
            data.qacc_warmstart[:] = 0
            mujoco.mj_forward(model, data)
            data.qacc_warmstart[:] = 0
        for _ in range(15000):
            mujoco.mj_step(model, data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertFalse(any(w.number for w in data.warning))
        self.assertLess(np.max(np.linalg.norm(data.xpos - initial, axis=1)), .002)
        contact_pairs = [{c.geom1, c.geom2} for c in data.contact]
        self.assertIn({geom('cube_01_geom'), base}, contact_pairs)
        for i in range(1, 6):
            self.assertIn({geom(f'cube_{i:02}_geom'), geom(f'cube_{i+1:02}_geom')}, contact_pairs)

    def test_tray_transport_clear_robot_home_pose(self):
        assets = CONFIG_DIR.parents[1] / 'franka_description/mujoco/franka'
        root = ET.parse(assets / 'mj_dual.xml').getroot()
        root.find('compiler').set('meshdir', str(assets / 'assets'))
        root.find('worldbody').extend(model_xml(CONFIG_DIR / 'tray_transport.yaml').find('worldbody'))
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        data = mujoco.MjData(model)
        for side in ['left', 'right']:
            for i, value in enumerate([0, -.785, 0, -2.356, 0, 1.571, .785], 1):
                joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'mj_{side}_joint{i}')
                data.qpos[model.jnt_qposadr[joint]] = value
        mujoco.mj_forward(model, data)
        for contact in data.contact:
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
                     for g in (contact.geom1, contact.geom2)]
            if any(name.startswith(('left_stool', 'right_stool', 'tray', 'cube_')) for name in names):
                self.assertGreaterEqual(contact.dist, -1e-5, names)


if __name__ == '__main__':
    unittest.main()
