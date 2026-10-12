"""Run: python3 -m unittest discover -s tools/tests -v (from package root)."""
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from env_generator import EnvConfig, EnvGenerator, load_config


def box(name='part', **kwargs):
    return dict(id=name, type='Box', dimensions=[0.2, 0.4, 0.6], **kwargs)


def generator(objects):
    result = EnvGenerator(EnvConfig.from_dict({'collision_objects': objects}))
    valid, errors = result.validate()
    if not valid:
        raise AssertionError(errors)
    return result


class EnvGeneratorTests(unittest.TestCase):
    def test_rotated_compound_world_pose_and_xml_local_pose(self):
        q = math.sqrt(0.5)
        gen = generator([dict(id='assembly', type='Compound',
                              pose={'position': [1, 2, 3], 'orientation': [0, 0, q, q]},
                              parts=[box(pose={'position': [1, 0, 0],
                                               'orientation': [q, 0, 0, q]})])])
        root = ET.fromstring(gen.generate_mujoco_xml())
        body = root.find('worldbody/body')
        self.assertIsNone(body.find('freejoint'))
        self.assertEqual(body.find('geom').get('pos'), '1 0 0')
        self.assertAlmostEqual(float(body.get('quat').split()[0]), q)
        entry = yaml.safe_load(gen.generate_mprc_yaml())['collision_objects'][0]
        self.assertEqual(entry['id'], 'assembly__part')
        self.assertEqual(entry['pose']['position'], [1, 3, 3])
        self.assertEqual(entry['pose']['orientation'], [0.5, 0.5, 0.5, 0.5])
        pose = yaml.safe_load(gen.generate_curobo_yaml())['cuboid']['assembly__part']['pose']
        self.assertEqual(pose, [1, 3, 3, 0.5, 0.5, 0.5, 0.5])

    def test_movable_compound_is_one_body_with_one_joint(self):
        gen = generator([dict(id='tool', type='Compound', movable=True, gravcomp=1,
                              density=250, material={'rgba': [1, 0, 0, 1]},
                              parts=[box('a'), box('b', density=500,
                                                  material={'rgba': [0, 1, 0, 1]})])])
        root = ET.fromstring(gen.generate_mujoco_xml())
        self.assertEqual(len(root.findall('.//body')), 1)
        self.assertEqual(len(root.findall('.//freejoint')), 1)
        geoms = root.findall('.//geom')
        self.assertEqual(len(geoms), 2)
        self.assertEqual([g.get('density') for g in geoms], ['250', '500'])
        self.assertEqual([g.get('rgba') for g in geoms], ['1 0 0 1', '0 1 0 1'])
        self.assertEqual(root.find('.//body').get('gravcomp'), '1')
        self.assertEqual(yaml.safe_load(gen.generate_mprc_yaml()), {'collision_objects': []})
        self.assertEqual(yaml.safe_load(gen.generate_curobo_yaml()), {'cuboid': {}})

    def test_nested_parts_still_share_one_body(self):
        gen = generator([dict(id='a', type='Compound', parts=[
            dict(id='b', type='Compound', pose={'position': [1, 0, 0]},
                 parts=[box('c', pose={'position': [0, 2, 0]})])])])
        root = ET.fromstring(gen.generate_mujoco_xml())
        self.assertEqual(len(root.findall('.//body')), 1)
        self.assertEqual(root.find('.//geom').get('pos'), '1 2 0')
        self.assertEqual(gen._static_objects()[0].id, 'a__b__c')

    def test_primitive_compatibility_and_mujoco_quaternion(self):
        gen = generator([box('static', pose={'orientation': [1, 0, 0, 0]}),
                         box('dynamic', movable=True)])
        root = ET.fromstring(gen.generate_mujoco_xml())
        self.assertEqual(root.find('worldbody/geom').get('quat'), '0 1 0 0')
        self.assertEqual(root.find('worldbody/geom').get('size'), '0.1 0.2 0.3')
        self.assertEqual(root.find('.//body').get('name'), 'dynamic')
        self.assertEqual(root.find('.//body/geom').get('name'), 'dynamic_geom')
        self.assertEqual(len(gen._static_objects()), 1)

    def test_non_box_curobo_bounds(self):
        gen = generator([dict(id='c', type='Cylinder', dimensions=[0.2, 0.5]),
                         dict(id='s', type='Sphere', dimensions=[0.3])])
        cuboids = yaml.safe_load(gen.generate_curobo_yaml())['cuboid']
        self.assertEqual(cuboids['c']['dims'], [0.4, 0.4, 0.5])
        self.assertEqual(cuboids['s']['dims'], [0.6, 0.6, 0.6])

    def test_invalid_compounds(self):
        cases = [
            [dict(id='a', type='Compound', parts=[])],
            [dict(id='a', type='Compound', parts=[box(movable=True)])],
            [dict(id='a', type='Compound', parts=[box(gravcomp=1)])],
            [dict(id='a', type='Compound', parts=[box(), box()])],
            [dict(id='a', type='Compound', parts=[box('b')]), box('a__b')],
            [box(pose={'orientation': [0, 0, 0, 0]})],
            [box(pose={'position': [0, 0]})],
            [dict(id='a', type='Compound', dimensions=[1], parts=[box()])],
        ]
        for objects in cases:
            with self.subTest(objects=objects):
                self.assertFalse(EnvGenerator(EnvConfig.from_dict(
                    {'collision_objects': objects})).validate()[0])

    def test_all_repository_examples(self):
        for path in (Path(__file__).resolve().parents[1] / 'env_config').glob('*.yaml'):
            with self.subTest(path=path.name):
                gen = EnvGenerator(load_config(path))
                self.assertEqual(gen.validate(), (True, []))
                ET.fromstring(gen.generate_mujoco_xml())
                self.assertIsInstance(yaml.safe_load(gen.generate_mprc_yaml())['collision_objects'], list)
                self.assertIsInstance(yaml.safe_load(gen.generate_curobo_yaml())['cuboid'], dict)


if __name__ == '__main__':
    unittest.main()
