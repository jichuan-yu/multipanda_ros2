"""Offline mesh-contact checks for the nominal task reference (never steps physics).

This model is independent of the running simulator. Discrete contact checking of
an IK branch is not a guarantee for a safety controller's actual joint trajectory.
"""
import sys
import xml.etree.ElementTree as ET

import numpy as np
from tray_transport_plan import ROOT


class ReferenceCollisionCheck:
    def __init__(self, plan, scene_path, model_path, initial_widths=None):
        import mujoco
        sys.path.insert(0, str(ROOT / 'tools'))
        from env_generator import EnvGenerator, load_config
        self.mujoco = mujoco
        root = ET.parse(model_path).getroot()
        root.find('compiler').set('meshdir', str(model_path.resolve().parent/'assets'))
        scene = load_config(scene_path)
        root.find('worldbody').extend(ET.fromstring(
            EnvGenerator(scene).generate_mujoco_xml()).find('worldbody'))
        ET.SubElement(root.find('worldbody'), 'geom', name='reference_floor',
                      type='plane', size='2 2 .05')
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.initial = self.data.qpos.copy()
        self.arm_indices = [[m.jnt_qposadr[m.joint(f'mj_{s}_joint{i}').id]
                             for i in range(1, 8)] for s in ['left', 'right']]
        self.finger_indices = [[m.jnt_qposadr[m.joint(f'mj_{s}_finger_joint{i}').id]
                                for i in [1, 2]] for s in ['left', 'right']]
        self.tray_name = plan.metadata['configuration'].get('tray', 'tray')
        # This environment explicitly identifies its six stacked payload cubes.
        self.payload = {self.tray_name} | {m.body(i).name for i in range(m.nbody)
                                          if m.body(i).name.startswith('cube_')}
        self.grasp = next(s.target.copy() for s in plan.stages if s.name == 'close')
        self.widths = list(initial_widths if initial_widths is not None else [plan.open_width]*2)
        self.wall_widths = plan.wall_widths
        self.open_width = plan.open_width
        self.carrying = False
        self.stage = None
        self.count = 0

    def __call__(self, stage, pose, joints):
        m, d = self.model, self.data
        if stage.name != self.stage:
            self.stage = stage.name
            if stage.action == 'close':
                self.widths = self.wall_widths
            if stage.action == 'enable':
                self.carrying = True
            if stage.action == 'open':
                self.widths = [self.open_width]*2
                # Placement endpoint was already checked; leave the tray there.
                if stage.name == 'release':
                    self.carrying = False
        for a in range(2):
            d.qpos[self.arm_indices[a]] = joints[a]
            d.qpos[self.finger_indices[a]] = self.widths[a]/2
        if self.carrying:
            delta = pose[0, :3]-self.grasp[0, :3]
            for name in self.payload:
                body = m.body(name).id
                adr = m.jnt_qposadr[m.body_jntadr[body]]
                d.qpos[adr:adr+3] = self.initial[adr:adr+3]+delta
        self.mujoco.mj_forward(m, d)
        for contact in d.contact:
            names = [m.body(m.geom_bodyid[g]).name for g in contact.geom]
            robot = any(n.startswith('mj_') for n in names)
            payload = any(n in self.payload for n in names)
            if not robot and not payload:
                continue
            if all(n in self.payload for n in names):
                continue  # Tray and stacked cubes have intended support contacts.
            if self.tray_name in names and any('finger' in n for n in names):
                continue  # Intentional finger/wall contact, all other meshes checked.
            if any(n.endswith('_link0') for n in names) and 'world' in names:
                continue  # Fixed bases mount on the floor.
            if contact.dist < -.001:
                raise ValueError(f'{stage.name}: nominal mesh collision {names}, '
                                 f'penetration={-contact.dist:.4f}m')
        self.count += 1
