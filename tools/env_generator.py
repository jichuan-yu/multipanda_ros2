#!/usr/bin/env python3
"""
Flexible Environment Generator for Dual Panda Simulation

Generates MuJoCo XML, MPRC YAML, and cuRobo Scene YAML from user-defined config files.
Supports Box, Cylinder, Sphere, and rigid Compound objects.

Usage:
    python3 env_generator.py --config env_config/my_env.yaml [OPTIONS]

Options:
    --config PATH           Input config YAML file (required)
    --xml-output PATH       Output path for MuJoCo XML file
    --yaml-output PATH      Output path for MPRC YAML file
    --curobo-output PATH    Output path for cuRobo Scene YAML file
    --dry-run               Print configuration without writing files
"""

import argparse
import sys
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import List, Dict, Any, Tuple

try:
    import yaml
except ImportError:
    print("Error: PyYAML is required. Install with: pip install pyyaml")
    sys.exit(1)


@dataclass
class Material:
    """Material properties for collision objects."""
    friction: Tuple[float, float, float] = (2.0, 0.005, 0.0001)
    rgba: Tuple[float, float, float, float] = (0.2, 0.4, 0.8, 1.0)


@dataclass
class Pose:
    """Position and orientation."""
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass
class CollisionObject:
    """Represents a collision object."""
    id: str
    type: str  # "Box", "Cylinder", "Sphere", or "Compound"
    dimensions: List[float] = field(default_factory=list)
    pose: Pose = field(default_factory=Pose)
    material: Material = field(default_factory=Material)
    movable: bool = False  # If True, the MuJoCo body has a free joint
    include_in_controller_collision: bool = True  # Independent of mobility
    density: float = 1000.0  # Density for movable objects (kg/m^3), default 1000
    gravcomp: float = 0.0  # Gravity compensation: 0=none, 1=full compensation (disable gravity effect)

    parts: List["CollisionObject"] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'CollisionObject':
        """Create from YAML dict."""
        pose_data = data.get('pose', {})
        material_data = data.get('material', {})

        pose = Pose(
            position=tuple(pose_data.get('position', [0.0, 0.0, 0.0])),
            orientation=tuple(pose_data.get('orientation', [0.0, 0.0, 0.0, 1.0]))
        )

        material = Material()
        if 'friction' in material_data:
            material.friction = tuple(material_data['friction'])
        if 'rgba' in material_data:
            material.rgba = tuple(material_data['rgba'])

        parts = []
        for part_data in data.get('parts', []):
            # Inherit physical/render properties, but never mobility or pose.
            part_data = dict(part_data)
            part_data['material'] = {**material_data, **part_data.get('material', {})}
            part_data.setdefault('density', data.get('density', 1000.0))
            parts.append(cls.from_dict(part_data))

        return cls(
            id=data['id'],
            type=data['type'],
            dimensions=data.get('dimensions', []),
            pose=pose,
            material=material,
            movable=data.get('movable', False),
            include_in_controller_collision=data.get('include_in_controller_collision', True),
            density=data.get('density', 1000.0),
            gravcomp=data.get('gravcomp', 0.0),
            parts=parts
        )

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dict for YAML output."""
        result = {
            'id': self.id,
            'type': self.type,
            'dimensions': self.dimensions,
            'pose': {
                'position': list(self.pose.position),
                'orientation': list(self.pose.orientation)
            }
        }
        if self.type == 'Compound':
            result.pop('dimensions')
            result['parts'] = [part.to_dict() for part in self.parts]
        return result


@dataclass
class EnvConfig:
    """Environment configuration."""
    description: str = ""
    collision_objects: List[CollisionObject] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'EnvConfig':
        """Create from YAML dict."""
        objects = []
        for obj_data in data.get('collision_objects', []):
            objects.append(CollisionObject.from_dict(obj_data))

        return cls(
            description=data.get('description', ''),
            collision_objects=objects
        )


class EnvGenerator:
    """Generates MuJoCo XML, MPRC YAML, and cuRobo YAML from config."""

    def __init__(self, config: EnvConfig):
        self.config = config

    @staticmethod
    def _compose_pose(parent: Pose, local: Pose) -> Pose:
        """Compose poses: p_world = p_parent + R_parent p_local (xyzw)."""
        def multiply(a, b):
            ax, ay, az, aw = a
            bx, by, bz, bw = b
            return (aw*bx + ax*bw + ay*bz - az*by,
                    aw*by - ax*bz + ay*bw + az*bx,
                    aw*bz + ax*by - ay*bx + az*bw,
                    aw*bw - ax*bx - ay*by - az*bz)

        def normalized(q):
            norm = math.sqrt(sum(v*v for v in q))
            return tuple(v / norm for v in q)

        q = normalized(parent.orientation)
        rotated = multiply(multiply(q, (*local.position, 0.0)),
                           (-q[0], -q[1], -q[2], q[3]))
        return Pose(tuple(p + r for p, r in zip(parent.position, rotated[:3])),
                    normalized(multiply(q, normalized(local.orientation))))

    def _leaves(self, obj: CollisionObject, parent: Pose = None,
                prefix: str = '', controller_only: bool = False) -> List[CollisionObject]:
        """Flatten rigid parts into primitives in the requested parent frame."""
        if controller_only and not obj.include_in_controller_collision:
            return []
        pose = self._compose_pose(parent or Pose(), obj.pose)
        name = f'{prefix}__{obj.id}' if prefix else obj.id
        if obj.type != 'Compound':
            return [replace(obj, id=name, pose=pose, movable=False)]
        return [leaf for part in obj.parts
                for leaf in self._leaves(part, pose, name, controller_only)]

    def _controller_objects(self) -> List[CollisionObject]:
        return [leaf for obj in self.config.collision_objects
                for leaf in self._leaves(obj, controller_only=True)]

    @staticmethod
    def _xml_pose(pose: Pose) -> Dict[str, str]:
        qx, qy, qz, qw = pose.orientation
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        return {'pos': ' '.join(f'{v:.9g}' for v in pose.position),
                'quat': ' '.join(f'{v/norm:.9g}' for v in (qw, qx, qy, qz))}

    def _add_geom(self, parent, obj: CollisionObject, name: str):
        if obj.type == 'Box':
            size = [d / 2 for d in obj.dimensions]
        elif obj.type == 'Cylinder':
            size = [obj.dimensions[0], obj.dimensions[1] / 2]
        elif obj.type == 'Sphere':
            size = [obj.dimensions[0]]
        else:
            raise ValueError(f'Unknown primitive type: {obj.type}')
        ET.SubElement(parent, 'geom', {
            'name': name, 'type': obj.type.lower(),
            'size': ' '.join(f'{v:.9g}' for v in size),
            **self._xml_pose(obj.pose),
            'friction': ' '.join(str(v) for v in obj.material.friction),
            'density': str(obj.density),
            'solimp': '0.998 0.998 0.001', 'solref': '0.001 1',
            'rgba': ' '.join(str(v) for v in obj.material.rgba),
        })

    def generate_mujoco_xml(self) -> str:
        """Use one body for every compound, with at most one free joint."""
        root = ET.Element('mujocoinclude')
        world = ET.SubElement(root, 'worldbody')
        if self.config.description:
            description = str(self.config.description).replace('--', '- -')
            world.append(ET.Comment(f' {description} '))
        for obj in self.config.collision_objects:
            if obj.type == 'Compound' or obj.movable:
                attrs = {'name': obj.id, **self._xml_pose(obj.pose)}
                if obj.gravcomp:
                    attrs['gravcomp'] = str(obj.gravcomp)
                body = ET.SubElement(world, 'body', attrs)
                if obj.movable:
                    ET.SubElement(body, 'freejoint', {'name': f'{obj.id}_joint'})
                # The body's pose already accounts for the root transform.
                for leaf in self._leaves(replace(obj, pose=Pose())):
                    self._add_geom(body, leaf, f'{leaf.id}_geom')
            else:
                self._add_geom(world, obj, obj.id)
        ET.indent(root, space='  ')
        return ET.tostring(root, encoding='unicode')

    def generate_mprc_yaml(self) -> str:
        """Generate MPRC YAML content (selected by include_in_controller_collision)."""
        # Export selected objects at their configured world poses, regardless of mobility
        controller_objects = self._controller_objects()

        lines = [
            '# Auto-generated by env_generator.py',
        ]

        if self.config.description:
            lines.append(f'# {self.config.description}')

        lines.append(f'# Total controller collision geometries: {len(controller_objects)} (selected by include_in_controller_collision)')
        lines.append('')
        lines.append('collision_objects:' if controller_objects else 'collision_objects: []')
        lines.append('')

        for obj in controller_objects:
            lines.extend(self._mprc_object(obj))
            lines.append('')

        return '\n'.join(lines)

    def _mprc_object(self, obj: CollisionObject) -> List[str]:
        """Generate MPRC YAML entry for an object."""
        x, y, z = obj.pose.position
        qx, qy, qz, qw = obj.pose.orientation

        lines = [
            f'  - id: {yaml.safe_dump(obj.id, default_style=chr(34)).strip()}',
            f'    type: "{obj.type}"',
            f'    dimensions: {obj.dimensions}',
        ]

        # Format pose
        pos_str = f"      position: [{x:.9g}, {y:.9g}, {z:.9g}]"
        ori_str = f"      orientation: [{qx:.9g}, {qy:.9g}, {qz:.9g}, {qw:.9g}]"

        lines.append('    pose:')
        lines.append(pos_str)
        lines.append(ori_str)

        return lines

    def generate_curobo_yaml(self) -> str:
        """Generate cuRobo Scene YAML content (selected by include_in_controller_collision).

        Format:
            cuboid:
              object_name:
                dims: [x, y, z]
                pose: [x, y, z, qw, qx, qy, qz]
        """
        # Export selected objects at their configured world poses, regardless of mobility
        controller_objects = self._controller_objects()

        lines = [
            '## SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.',
            '## SPDX-License-Identifier: Apache-2.0',
            '##',
            '## Auto-generated by env_generator.py',
            '##',
        ]

        if self.config.description:
            lines.append(f'## {self.config.description}')
            lines.append('##')

        lines.append(f'## Total controller collision geometries: {len(controller_objects)} (selected by include_in_controller_collision)')
        lines.append('')
        lines.append('cuboid:' if controller_objects else 'cuboid: {}')
        lines.append('')

        for obj in controller_objects:
            lines.extend(self._curobo_object(obj))
            lines.append('')

        return '\n'.join(lines)

    def _curobo_object(self, obj: CollisionObject) -> List[str]:
        """Generate cuRobo YAML entry for an object.

        cuRobo format uses wxyz quaternion order and flattened pose array.
        """
        x, y, z = obj.pose.position
        qx, qy, qz, qw = obj.pose.orientation

        lines = [
            f'  ## {obj.id}: {obj.type}, dims={obj.dimensions}',
            f'  {yaml.safe_dump(obj.id, default_style=chr(34)).strip()}:',
        ]

        # Dimensions - cuRobo uses 'dims' key
        if obj.type == "Box":
            dims_str = f"    dims: {obj.dimensions}  # x, y, z dimensions"
        elif obj.type == "Cylinder":
            # Conservative bounding box in the primitive local frame
            r, h = obj.dimensions
            dims_str = f"    dims: {[2*r, 2*r, h]}  # enclosing cuboid for cylinder"
        elif obj.type == "Sphere":
            # Conservative bounding box in the primitive local frame
            r = obj.dimensions[0]
            dims_str = f"    dims: {[2*r, 2*r, 2*r]}  # enclosing cuboid for sphere"
        else:
            dims_str = f"    dims: {obj.dimensions}"

        lines.append(dims_str)

        # Pose - cuRobo uses [x, y, z, qw, qx, qy, qz] format
        pose_str = f"    pose: [{x:.9g}, {y:.9g}, {z:.9g}, {qw:.9g}, {qx:.9g}, {qy:.9g}, {qz:.9g}]"
        lines.append(pose_str)

        return lines

    def validate(self) -> Tuple[bool, List[str]]:
        """Validate configuration."""
        errors = []

        def finite_vector(values, length):
            return (isinstance(values, (list, tuple)) and len(values) == length
                    and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                            and math.isfinite(v) for v in values))

        def check(obj, path, child=False):
            if not isinstance(obj.id, str) or not obj.id.strip():
                errors.append(f'{path}: id must be a non-empty string')
            if not finite_vector(obj.pose.position, 3):
                errors.append(f'{path}: position requires 3 finite numbers')
            if not finite_vector(obj.pose.orientation, 4):
                errors.append(f'{path}: orientation requires 4 finite numbers [x,y,z,w]')
            elif not math.isclose(sum(v*v for v in obj.pose.orientation), 1.0, abs_tol=1e-3):
                errors.append(f'{path}: orientation must be a unit quaternion')
            if child and (obj.movable or obj.gravcomp != 0):
                errors.append(f'{path}: movable/gravcomp must be set on the root object only')
            if not isinstance(obj.include_in_controller_collision, bool):
                errors.append(f'{path}: include_in_controller_collision must be a boolean')
            if not isinstance(obj.movable, bool):
                errors.append(f'{path}: movable must be a boolean')
            if not finite_vector([obj.density], 1) or obj.density <= 0:
                errors.append(f'{path}: density must be finite and positive')
            if not finite_vector([obj.gravcomp], 1):
                errors.append(f'{path}: gravcomp must be finite')
            for key, length in [('friction', 3), ('rgba', 4)]:
                if not finite_vector(getattr(obj.material, key), length):
                    errors.append(f'{path}: {key} requires {length} finite numbers')
            if obj.type == 'Compound':
                if not obj.parts:
                    errors.append(f'{path}: Compound requires non-empty parts')
                if obj.dimensions:
                    errors.append(f'{path}: Compound uses parts, not dimensions')
                check_siblings(obj.parts, path)
            else:
                lengths = {'Box': 3, 'Cylinder': 2, 'Sphere': 1}
                if obj.type not in lengths:
                    errors.append(f'{path}: Unknown type {obj.type!r}')
                elif not finite_vector(obj.dimensions, lengths[obj.type]):
                    errors.append(f'{path}: {obj.type} requires {lengths[obj.type]} finite dimensions')
                elif any(d <= 0 for d in obj.dimensions):
                    errors.append(f'{path}: dimensions must be positive')
                if obj.parts:
                    errors.append(f'{path}: only Compound may contain parts')

        def check_siblings(objects, prefix=''):
            names = set()
            for obj in objects:
                path = f'{prefix}/{obj.id}'
                if str(obj.id) in names:
                    errors.append(f'{path}: duplicate id')
                names.add(str(obj.id))
                check(obj, path, child=bool(prefix))

        check_siblings(self.config.collision_objects)
        if not errors:
            # Also catch collisions introduced by flattening or the _geom suffix.
            leaf_names, geom_names = set(), set()
            for obj in self.config.collision_objects:
                for leaf in self._leaves(obj):
                    geom_name = (f'{leaf.id}_geom'
                                 if obj.type == 'Compound' or obj.movable else leaf.id)
                    if leaf.id in leaf_names or geom_name in geom_names:
                        errors.append(f'{leaf.id}: generated name collision')
                    leaf_names.add(leaf.id)
                    geom_names.add(geom_name)

        return len(errors) == 0, errors


def load_config(config_path: Path) -> EnvConfig:
    """Load environment config from YAML file."""
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f) or {}

    # Ensure collision_objects exists
    if 'collision_objects' not in data:
        data['collision_objects'] = []

    return EnvConfig.from_dict(data)


def main():
    parser = argparse.ArgumentParser(
        description='Generate MuJoCo XML, MPRC YAML, and cuRobo Scene YAML from config files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate with default paths
  python3 env_generator.py --config env_config/my_env.yaml

  # Custom output paths
  python3 env_generator.py --config env_config/my_env.yaml \\
      --xml-output /path/to/objects.xml \\
      --yaml-output /path/to/collision_env_sim.yaml \\
      --curobo-output /path/to/my_task_scene.yml

  # Preview without writing
  python3 env_generator.py --config env_config/my_env.yaml --dry-run
        """
    )

    parser.add_argument('--config', type=str, required=True,
                        help='Input config YAML file (from env_config/)')
    parser.add_argument('--xml-output', type=str, default=None,
                        help='Output path for MuJoCo XML file')
    parser.add_argument('--yaml-output', type=str, default=None,
                        help='Output path for MPRC YAML file')
    parser.add_argument('--curobo-output', type=str, default=None,
                        help='Output path for cuRobo Scene YAML file')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print configuration without writing files')

    args = parser.parse_args()

    # Resolve config path - try relative to script directory if not found
    config_path = Path(args.config)
    if not config_path.exists():
        # Try relative to script directory
        script_dir = Path(__file__).parent
        config_path = script_dir / args.config
        if not config_path.exists():
            # Try relative to workspace root (src/multipanda_ros2/env_config/)
            workspace_root = script_dir.parent.parent
            config_path = workspace_root / 'env_config' / Path(args.config).name

    if not config_path.exists():
        print(f"Error: Config file not found: {args.config}")
        print(f"Tried paths:")
        print(f"  - {Path(args.config).absolute()}")
        script_dir = Path(__file__).parent
        print(f"  - {script_dir / args.config}")
        print(f"  - {script_dir.parent.parent / 'env_config' / Path(args.config).name}")
        return 1

    # Load config
    print(f"Loading config from: {config_path}")
    try:
        env_config = load_config(config_path)
    except Exception as e:
        print(f"Error loading config: {e}")
        return 1

    if env_config.description:
        print(f"Description: {env_config.description}")
    print(f"Objects: {len(env_config.collision_objects)}")

    # Generate
    generator = EnvGenerator(env_config)

    # Validate
    valid, errors = generator.validate()
    if not valid:
        print("\nValidation errors:")
        for error in errors:
            print(f"  - {error}")
        return 1

    print("\nValidation passed!")

    # Generate outputs
    xml_content = generator.generate_mujoco_xml()
    yaml_content = generator.generate_mprc_yaml()
    curobo_content = generator.generate_curobo_yaml()

    # Default output paths
    if args.xml_output is None:
        script_dir = Path(__file__).parent
        args.xml_output = str(script_dir.parent / 'franka_description' / 'mujoco' / 'franka' / 'objects.xml')

    if args.yaml_output is None:
        script_dir = Path(__file__).parent
        # Go to src/dualarm_mprc/
        dualarm_mprc_dir = script_dir.parent.parent / 'dualarm_mprc'
        args.yaml_output = str(dualarm_mprc_dir / 'dualarm_reactive_control' / 'config' / 'collision_env_my_task.yaml')

    if args.curobo_output is None:
        script_dir = Path(__file__).parent
        # Go to src/curobo/curobo/content/configs/scene/
        curobo_dir = script_dir.parent.parent / 'curobo' / 'curobo' / 'content' / 'configs' / 'scene'
        args.curobo_output = str(curobo_dir / 'my_task_scene.yml')

    if args.dry_run:
        print("\n" + "=" * 60)
        print("DRY RUN - MuJoCo XML Preview")
        print("=" * 60)
        print(xml_content)

        print("\n" + "=" * 60)
        print("DRY RUN - MPRC YAML Preview")
        print("=" * 60)
        print(yaml_content)

        print("\n" + "=" * 60)
        print("DRY RUN - cuRobo Scene YAML Preview")
        print("=" * 60)
        print(curobo_content)
    else:
        # Write files
        xml_path = Path(args.xml_output)
        yaml_path = Path(args.yaml_output)
        curobo_path = Path(args.curobo_output)

        xml_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        curobo_path.parent.mkdir(parents=True, exist_ok=True)

        xml_path.write_text(xml_content)
        yaml_path.write_text(yaml_content)
        curobo_path.write_text(curobo_content)

        print("\n" + "=" * 60)
        print("Files written successfully:")
        print("=" * 60)
        print(f"  MuJoCo XML:   {xml_path}")
        print(f"  MPRC YAML:    {yaml_path}")
        print(f"  cuRobo Scene: {curobo_path}")
        print()
        print("Next steps:")
        print("  1. Build: docker exec -it multipanda-container bash")
        print("           cd /home/xiaozy24/dual_panda_ws")
        print("           colcon build --packages-select franka_description")
        print("  2. Launch: ros2 launch my_task_description my_task_sim.launch.py use_rviz:=true")

    return 0


if __name__ == '__main__':
    sys.exit(main())
