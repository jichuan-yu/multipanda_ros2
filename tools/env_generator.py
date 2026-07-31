#!/usr/bin/env python3
"""
Flexible Environment Generator for Dual Panda Simulation

Generates MuJoCo XML, MPRC YAML, and cuRobo Scene YAML from user-defined config files.
Supports Box, Cylinder, and Sphere shapes.

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
from dataclasses import dataclass, field
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
    type: str  # "Box", "Cylinder", or "Sphere"
    dimensions: List[float]
    pose: Pose = field(default_factory=Pose)
    material: Material = field(default_factory=Material)
    movable: bool = False  # If True, object can be moved (only in XML, not in MPRC YAML)
    density: float = 1000.0  # Density for movable objects (kg/m^3), default 1000

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

        return cls(
            id=data['id'],
            type=data['type'],
            dimensions=data['dimensions'],
            pose=pose,
            material=material,
            movable=data.get('movable', False),
            density=data.get('density', 1000.0)
        )

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dict for YAML output."""
        return {
            'id': self.id,
            'type': self.type,
            'dimensions': self.dimensions,
            'pose': {
                'position': list(self.pose.position),
                'orientation': list(self.pose.orientation)
            }
        }


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
    """Generates MuJoCo XML and MPRC YAML from config."""

    def __init__(self, config: EnvConfig):
        self.config = config

    def generate_mujoco_xml(self) -> str:
        """Generate MuJoCo XML content."""
        lines = [
            '<mujocoinclude>',
            '  <worldbody>',
            '',
        ]

        if self.config.description:
            lines.append(f'    <!-- {self.config.description} -->')
            lines.append('')

        for obj in self.config.collision_objects:
            lines.extend(self._mujoco_geom(obj))
            lines.append('')

        lines.append('  </worldbody>')
        lines.append('</mujocoinclude>')

        return '\n'.join(lines)

    def _mujoco_geom(self, obj: CollisionObject) -> List[str]:
        """Generate MuJoCo <geom> element for an object."""
        x, y, z = obj.pose.position
        qx, qy, qz, qw = obj.pose.orientation

        if obj.type == "Box":
            # size = [half_length, half_width, half_height]
            l, w, h = obj.dimensions
            size = f"{l/2:.4f} {w/2:.4f} {h/2:.4f}"
            geom_type = "box"
            # Box position is center
            pos = f"{x:.4f} {y:.4f} {z:.4f}"

        elif obj.type == "Cylinder":
            r, h = obj.dimensions
            size = f"{r:.4f} {h/2:.4f}"
            geom_type = "cylinder"
            # Cylinder position is center
            pos = f"{x:.4f} {y:.4f} {z:.4f}"

        elif obj.type == "Sphere":
            r = obj.dimensions[0]
            size = f"{r:.4f} {r:.4f} {r:.4f}"
            geom_type = "sphere"
            pos = f"{x:.4f} {y:.4f} {z:.4f}"

        else:
            raise ValueError(f"Unknown object type: {obj.type}")

        if obj.movable:
            # Movable object: wrap in <body> with free joint, geom at local origin
            body_pos = f'pos="{pos}"'
            body_quat = f'quat="{qx:.4f} {qy:.4f} {qz:.4f} {qw:.4f}"' if obj.pose.orientation != (0.0, 0.0, 0.0, 1.0) else ''

            lines = [
                f'    <!-- {obj.id}: {obj.type}, dims={obj.dimensions} [MOVABLE] -->',
                f'    <body name="{obj.id}" {body_pos} {body_quat}>',
                f'      <freejoint name="{obj.id}_joint"/>',
                f'      <geom name="{obj.id}_geom" type="{geom_type}" size="{size}"',
            ]

            # Add material properties
            lines[-1] += f' friction="{" ".join(str(v) for v in obj.material.friction)}"'
            lines[-1] += f' density="{obj.density}"'
            lines[-1] += f' solimp="0.998 0.998 0.001" solref="0.001 1"'
            lines[-1] += f' rgba="{" ".join(str(v) for v in obj.material.rgba)}"'
            lines[-1] += '/>'
            lines.append('    </body>')

        else:
            # Static object: simple geom at world position
            lines = [
                f'    <!-- {obj.id}: {obj.type}, dims={obj.dimensions} -->',
                f'    <geom name="{obj.id}" type="{geom_type}" size="{size}" '
                f'pos="{pos}"'
            ]

            # Add orientation if not default identity
            if obj.pose.orientation != (0.0, 0.0, 0.0, 1.0):
                lines[-1] += f' quat="{qx:.4f} {qy:.4f} {qz:.4f} {qw:.4f}"'

            lines.append(f'          friction="{" ".join(str(v) for v in obj.material.friction)}" '
                        f'solimp="0.998 0.998 0.001" solref="0.001 1"')
            lines.append(f'          rgba="{" ".join(str(v) for v in obj.material.rgba)}"/>')

        return lines

    def generate_mprc_yaml(self) -> str:
        """Generate MPRC YAML content (excludes movable objects)."""
        # Filter out movable objects - they are not static obstacles
        static_objects = [obj for obj in self.config.collision_objects if not obj.movable]

        lines = [
            '# Auto-generated by env_generator.py',
        ]

        if self.config.description:
            lines.append(f'# {self.config.description}')

        lines.append(f'# Total static objects: {len(static_objects)} (movable objects excluded)')
        lines.append('')
        lines.append('collision_objects:')
        lines.append('')

        for obj in static_objects:
            lines.extend(self._mprc_object(obj))
            lines.append('')

        return '\n'.join(lines)

    def _mprc_object(self, obj: CollisionObject) -> List[str]:
        """Generate MPRC YAML entry for an object."""
        x, y, z = obj.pose.position
        qx, qy, qz, qw = obj.pose.orientation

        lines = [
            f'  - id: "{obj.id}"',
            f'    type: "{obj.type}"',
            f'    dimensions: {obj.dimensions}',
        ]

        # Format pose
        pos_str = f"      position: [{x:.3f}, {y:.3f}, {z:.3f}]"
        ori_str = f"      orientation: [{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]"

        lines.append('    pose:')
        lines.append(pos_str)
        lines.append(ori_str)

        return lines

    def generate_curobo_yaml(self) -> str:
        """Generate cuRobo Scene YAML content (excludes movable objects).

        Format:
            cuboid:
              object_name:
                dims: [x, y, z]
                pose: [x, y, z, qw, qx, qy, qz]
        """
        # Filter out movable objects - they are not static obstacles
        static_objects = [obj for obj in self.config.collision_objects if not obj.movable]

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

        lines.append(f'## Total static objects: {len(static_objects)} (movable objects excluded)')
        lines.append('')
        lines.append('cuboid:')
        lines.append('')

        for obj in static_objects:
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
            f'  {obj.id}:',
        ]

        # Dimensions - cuRobo uses 'dims' key
        if obj.type == "Box":
            dims_str = f"    dims: {obj.dimensions}  # x, y, z dimensions"
        elif obj.type == "Cylinder":
            # For cylinder, cuRobo may use cuboid approximation or different format
            dims_str = f"    dims: {obj.dimensions}  # radius, height"
        elif obj.type == "Sphere":
            # For sphere, cuRobo may use cuboid approximation or different format
            dims_str = f"    dims: {obj.dimensions}  # radius"
        else:
            dims_str = f"    dims: {obj.dimensions}"

        lines.append(dims_str)

        # Pose - cuRobo uses [x, y, z, qw, qx, qy, qz] format
        pose_str = f"    pose: [{x:.3f}, {y:.3f}, {z:.3f}, {qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f}]"
        lines.append(pose_str)

        return lines

    def validate(self) -> Tuple[bool, List[str]]:
        """Validate configuration."""
        errors = []

        for obj in self.config.collision_objects:
            # Check type
            if obj.type not in ["Box", "Cylinder", "Sphere"]:
                errors.append(f"{obj.id}: Unknown type '{obj.type}'")

            # Check dimensions
            if obj.type == "Box" and len(obj.dimensions) != 3:
                errors.append(f"{obj.id}: Box requires 3 dimensions [length, width, height]")
            elif obj.type == "Cylinder" and len(obj.dimensions) != 2:
                errors.append(f"{obj.id}: Cylinder requires 2 dimensions [radius, height]")
            elif obj.type == "Sphere" and len(obj.dimensions) != 1:
                errors.append(f"{obj.id}: Sphere requires 1 dimension [radius]")

            # Check for positive dimensions
            for i, d in enumerate(obj.dimensions):
                if d <= 0:
                    errors.append(f"{obj.id}: Dimension {i} must be positive, got {d}")

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
