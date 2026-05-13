#!/usr/bin/env python3
"""
Random Static Obstacle Generator for Dual Panda Simulation

Generates random non-overlapping pillar configurations for MuJoCo simulation
and MPRC collision detection.

Usage:
    python3 generate_collision_env.py [OPTIONS]

Options:
    --seed N              Random seed for reproducibility (default: random)
    --small-pillars N     Number of small pillars (0.05x0.05m) (default: 10)
    --large-pillars N     Number of large pillars (0.1x0.1m) (default: 2)
    --margin M            Safety margin in meters (default: 0.01)
    --xml-output PATH     Output path for MuJoCo XML file
    --yaml-output PATH    Output path for MPRC YAML file
    --dry-run             Print configuration without writing files
"""

import argparse
import random
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


@dataclass
class Pillar:
    """Represents a single pillar obstacle."""
    index: int
    x: float  # Center X position (meters)
    y: float  # Center Y position (meters)
    length: float  # Length along X (meters)
    width: float  # Width along Y (meters)
    height: float  # Height along Z (meters)

    @property
    def name(self) -> str:
        return f"pillar_{self.index:02d}"

    @property
    def half_length(self) -> float:
        return self.length / 2

    @property
    def half_width(self) -> float:
        return self.width / 2

    @property
    def half_height(self) -> float:
        return self.height / 2

    @property
    def z_position(self) -> float:
        """Z position for center of pillar (height/2 above ground)."""
        return self.half_height


class CollisionEnvGenerator:
    """Generates random non-overlapping pillar configurations."""

    # Position range for pillar centers
    X_MIN = 0.35
    X_MAX = 1.0
    Y_MIN = -0.4
    Y_MAX = 0.4

    # Height options for small pillars (multiples of 0.1)
    SMALL_HEIGHTS = [0.3, 0.4, 0.5, 0.6]

    # Small pillar dimensions
    SMALL_SIZE = 0.05

    # Large pillar dimensions
    LARGE_SIZE = 0.1
    LARGE_HEIGHT = 0.2

    def __init__(self, seed: int = None, margin: float = 0.01, max_retries: int = 1000):
        """
        Initialize generator.

        Args:
            seed: Random seed (None for random)
            margin: Safety margin between pillars (meters)
            max_retries: Maximum placement attempts per pillar
        """
        self.seed = seed
        self.margin = margin
        self.max_retries = max_retries

        if seed is not None:
            random.seed(seed)
            print(f"Using random seed: {seed}")
        else:
            print("Using random seed (unspecified)")

    def boxes_overlap(self, p1: Pillar, p2: Pillar) -> bool:
        """
        Check if two pillars overlap (including safety margin).

        Args:
            p1: First pillar
            p2: Second pillar

        Returns:
            True if pillars overlap
        """
        dx = abs(p1.x - p2.x)
        dy = abs(p1.y - p2.y)

        min_dist_x = (p1.length + p2.length) / 2 + self.margin
        min_dist_y = (p1.width + p2.width) / 2 + self.margin

        return dx < min_dist_x and dy < min_dist_y

    def check_overlap_with_existing(self, pillar: Pillar, existing: List[Pillar]) -> bool:
        """Check if pillar overlaps with any existing pillars."""
        for existing_pillar in existing:
            if self.boxes_overlap(pillar, existing_pillar):
                return True
        return False

    def try_place_pillar(self, length: float, width: float, height: float,
                         existing: List[Pillar]) -> Pillar:
        """
        Try to place a pillar at a random non-overlapping position.

        Args:
            length: Pillar length
            width: Pillar width
            height: Pillar height
            existing: List of already placed pillars

        Returns:
            Placed pillar, or None if placement failed
        """
        for _ in range(self.max_retries):
            # Generate random position and round to 0.01 precision
            x = round(random.uniform(self.X_MIN, self.X_MAX), 2)
            y = round(random.uniform(self.Y_MIN, self.Y_MAX), 2)

            pillar = Pillar(
                index=len(existing) + 1,
                x=x,
                y=y,
                length=length,
                width=width,
                height=height
            )

            if not self.check_overlap_with_existing(pillar, existing):
                return pillar

        return None

    def generate_pillars(self, num_small: int = 10, num_large: int = 2) -> List[Pillar]:
        """
        Generate a complete pillar configuration.

        Args:
            num_small: Number of small pillars (0.05x0.05m)
            num_large: Number of large pillars (0.1x0.1m, height=0.2m)

        Returns:
            List of generated pillars
        """
        pillars = []

        # Create pillar templates (size categories)
        templates = []
        for _ in range(num_small):
            height = random.choice(self.SMALL_HEIGHTS)
            templates.append((self.SMALL_SIZE, self.SMALL_SIZE, height, "small"))

        for _ in range(num_large):
            templates.append((self.LARGE_SIZE, self.LARGE_SIZE, self.LARGE_HEIGHT, "large"))

        # Shuffle for randomness
        random.shuffle(templates)

        # Place each pillar
        for length, width, height, size_type in templates:
            pillar = self.try_place_pillar(length, width, height, pillars)

            if pillar is None:
                print(f"Warning: Could not place {size_type} pillar after {self.max_retries} attempts")
                print("Consider increasing the position range or reducing the number of pillars")
                return None

            pillars.append(pillar)
            print(f"Placed {pillar.name}: X={pillar.x:.2f}, Y={pillar.y:.2f}, "
                  f"Size={length}x{width}, H={height}")

        # Sort by index for consistent output
        pillars.sort(key=lambda p: p.index)

        return pillars


def generate_mujoco_xml(pillars: List[Pillar]) -> str:
    """Generate MuJoCo XML content for pillars."""
    lines = [
        '<mujocoinclude>',
        '  <worldbody>',
    ]

    # Preserve old commented-out sections (reference material)
    lines.append('    <!-- <body name="obj_box_01" pos="0.5 -0.13 0.061">')
    lines.append('        <geom name="obj_box_01" type="box" size="0.03 0.03 0.03" '
                 'friction="2 0.005 0.0001" solimp="0.998 0.998 0.001" solref="0.001 1" '
                 'density="100" rgba="1 0.56 0.43 1"/>')
    lines.append('        <joint name="obj_box_01_joint" type="free" damping="0.0005"/>')
    lines.append('    </body>')
    lines.append('    <body name="sphere_01" pos="0.5 0 0.8" mocap="true">')
    lines.append('        <geom name="sphere_01" type="sphere" size="0.1" rgba="0 1 0 1"/>')
    lines.append('    </body> -->')

    lines.append('')
    lines.append('    <!-- Auto-generated static pillars -->')
    lines.append('    <!-- Format: X Y length width height -->')
    lines.append('    <!-- MuJoCo size = half-extents [length/2, width/2, height/2], pos Z = height/2 -->')
    lines.append('')

    for p in pillars:
        lines.append(f'    <!-- {p.name}: X={p.x:.2f}, Y={p.y:.2f}, L={p.length}, W={p.width}, H={p.height} -->')
        lines.append(f'    <geom name="{p.name}" type="box" '
                     f'size="{p.half_length} {p.half_width} {p.half_height}" '
                     f'pos="{p.x:.2f} {p.y:.2f} {p.z_position:.2f}"')
        lines.append(f'          friction="2 0.005 0.0001" solimp="0.998 0.998 0.001" solref="0.001 1"')
        lines.append(f'          rgba="0.2 0.4 0.8 1"/>')

    lines.append('')
    lines.append('  </worldbody>')
    lines.append('</mujocoinclude>')

    return '\n'.join(lines)


def generate_mprc_yaml(pillars: List[Pillar]) -> str:
    """Generate MPRC YAML content for pillars."""
    lines = [
        '# Collision environment configuration for my_task_sim.launch.py',
        f'# Auto-generated with {len(pillars)} static pillars',
        '# Matches the static pillars defined in franka_description/mujoco/franka/objects.xml',
        '',
        'collision_objects:',
        '  # Auto-generated static pillars',
        '  # Format: X Y length width height',
        '  # MPRC dimensions: full [length, width, height], position Z = height/2',
        '',
    ]

    for p in pillars:
        lines.append(f'  - id: "{p.name}"')
        lines.append(f'    type: "Box"')
        lines.append(f'    dimensions: [{p.length}, {p.width}, {p.height}]  # full length, width, height')
        lines.append(f'    pose:')
        lines.append(f'      position: [{p.x:.2f}, {p.y:.2f}, {p.z_position:.2f}]     # X, Y, height/2')
        lines.append(f'      orientation: [0.0, 0.0, 0.0, 1.0]')
        lines.append('')

    lines.append('  # Old dynamic spheres (kept for reference, now removed)')
    lines.append('  # - id: "dyn_sphere_1"')
    lines.append('  #   type: "Sphere"')
    lines.append('  #   dimensions: [0.05]')
    lines.append('  #   pose:')
    lines.append('  #     position: [0.2, 0.0, 0.6]')
    lines.append('  #     orientation: [0.0, 0.0, 0.0, 1.0]')

    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(
        description='Generate random static obstacle configurations for dual Panda simulation'
    )
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for reproducibility (default: random)')
    parser.add_argument('--small-pillars', type=int, default=10,
                        help='Number of small pillars 0.05x0.05m (default: 10)')
    parser.add_argument('--large-pillars', type=int, default=2,
                        help='Number of large pillars 0.1x0.1m (default: 2)')
    parser.add_argument('--margin', type=float, default=0.01,
                        help='Safety margin between pillars in meters (default: 0.01)')
    parser.add_argument('--xml-output', type=str, default=None,
                        help='Output path for MuJoCo XML file')
    parser.add_argument('--yaml-output', type=str, default=None,
                        help='Output path for MPRC YAML file')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print configuration without writing files')

    args = parser.parse_args()

    # Default output paths
    # Script is at: src/multipanda_ros2/tools/generate_collision_env.py
    if args.xml_output is None:
        # Go up to src/multipanda_ros2/, then to franka_description/
        multipanda_dir = Path(__file__).parent.parent
        args.xml_output = str(multipanda_dir / 'franka_description' / 'mujoco' / 'franka' / 'objects.xml')

    if args.yaml_output is None:
        # Go up to src/, then to dualarm_mprc/
        src_dir = Path(__file__).parent.parent.parent
        args.yaml_output = str(src_dir / 'dualarm_mprc' / 'dualarm_reactive_control' /
                               'config' / 'collision_env_my_task.yaml')

    print("=" * 60)
    print("Random Static Obstacle Generator")
    print("=" * 60)
    print(f"Small pillars: {args.small_pillars} (0.05x0.05m)")
    print(f"Large pillars: {args.large_pillars} (0.1x0.1m, H=0.2m)")
    print(f"Position range: X=[{CollisionEnvGenerator.X_MIN}, {CollisionEnvGenerator.X_MAX}], "
          f"Y=[{CollisionEnvGenerator.Y_MIN}, {CollisionEnvGenerator.Y_MAX}]")
    print(f"Safety margin: {args.margin}m")
    print("=" * 60)
    print()

    # Generate pillars
    generator = CollisionEnvGenerator(seed=args.seed, margin=args.margin)
    pillars = generator.generate_pillars(num_small=args.small_pillars, num_large=args.large_pillars)

    if pillars is None:
        print("\nError: Failed to generate valid configuration")
        print("Try:")
        print("  - Reducing the number of pillars")
        print("  - Increasing the safety margin with --margin")
        print("  - Using a different seed with --seed")
        return 1

    print()
    print(f"Successfully generated {len(pillars)} pillars")
    print()

    # Generate file contents
    xml_content = generate_mujoco_xml(pillars)
    yaml_content = generate_mprc_yaml(pillars)

    if args.dry_run:
        print("=" * 60)
        print("DRY RUN - Configuration Preview")
        print("=" * 60)
        print()
        print("MuJoCo XML (first 50 lines):")
        print("-" * 60)
        for i, line in enumerate(xml_content.split('\n')[:50]):
            print(line)
        print()
        print("MPRC YAML (first 30 lines):")
        print("-" * 60)
        for i, line in enumerate(yaml_content.split('\n')[:30]):
            print(line)
    else:
        # Write files
        xml_path = Path(args.xml_output)
        yaml_path = Path(args.yaml_output)

        xml_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)

        xml_path.write_text(xml_content)
        yaml_path.write_text(yaml_content)

        print("=" * 60)
        print("Files written successfully:")
        print("=" * 60)
        print(f"  MuJoCo XML: {xml_path}")
        print(f"  MPRC YAML:  {yaml_path}")
        print()
        print("Next steps:")
        print("  1. Build: docker exec -it multipanda-container bash")
        print("           cd /home/xiaozy24/dual_panda_ws")
        print("           colcon build --packages-select franka_description")
        print("  2. Launch: ros2 launch my_task_description my_task_sim.launch.py use_rviz:=true")

    return 0


if __name__ == '__main__':
    sys.exit(main())
