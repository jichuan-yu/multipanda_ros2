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
class Zone:
    """Represents a placement zone with X/Y ranges and pillar counts."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    num_small: int
    num_large: int
    name: str


@dataclass
class Pillar:
    """Represents a single pillar obstacle."""
    index: int
    x: float  # Center X position (meters)
    y: float  # Center Y position (meters)
    length: float  # Length along X (meters)
    width: float  # Width along Y (meters)
    height: float  # Height along Z (meters)
    pillar_type: int = 0  # 0 = rectangular (box), 1 = circular (cylinder)

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

    @property
    def radius(self) -> float:
        """For circular pillars: radius = min(length, width) / 2"""
        return min(self.length, self.width) / 2

    @property
    def type_name(self) -> str:
        """Human-readable type name."""
        return "cylinder" if self.pillar_type == 1 else "box"


class CollisionEnvGenerator:
    """Generates random non-overlapping pillar configurations."""

    # Height options for small pillars (multiples of 0.1)
    SMALL_HEIGHTS = [0.3, 0.4, 0.5, 0.6, 0.7]

    # Small pillar dimensions
    SMALL_SIZE = 0.05

    # Large pillar dimensions
    LARGE_SIZE = 0.1
    LARGE_HEIGHT = 0.2

    # Define zones with their X/Y ranges and pillar counts
    ZONES = [
        # Zone 1: Center near origin
        Zone(0.2, 0.4, -0.05, 0.05, 1, 0, "Zone1_center"),
        # Zone 2: Left rear
        Zone(0.2, 0.4, -0.45, -0.4, 1, 0, "Zone2_left_rear"),
        # Zone 3: Left front
        Zone(0.2, 0.4, 0.4, 0.45, 1, 0, "Zone3_left_front"),
        # Zone 4: Middle
        Zone(0.4, 0.6, -0.4, 0.4, 3, 1, "Zone4_middle"),
        # Zone 5: Right middle
        Zone(0.6, 0.75, -0.3, 0.3, 2, 1, "Zone5_right_middle"),
        # Zone 6: Right front
        Zone(0.75, 0.85, -0.2, 0.2, 2, 0, "Zone6_right_front"),
    ]

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
        Always uses rectangular footprint for overlap detection.

        Args:
            p1: First pillar
            p2: Second pillar

        Returns:
            True if pillars overlap
        """
        dx = abs(p1.x - p2.x)
        dy = abs(p1.y - p2.y)

        # Use original rectangle dimensions for overlap check
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
                         existing: List[Pillar], zone: Zone) -> Pillar:
        """
        Try to place a pillar at a random non-overlapping position within a zone.

        Args:
            length: Pillar length
            width: Pillar width
            height: Pillar height
            existing: List of already placed pillars
            zone: Zone to place pillar in

        Returns:
            Placed pillar, or None if placement failed
        """
        # Randomly assign pillar type: 0 (box) or 1 (cylinder) with equal probability
        pillar_type = random.randint(0, 1)

        for _ in range(self.max_retries):
            # Generate random position within zone bounds, rounded to 0.01
            x = round(random.uniform(zone.x_min, zone.x_max), 2)
            y = round(random.uniform(zone.y_min, zone.y_max), 2)

            pillar = Pillar(
                index=len(existing) + 1,
                x=x,
                y=y,
                length=length,
                width=width,
                height=height,
                pillar_type=pillar_type
            )

            if not self.check_overlap_with_existing(pillar, existing):
                return pillar

        return None

    def generate_pillars(self) -> List[Pillar]:
        """
        Generate a complete pillar configuration based on predefined zones.

        Returns:
            List of generated pillars, or None if placement failed
        """
        pillars = []
        pillar_index = 1

        for zone in self.ZONES:
            print(f"\nGenerating for {zone.name}: X=[{zone.x_min}, {zone.x_max}], "
                  f"Y=[{zone.y_min}, {zone.y_max}]")

            # Collect pillars to place in this zone
            zone_templates = []

            # Add small pillars for this zone
            for _ in range(zone.num_small):
                height = random.choice(self.SMALL_HEIGHTS)
                zone_templates.append((self.SMALL_SIZE, self.SMALL_SIZE, height, "small", zone))

            # Add large pillars for this zone
            for _ in range(zone.num_large):
                zone_templates.append((self.LARGE_SIZE, self.LARGE_SIZE, self.LARGE_HEIGHT, "large", zone))

            # Shuffle for randomness within zone
            random.shuffle(zone_templates)

            # Place each pillar in this zone
            for length, width, height, size_type, pillar_zone in zone_templates:
                pillar = self.try_place_pillar(length, width, height, pillars, pillar_zone)

                if pillar is None:
                    print(f"Warning: Could not place {size_type} pillar in {pillar_zone.name} "
                          f"after {self.max_retries} attempts")
                    return None

                pillar.index = pillar_index
                pillar_index += 1
                pillars.append(pillar)
                type_str = "CYLINDER" if pillar.pillar_type == 1 else "BOX"
                print(f"  Placed {pillar.name}: X={pillar.x:.2f}, Y={pillar.y:.2f}, "
                      f"Size={length}x{width}, H={height}, Type={type_str}, Zone={pillar_zone.name}")

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
    lines.append('    <!-- Type: 0=Box, 1=Cylinder -->')
    lines.append('    <!-- For cylinders: diameter = min(length, width), fromto defines height -->')
    lines.append('')

    for p in pillars:
        if p.pillar_type == 1:
            # Cylinder: use fromto to define vertical orientation
            # MuJoCo cylinder size is [radius, height*0.5]
            type_str = "CYLINDER"
            geom_type = "cylinder"
            size = f"{p.radius:.3f} {p.half_height:.3f}"
            # For cylinder, pos is center, same as box
        else:
            type_str = "BOX"
            geom_type = "box"
            size = f"{p.half_length:.3f} {p.half_width:.3f} {p.half_height:.3f}"

        lines.append(f'    <!-- {p.name}: X={p.x:.2f}, Y={p.y:.2f}, Type={type_str}, H={p.height} -->')
        lines.append(f'    <geom name="{p.name}" type="{geom_type}" '
                     f'size="{size}" '
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
        '  # Type: Box (rectangular) or Cylinder (circular)',
        '  # For Box: dimensions = [length, width, height]',
        '  # For Cylinder: dimensions = [diameter, height]',
        '',
    ]

    for p in pillars:
        if p.pillar_type == 1:
            # Cylinder: diameter = min(length, width)
            obj_type = "Cylinder"
            diameter = min(p.length, p.width) / 2 # MPRC expects diameter to be radius, so we divide by 2
            dimensions = f"[{diameter}, {p.height}]  # diameter, height"
        else:
            # Box
            obj_type = "Box"
            dimensions = f"[{p.length}, {p.width}, {p.height}]  # length, width, height"

        lines.append(f'  - id: "{p.name}"')
        lines.append(f'    type: "{obj_type}"')
        lines.append(f'    dimensions: {dimensions}')
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

    # Calculate totals from zones
    total_small = sum(z.num_small for z in CollisionEnvGenerator.ZONES)
    total_large = sum(z.num_large for z in CollisionEnvGenerator.ZONES)

    print("=" * 60)
    print("Random Static Obstacle Generator (Zone-based)")
    print("=" * 60)
    print(f"Total: {total_small} small (0.05x0.05m), {total_large} large (0.1x0.1m, H=0.2m)")
    print(f"Safety margin: {args.margin}m")
    print("Zones:")
    for z in CollisionEnvGenerator.ZONES:
        large_info = f", {z.num_large} LARGE" if z.num_large > 0 else ""
        print(f"  {z.name}: X=[{z.x_min}, {z.x_max}], Y=[{z.y_min}, {z.y_max}] "
              f"→ {z.num_small} SMALL{large_info}")
    print("=" * 60)
    print()

    # Generate pillars
    generator = CollisionEnvGenerator(seed=args.seed, margin=args.margin)
    pillars = generator.generate_pillars()

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
