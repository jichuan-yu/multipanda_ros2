#!/usr/bin/env python3
"""Generate test CSV for dual-panda teleoperation."""

import csv

# ============ Hardcoded Configuration ============
TIMESTAMP_STEP = 0.01          # Time step between rows
FORWARD_ROWS = 50             # Number of rows for forward motion
STOP_ROWS = 50                 # Number of rows for stop phase
REVERSE_ROWS = 50             # Number of rows for reverse motion

# Initial pose values (from test_simple_safe.csv)
INIT_X_LEFT = 0.307
INIT_Y_LEFT = 0.260
INIT_Z_LEFT = 0.487
INIT_QX_LEFT = 1.000
INIT_QY_LEFT = 0.000
INIT_QZ_LEFT = 0.000
INIT_QW_LEFT = 0.000

INIT_X_RIGHT = 0.307
INIT_Y_RIGHT = -0.260
INIT_Z_RIGHT = 0.487
INIT_QX_RIGHT = 1.000
INIT_QY_RIGHT = 0.000
INIT_QZ_RIGHT = 0.000
INIT_QW_RIGHT = 0.000

# Motion parameters
X_STEP_PER_ROW = 0.001         # How much x changes per row during motion
OUTPUT_FILE = "test_generated.csv"
# =================================================


def generate_row(timestamp, x_left, x_right):
    """Generate a single CSV row with fixed orientation."""
    return {
        'timestamp': f'{timestamp:.3f}',
        'x_left': f'{x_left:.3f}',
        'y_left': f'{INIT_Y_LEFT:.3f}',
        'z_left': f'{INIT_Z_LEFT:.3f}',
        'qx_left': f'{INIT_QX_LEFT:.3f}',
        'qy_left': f'{INIT_QY_LEFT:.3f}',
        'qz_left': f'{INIT_QZ_LEFT:.3f}',
        'qw_left': f'{INIT_QW_LEFT:.3f}',
        'x_right': f'{x_right:.3f}',
        'y_right': f'{INIT_Y_RIGHT:.3f}',
        'z_right': f'{INIT_Z_RIGHT:.3f}',
        'qx_right': f'{INIT_QX_RIGHT:.3f}',
        'qy_right': f'{INIT_QY_RIGHT:.3f}',
        'qz_right': f'{INIT_QZ_RIGHT:.3f}',
        'qw_right': f'{INIT_QW_RIGHT:.3f}',
    }


def main():
    rows = []
    timestamp = 0.0

    # Phase 1: Forward motion (x increases)
    current_x_left = INIT_X_LEFT
    current_x_right = INIT_X_RIGHT
    for _ in range(FORWARD_ROWS):
        rows.append(generate_row(timestamp, current_x_left, current_x_right))
        timestamp += TIMESTAMP_STEP
        current_x_left += X_STEP_PER_ROW
        current_x_right += X_STEP_PER_ROW

    # Phase 2: Stop (x remains constant)
    for _ in range(STOP_ROWS):
        rows.append(generate_row(timestamp, current_x_left, current_x_right))
        timestamp += TIMESTAMP_STEP

    # Phase 3: Reverse motion (x decreases)
    for _ in range(REVERSE_ROWS):
        rows.append(generate_row(timestamp, current_x_left, current_x_right))
        timestamp += TIMESTAMP_STEP
        current_x_left -= X_STEP_PER_ROW
        current_x_right -= X_STEP_PER_ROW

    # Write to CSV
    fieldnames = [
        'timestamp', 'x_left', 'y_left', 'z_left', 'qx_left', 'qy_left', 'qz_left', 'qw_left',
        'x_right', 'y_right', 'z_right', 'qx_right', 'qy_right', 'qz_right', 'qw_right'
    ]

    with open(OUTPUT_FILE, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Generated {len(rows)} rows to {OUTPUT_FILE}")
    print(f"  Phase 1 (forward): {FORWARD_ROWS} rows")
    print(f"  Phase 2 (stop): {STOP_ROWS} rows")
    print(f"  Phase 3 (reverse): {REVERSE_ROWS} rows")
    print(f"  Total time: {timestamp:.3f}s")


if __name__ == '__main__':
    main()
