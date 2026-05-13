#!/usr/bin/env python3
import argparse
import signal
import sys
import time

import numpy as np
from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig


def parse_args():
    parser = argparse.ArgumentParser(description='Print current Gello joint angles at 5 Hz.')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for the Gello device.')
    parser.add_argument('--rate', type=float, default=5.0, help='Print frequency in Hz.')
    parser.add_argument('--offsets', nargs=7, type=float, default=[
        1.0*np.pi,
        1.0*np.pi,
        0 * np.pi,
        np.pi,
        np.pi,
        np.pi,
        0.25* np.pi,
    ], help='Calibration joint offsets applied to raw Gello joint values.')
    parser.add_argument('--signs', nargs=7, type=float, default=[1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0], help='Calibration joint signs applied to raw Gello joint values.')
    return parser.parse_args()


def build_agent(port, offsets, signs):
    dynamixel_config = DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        joint_offsets=tuple(offsets),
        joint_signs=tuple(signs),
        gripper_config=None,
    )
    return GelloAgent(port=port, dynamixel_config=dynamixel_config, real=True)


def format_joint_list(name, joints):
    return name + ': [' + ', '.join(f'{float(v):.4f}' for v in joints) + ']'


def main():
    args = parse_args()
    if args.rate <= 0.0:
        raise ValueError('Rate must be greater than 0.')

    try:
        agent = build_agent(args.port, args.offsets, args.signs)
    except Exception as exc:
        print(f'Failed to initialize Gello Agent: {exc}', file=sys.stderr)
        return 1

    stop = False

    def handle_signal(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    period = 1.0 / args.rate
    print(f'Starting Gello joint monitor at {args.rate:.1f} Hz on port {args.port}')
    print('Use Ctrl+C to stop.')

    try:
        while not stop:
            start = time.time()
            joints = agent.act({})
            if joints is None or len(joints) < 7:
                print('Warning: failed to read 7 joint values from Gello')
            else:
                raw_joints = np.asarray(joints[:7], dtype=float)
                print(f'[{time.strftime("%Y-%m-%d %H:%M:%S")}]')
                print(format_joint_list('raw_gello', raw_joints))
                print('-' * 72)

            elapsed = time.time() - start
            time.sleep(max(0.0, period - elapsed))
    except Exception as exc:
        print(f'Error while monitoring Gello joints: {exc}', file=sys.stderr)
    finally:
        try:
            if hasattr(agent, '_robot'):
                agent._robot.set_torque_mode(False)
        except Exception:
            pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
