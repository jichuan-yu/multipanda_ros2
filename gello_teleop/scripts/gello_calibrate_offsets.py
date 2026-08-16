#!/usr/bin/env python3
"""
GELLO 双臂标定脚本：求解 joint_offsets 并回写左右 YAML。

用法：先将待标定臂摆到『指定标准姿态』，再运行脚本读取电机原始角度，
计算 joint_offsets 并写回对应 YAML。

目标姿态(rad) —— 修正后该姿态下最终传入控制器的 7 个关节角：
    TARGET = [0, 0, 0, -0.5*pi, 0, 0.5*pi, 0.25*pi]

取整规则（对算出的 joint_offset 取整）：
    - 前 6 个关节：向最近的 0.5*pi 整数倍取整；
    - 第 7 个(末端)：向最近的 0.25*pi 整数倍取整。

数学依据（见 gello/robots/dynamixel.py DynamixelRobot.get_joint_state）：
    corrected_i = joint_signs[i] * (raw_i - joint_offsets[i])
    => joint_offsets[i] = raw_i - joint_signs[i] * TARGET[i] （再按规则取整）

左右臂分别写回 config/gello_arm1.yaml、config/gello_arm2.yaml。

用法：
    python3 gello_calibrate_offsets.py --arm left
    python3 gello_calibrate_offsets.py --arm right
    python3 gello_calibrate_offsets.py --arm both     # 先摆左标左，再摆右标右
    python3 gello_calibrate_offsets.py --arm left --auto-confirm   # 跳过回车确认
"""

import argparse
import re
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from gello.dynamixel.driver import DynamixelDriver

# ---- 标定目标与取整规则（rad）-------------------------------------------
# 目标姿态：标定时 GELLO 应摆成该姿态；经 offset+sign 修正后关节应等于这些值
TARGET = np.array([0.0, 0.0, 0.0, -0.5 * np.pi, 0.0, 0.5 * np.pi, 0.25 * np.pi])
# 各关节 joint_offset 的取整量子：前 6 为 0.5*pi，第 7(末端) 为 0.25*pi
QUANTUM = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.25]) * np.pi

# ---- 硬件参数 ------------------------------------------------------------
BAUDRATE = 57600
WARMUP = 10        # 读数预热次数
def round_to_multiple(value: float, quantum: float) -> float:
    """取最接近 quantum 整数倍的值。"""
    return float(np.round(float(value) / quantum) * quantum)


def load_gello_yaml(path: Path) -> dict:
    """加载 GELLO YAML，返回 cfg 及用到的关键字段。"""
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)
    dc = cfg['agent']['dynamixel_config']
    return {
        'cfg': cfg,
        'port': cfg['agent'].get('port'),
        'joint_ids': list(dc.get('joint_ids')),
        'joint_signs': list(dc.get('joint_signs')),
    }


def read_raw_joints(info: dict) -> np.ndarray:
    """通过 DynamixelDriver 读取当前 7 个电机原始角度（未修正）。"""
    if not info['port']:
        sys.exit('错误：配置中未设置 agent.port')
    driver = DynamixelDriver(info['joint_ids'], port=info['port'], baudrate=BAUDRATE)
    for _ in range(WARMUP):
        driver.get_joints()            # 总线预热
    time.sleep(0.05)
    raw = np.asarray(driver.get_joints(), dtype=float)
    return raw


def compute_offsets(raw: np.ndarray, signs: list) -> tuple:
    """计算 joint_offsets；返回 (offsets, corrected, residuals)。

    corrected_i = signs[i] * (raw_i - offsets[i])  应接近 TARGET_i。
    """
    offsets, corrected, residuals = [], [], []
    for i in range(7):
        off = round_to_multiple(raw[i] - signs[i] * TARGET[i], QUANTUM[i])
        corr = signs[i] * (raw[i] - off)
        offsets.append(off)
        corrected.append(corr)
        residuals.append(corr - TARGET[i])
    return np.asarray(offsets), np.asarray(corrected), np.asarray(residuals)

# ---- 默认配置文件 ---------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = {
    'left': SCRIPT_DIR.parent / 'config' / 'gello_arm1.yaml',
    'right': SCRIPT_DIR.parent / 'config' / 'gello_arm2.yaml',
}
ARM_NAME = {'left': '左臂(gello_arm1.yaml)', 'right': '右臂(gello_arm2.yaml)'}


def fmt_offset(value: float) -> str:
    """格式化为配置风格（5 位小数），避免 -0.00000。"""
    v = 0.0 if abs(float(value)) < 5e-6 else float(value)
    return f'{v:.5f}'


def write_offsets(cfg: dict, path: Path, offsets: np.ndarray) -> Path:
    """仅替换 YAML 的 joint_offsets 行数值，保持原文件其余格式不变。（cfg 仅为兼容）"""
    text = path.read_text()
    new_list = '[' + ', '.join(fmt_offset(v) for v in offsets) + ']'
    pattern = re.compile(r'(joint_offsets:\s*)(\[[^\]]*\])')
    new_text, n = pattern.subn(lambda m: m.group(1) + new_list, text, count=1)
    if n != 1:
        raise RuntimeError(
            f'未在 {path} 中找到 flow 形式的 joint_offsets 列表，未做任何修改')
    path.write_text(new_text)
    return path


def calibrate_arm(arm: str, config_path: Path, auto_confirm: bool) -> None:
    info = load_gello_yaml(config_path)

    print('\n' + '=' * 72)
    print(f'开始标定：{ARM_NAME[arm]}')
    print(f'  配置文件   : {config_path}')
    print(f'  串口       : {info["port"]}')
    print(f'  joint_signs: {info["joint_signs"]}')
    print('  目标姿态(rad): ' + ', '.join(f'{float(v):.4f}' for v in TARGET))
    print('  请把该臂 GELLO 摆到上述标准姿态，然后回车读取当前读数：')
    if not auto_confirm:
        input('  >>> 按回车继续... ')

    raw = read_raw_joints(info)
    offsets, corrected, residuals = compute_offsets(raw, info['joint_signs'])

    print('\n读数与计算结果：')
    header = (f"  {'idx':<4}{'raw(rad)':<11}{'sign':<6}{'offset(rad)':<13}"
              f"{'offset/quantum':<16}{'校正后(rad)':<13}{'目标(rad)':<12}{'残差':<10}")
    print(header)
    print('  ' + '-' * (len(header) - 3))
    for i in range(7):
        print(f"  {i + 1:<4}{raw[i]:<11.4f}{info['joint_signs'][i]:<6.1f}"
              f"{offsets[i]:<13.4f}{offsets[i] / QUANTUM[i]:<16.4f}"
              f"{corrected[i]:<13.4f}{TARGET[i]:<12.4f}{residuals[i]:<10.4f}")
    print()
    print('计算得到的 joint_offsets：')
    print('   ' + ', '.join(f'{float(v):.5f}' for v in offsets))

    done = write_offsets(info['cfg'], config_path, offsets)
    print(f'已写回：{done}')

    with open(done) as f:
        cfg2 = yaml.safe_load(f)
    written = cfg2['agent']['dynamixel_config']['joint_offsets']
    print('写入后的 joint_offsets：')
    print('   ' + ', '.join(f'{float(v):.5f}' for v in written))
    print('=' * 72)


def main() -> None:
    parser = argparse.ArgumentParser(
        description='GELLO 双臂标定：求解 joint_offsets 并回写左右 YAML')
    parser.add_argument('--arm', choices=['left', 'right', 'both'], default='both',
                        help='标定哪条臂（both 依次标左右，默认 both）')
    parser.add_argument('--left-config', type=str, default=None,
                        help='左臂 YAML 路径（默认 config/gello_arm1.yaml）')
    parser.add_argument('--right-config', type=str, default=None,
                        help='右臂 YAML 路径（默认 config/gello_arm2.yaml）')
    parser.add_argument('--auto-confirm', action='store_true',
                        help='跳过回车确认（需已摆好姿态）')
    args = parser.parse_args()

    left_cfg = Path(args.left_config) if args.left_config else DEFAULT_CONFIG['left']
    right_cfg = Path(args.right_config) if args.right_config else DEFAULT_CONFIG['right']

    if not left_cfg.exists():
        sys.exit(f'错误：未找到左臂配置文件 {left_cfg}')
    if not right_cfg.exists():
        sys.exit(f'错误：未找到右臂配置文件 {right_cfg}')

    if args.arm in ('left', 'both'):
        calibrate_arm('left', left_cfg, args.auto_confirm)
    if args.arm in ('right', 'both'):
        calibrate_arm('right', right_cfg, args.auto_confirm)


if __name__ == '__main__':
    main()