#!/usr/bin/env python3

import pyspacemouse
import time
import sys

def main():
    # 尝试连接到 SpaceMouse
    success = pyspacemouse.open()
    if not success:
        print("无法连接到 SpaceMouse！")
        print("请检查设备是否已插入，以及当前用户是否有读取 HID 设备的权限。")
        print("可以尝试使用 sudo 运行，或者配置 udev 规则。")
        sys.exit(1)

    print("成功连接到 SpaceMouse！")
    print("开始监听 SpaceMouse 数据。按 Ctrl+C 退出...\n")
    print("-" * 120)
    print(f"{'延时信息':>32} | {'X':>8} | {'Y':>8} | {'Z':>8} | {'Roll':>8} | {'Pitch':>8} | {'Yaw':>8} | {'Buttons'}")
    print("-" * 120)

    try:
        last_event_time = time.perf_counter()
        while True:
            # 读取当前状态
            t1 = time.perf_counter()
            state = pyspacemouse.read()
            t2 = time.perf_counter()
            
            read_delay = (t2 - t1) * 1000.0 # 单次读取耗时(ms)

            if state:
                # 检查是否所有状态均为0或者没有按键被按下
                is_idle = (state.x == 0.0 and state.y == 0.0 and state.z == 0.0 and 
                           state.roll == 0.0 and state.pitch == 0.0 and state.yaw == 0.0 and 
                           not any(state.buttons))
                
                if not is_idle:
                    dt = (t2 - last_event_time) * 1000.0 # 两次有效事件的间隔(ms)
                    last_event_time = t2
                    freq = 1000.0 / dt if dt > 0 else 0.0
                    
                    # pyspacemouse 解析后的状态包含：x, y, z, roll, pitch, yaw, buttons
                    # 使用带符号和固定小数位的格式化输出，方便观察
                    delay_info = f"[r:{read_delay:4.2f}ms|dt:{dt:5.1f}ms|{freq:3.0f}Hz]"
                    print(f"{delay_info:>32} | "
                          f"{state.x:+8.3f} | {state.y:+8.3f} | {state.z:+8.3f} | "
                          f"{state.roll:+8.3f} | {state.pitch:+8.3f} | {state.yaw:+8.3f} | "
                          f"{state.buttons}")
                else:
                    last_event_time = t2
            
            # 短暂休眠以降低 CPU 占用
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，正在退出...")
    finally:
        # 确保安全关闭设备
        pyspacemouse.close()

if __name__ == '__main__':
    main()
