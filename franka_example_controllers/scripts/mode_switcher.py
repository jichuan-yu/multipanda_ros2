#!/usr/bin/env python3
"""
DualArmMprcController模式切换脚本

功能：
- 在HQP模式和零空间模式之间切换
- 查看当前控制模式
- 监控模式切换状态
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
import sys


class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')

        # 创建服务客户端
        self.set_param_client = self.create_client(
            SetParameters,
            '/dualarm_mprc_controller/set_parameters'
        )
        self.get_param_client = self.create_client(
            GetParameters,
            '/dualarm_mprc_controller/get_parameters'
        )

        self.get_logger().info('Mode Switcher Initialized')

    def get_current_mode(self):
        """获取当前控制模式"""
        if not self.get_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Get parameters service not available')
            return None

        request = GetParameters.Request()
        request.names = ['use_hqp']

        future = self.get_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            response = future.result()
            if response.values and len(response.values) > 0:
                is_hqp = response.values[0].bool_value
                mode = "HQP" if is_hqp else "Null-Space"
                print(f"Current mode: {mode}")
                return is_hqp
        else:
            print("Failed to get current mode")

        return None

    def set_mode(self, use_hqp):
        """设置控制模式"""
        if not self.set_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Set parameters service not available')
            return False

        # 创建参数
        param = ParameterValue()
        param.type = ParameterType.PARAMETER_BOOL
        param.bool_value = use_hqp

        request = SetParameters.Request()
        request.parameters = [param]

        future = self.set_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            response = future.result()
            if response.results[0].successful:
                mode = "HQP" if use_hqp else "Null-Space"
                print(f"✓ Successfully switched to {mode} mode")
                return True
            else:
                print(f"✗ Failed to set mode: {response.results[0].reason}")
                return False
        else:
            print("✗ Service call failed")
            return False

    def interactive_mode(self):
        """交互式模式切换"""
        print("\n" + "="*60)
        print("DualArmMprcController Mode Switcher")
        print("="*60)

        while True:
            print("\nCurrent Options:")
            print("1. Check current mode")
            print("2. Switch to Null-Space mode (default)")
            print("3. Switch to HQP mode (experimental)")
            print("4. Exit")

            choice = input("\nEnter your choice (1-4): ").strip()

            if choice == '1':
                current_mode = self.get_current_mode()
                if current_mode is True:
                    print("  Currently in: HQP mode")
                elif current_mode is False:
                    print("  Currently in: Null-Space mode")

            elif choice == '2':
                print("\nSwitching to Null-Space mode...")
                if self.set_mode(False):
                    print("Note: Null-Space mode is the stable, tested mode.")
                    print("Features: Basic safety control, proven performance")

            elif choice == '3':
                print("\nSwitching to HQP mode...")
                if self.set_mode(True):
                    print("Note: HQP mode is experimental.")
                    print("Features: Advanced hierarchical safety constraints")
                    print("Warning: May have higher computational cost")

            elif choice == '4':
                print("\nExiting...")
                break

            else:
                print("Invalid choice. Please enter 1-4.")


def main(args=None):
    rclpy.init(args=args)

    mode_switcher = ModeSwitcher()

    # 如果有命令行参数，直接执行
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()

        if command == 'status':
            mode_switcher.get_current_mode()

        elif command == 'nullspace' or command == 'null-space':
            print("Switching to Null-Space mode...")
            mode_switcher.set_mode(False)

        elif command == 'hqp':
            print("Switching to HQP mode...")
            mode_switcher.set_mode(True)

        else:
            print(f"Unknown command: {command}")
            print("Usage: python3 mode_switcher.py [status|nullspace|hqp]")

    else:
        # 交互式模式
        try:
            mode_switcher.interactive_mode()
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")

    mode_switcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
