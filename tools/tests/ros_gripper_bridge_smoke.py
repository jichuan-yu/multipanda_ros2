"""Test the compiled C++ bridge against an isolated synthetic action server."""
import os
if os.environ.get('ROS_DOMAIN_ID') != '220' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('Use ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1')

from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from franka_msgs.action import Grasp
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = Node('bridge_test_action_server')
received = []
server = ActionServer(node, Grasp, '/mj_left_gripper_sim_node/grasp',
    execute_callback=lambda handle: execute(handle),
    goal_callback=lambda goal: GoalResponse.REJECT if goal.width < 0 else GoalResponse.ACCEPT)


def execute(handle):
    result = Grasp.Result()
    result.success = handle.request.width > 0
    if result.success:
        handle.succeed()
    else:
        handle.abort()
    return result


pub = node.create_publisher(Float64MultiArray, '/mj_left_gripper/grasp_desired', 10)
sub = node.create_subscription(Float64MultiArray, '/mj_left_gripper/grasp_status',
                               lambda msg: received.append(list(msg.data)), 10)
workspace = Path(__file__).resolve().parents[4]
binary = workspace/'build/franka_example_controllers/gripper_action_bridge'
with open('/tmp/tray_bridge_protocol.log', 'w') as log:
    proc = subprocess.Popen([str(binary), '--ros-args', '-p', 'arm_id:=left'], stdout=log, stderr=log)
    try:
        deadline = time.monotonic()+10
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert pub.get_subscription_count() > 0
        for width, request_id, expected in [(.08, 101., 2.), (0., 102., -2.), (-.1, 103., -1.), (.08, 0., 2.)]:
            data = [width, .05, 30., .005, .005]
            if request_id:
                data.append(request_id)
            pub.publish(Float64MultiArray(data=data))
            deadline = time.monotonic()+8
            while [request_id, expected] not in received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            assert [request_id, expected] in received, received
        print('PASS: compiled bridge success/failure/rejection/legacy request IDs', received)
    finally:
        proc.terminate(); proc.wait(timeout=5)
        server.destroy(); node.destroy_node(); rclpy.shutdown()
