"""Integration test: run in a dedicated ROS_DOMAIN_ID after building the packages.
Starts only isolated test processes; requires sourced ROS/workspace and Coal library path.
"""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_prefix
import time,math,subprocess
from pathlib import Path
import rclpy
from rclpy.node import Node
from dual_arm_reactive_control.msg import CollisionObject,CollisionPairArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray
from mujoco_ros_msgs.srv import SetBodyState,SetPause
from std_srvs.srv import Empty
if not os.environ.get('ROS_DOMAIN_ID') or os.environ['ROS_DOMAIN_ID'] == '0':
 raise RuntimeError('Set a dedicated nonzero ROS_DOMAIN_ID before running this test')
work = Path(tempfile.mkdtemp(prefix='collision-sync-test-'))
fixture = Path(__file__).resolve().parent
server_config = {
 'mujoco_server': {'ros__parameters': {
  'modelfile': str(fixture / 'scene.xml'), 'no_render': True, 'headless': True,
  'unpause': False, 'MujocoPlugins': {'names': ['dynamic_obstacles'],
   'dynamic_obstacles': {'type': 'mujoco_ros::DynamicObstaclesPlugin'}}}},
 'dynamic_obstacles': {'ros__parameters': {
  'object_ids': ['floating_sphere','tool__box','tool__cylinder'],
  'geom_names': ['floating_sphere_geom','tool__box_geom','tool__cylinder_geom'],
  'publish_rate': 100.0}}}
(work/'server.yaml').write_text(yaml.safe_dump(server_config))
(work/'collision.yaml').write_text(yaml.safe_dump({'collision_objects': [
 {'id':'floating_sphere','type':'Sphere','dimensions':[.05],
  'pose':{'position':[.8,0.,.45],'orientation':[0.,0.,0.,1.]}}]}))
rclpy.init();n=Node('sync_test');states={};counts={}
def cb(m): states[m.id]=m;counts[m.id]=counts.get(m.id,0)+1
n.create_subscription(CollisionObject,'/dynamic_obstacle',cb,100)
def wait(check,timeout=5):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.02)
  if check():return
 raise AssertionError('timeout: '+str(states.keys()))
def call(typ,name,req):
 c=n.create_client(typ,name);assert c.wait_for_service(timeout_sec=3)
 f=c.call_async(req);wait(f.done);assert f.result().success, f.result();n.destroy_client(c)
def move(name,pos,quat=(0,0,0,1),vel=0):
 q=SetBodyState.Request();q.state.name=name;q.set_pose=True;q.set_twist=True
 q.state.pose.header.frame_id='world';q.state.twist.header.frame_id='world'
 q.state.pose.pose.position.x,q.state.pose.pose.position.y,q.state.pose.pose.position.z=map(float,pos)
 q.state.pose.pose.orientation.x,q.state.pose.pose.orientation.y,q.state.pose.pose.orientation.z,q.state.pose.pose.orientation.w=map(float,quat)
 q.state.twist.twist.linear.x=float(vel)
 call(SetBodyState,'/set_body_state',q)
processes=[]
import atexit
@atexit.register
def cleanup():
 for p in reversed(processes):
  p.terminate()
  try:p.wait(timeout=3)
  except subprocess.TimeoutExpired:p.kill();p.wait()
 print('Integration test logs:', work)
server = str(Path(get_package_prefix('mujoco_ros'))/'lib/mujoco_ros/mujoco_node')
processes.append(subprocess.Popen([server,'--ros-args','--params-file',str(work/'server.yaml')],
 stdout=(work/'server.log').open('w'),stderr=subprocess.STDOUT))
for executable in ['main_sim_node', 'collision_env_visualizer_node']:
 log=(work/(executable+'.log')).open('w')
 args=[str(Path(get_package_prefix('dual_arm_reactive_control'))/'lib/dual_arm_reactive_control'/executable)]
 if executable=='main_sim_node': args += ['--ros-args','-p','relative_pose_constraint_enabled:=false']
 else: args += ['--ros-args','-p','collision_env_config:='+str(work/'collision.yaml')]
 processes.append(subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT))
qpub=n.create_publisher(JointState,'/joint_states',10)
tpub=n.create_publisher(Float64MultiArray,'/dualarm_teleop_cmd',10)
joints=JointState();joints.name=[f'mj_{side}_joint{i}' for side in ['left','right'] for i in range(1,8)]
joints.position=[0.,-.785,0.,-2.356,0.,1.571,.785]*2;joints.velocity=[0.]*14
cmd=Float64MultiArray();cmd.data=list(joints.position)+[4.,3.,0.]
n.create_timer(.02,lambda:(qpub.publish(joints),tpub.publish(cmd)))
pairs=[];markers=[]
def pairs_cb(msg): pairs[:]=msg.pairs
def markers_cb(msg): markers[:]=msg.markers
n.create_subscription(CollisionPairArray,'/collision_pairs',pairs_cb,10)
n.create_subscription(MarkerArray,'/collision_env_markers',markers_cb,10)
wait(lambda:len(states)==3)

assert 'excluded' not in states
move('floating_sphere',[.9,.1,.6],vel=.02)
wait(lambda:abs(states['floating_sphere'].pose.position.x-.9)<1e-6)
assert abs(states['floating_sphere'].twist.linear.x-.02)<1e-6
wait(lambda:any(m.ns=='static_collision' and abs(m.pose.position.x-.9)<1e-5 and abs(m.pose.position.z-.6)<1e-5 for m in markers))
# Every nearest obstacle point for this sphere must lie on its new surface.
wait(lambda:any(p.object2_id=='floating_sphere' for p in pairs) and all(abs(math.dist([p.nearest_point2.x,p.nearest_point2.y,p.nearest_point2.z],[.9,.1,.6])-.05)<1e-5 for p in pairs if p.object2_id=='floating_sphere'))
sphere_pairs=[p for p in pairs if p.object2_id=='floating_sphere']
for p in sphere_pairs:
 v=p.nearest_point2
 assert abs(math.dist([v.x,v.y,v.z],[.9,.1,.6])-.05)<1e-5, p
print('PASS: actual MPRC nearest collision points and RViz markers follow moved sphere',flush=True)
move('tool',[2,0,1],(0,0,math.sqrt(.5),math.sqrt(.5)))
wait(lambda:abs(states['tool__box'].pose.position.y-.2)<1e-6 and abs(states['tool__cylinder'].pose.position.x-1.7)<1e-6)
assert abs(states['tool__cylinder'].pose.position.x-1.7)<1e-6
assert list(states['tool__cylinder'].geometry.dimensions)==[.12,.03]
q=SetPause.Request();q.paused=False;call(SetPause,'/set_pause',q)
start=time.monotonic();counts.clear()
wait(lambda:time.monotonic()-start>1.2)
assert states['floating_sphere'].pose.position.x>.915
assert abs(states['floating_sphere'].twist.linear.x-.02)<1e-5
assert abs(states['floating_sphere'].pose.position.z-.6)<1e-6
assert 50<counts['floating_sphere']<150,counts
print('PASS: paused updates, true velocity, running motion, sphere radius, compound rotation/offset, cylinder dimensions, excluded object, bounded rate',counts)
reset=n.create_client(Empty,'/reset')
assert reset.wait_for_service(timeout_sec=3)
f=reset.call_async(Empty.Request());wait(f.done)
wait(lambda:abs(states['floating_sphere'].pose.position.x-.8)<1e-5)
print('PASS: reset refreshes synchronized poses')
n.destroy_client(reset)
n.destroy_node();rclpy.shutdown()
