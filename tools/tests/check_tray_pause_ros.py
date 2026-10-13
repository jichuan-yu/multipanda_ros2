"""Explicit integration check: installed ROS simulator, paused launch, two resume cycles.

Run with ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 after sourcing ROS and install.
Do not run alongside another simulation (the launch shares its temporary model files).
"""
import os,signal,subprocess,time,math
from pathlib import Path
import rclpy
from mujoco_ros_msgs.srv import GetBodyState,SetPause
assert os.environ.get('ROS_DOMAIN_ID')=='221', 'Use isolated ROS_DOMAIN_ID=221'
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1', 'Use ROS_LOCALHOST_ONLY=1'
# The project's launch uses a shared /tmp/mujoco_test_env directory even across domains.
for process_path in Path('/proc').iterdir():
 if not process_path.name.isdigit(): continue
 try:
  command=(process_path/'cmdline').read_bytes().split(b'\0')[0]
  if command and Path(command.decode()).name=='mujoco_node':
   raise RuntimeError('Stop the existing simulator before running this regression')
 except (FileNotFoundError, PermissionError): pass
log=Path('/tmp/tray_pause_current.log')
with log.open('w') as stream:
 proc=subprocess.Popen(['ros2','launch','my_task_description','my_task_sim.launch.py','use_rviz:=false','no_render:=true','unpause:=false'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
 rclpy.init();node=rclpy.create_node('tray_pause_current_check')
 get=node.create_client(GetBodyState,'/get_body_state');pause=node.create_client(SetPause,'/set_pause')
 def call(client,req):
  fut=client.call_async(req);rclpy.spin_until_future_complete(node,fut,timeout_sec=5)
  assert fut.done(),'service timeout';res=fut.result();assert res.success,res;return res
 def toggle(v):
  req=SetPause.Request();req.paused=v;call(pause,req)
 def positions():
  out=[]
  for name in ['tray']+[f'cube_{i:02}' for i in range(1,7)]:
   req=GetBodyState.Request();req.name=name;p=call(get,req).state.pose.pose.position;out.append((p.x,p.y,p.z))
  return out
 try:
  assert get.wait_for_service(timeout_sec=25) and pause.wait_for_service(timeout_sec=5),log.read_text()[-4000:]
  toggle(True);initial=positions();print('INITIAL',initial,flush=True)
  for cycle in range(2):
   time.sleep(3);toggle(False);peak=0
   deadline=time.monotonic()+12
   while time.monotonic()<deadline:
    current=positions();peak=max(peak,max(math.dist(a,b) for a,b in zip(initial,current)))
    if peak>.01:print('UNSTABLE',cycle,peak,current,flush=True);break
    time.sleep(.2)
   toggle(True);print('CYCLE',cycle,'max displacement',peak,flush=True)
   assert peak<.002,(cycle,peak)
 finally:
  node.destroy_node();rclpy.shutdown();os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
