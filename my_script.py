import re
with open('franka_description/mujoco/franka/mj_dual.xml', 'r') as f:
    text = f.read()

# find where to inject left camera
text = re.sub(r'(<body name="mj_left_hand"[^>]*>)', r'\1\n<camera name="left_arm_cam" pos="0.05 0 0.05" xyaxes="0 1 0 -1 0 0"/>', text)
print(text.find('left_arm_cam'))
