你之前已经在franka_bringup/config/sim/dual_sim_controllers.yaml为我实现了subscriber下新写的双臂关节控制器
接下来我希望你为我实现笛卡尔空间的双臂控制器
考虑到现有的subscriber控制器不能很好适配双臂环境
请你基于现有的subscriber控制器，为我新写一个笛卡尔双臂控制器
新控制器在代码风格等方面应该与现有控制器一致
并为我在写tools/下写一个自动化脚本key_pub.py，通过读取键盘输入控制机械臂运动
映射关系：
Z/X 选择左/右侧机械臂
W/S 控制沿X轴向前/向后
A/D 控制沿Y轴向左/向右
Q/E 控制沿Z轴向上/向下
J/L 控制绕X轴逆时针/顺时针旋转
I/K 控制绕Y轴逆时针/顺时针旋转
U/O 控制绕Z轴逆时针/顺时针旋转
（默认选择左侧机械臂） 
