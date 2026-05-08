请为我在src/multipanda_ros2/spacemouse_teleop下写一个key_pub_ik.py脚本，可以参考key_pub.py
读取键盘输入并按如下规则映射到笛卡尔空间：
----------------------------------------
Keyboard Teleop for Dual Cartesian Arm
----------------------------------------
Z/X : Select Left/Right arm (Current: {})
W/S : Forward/Backward (X)
A/D : Left/Right (Y)
Q/E : Up/Down (Z)
J/L : Rotate around X (-/+)
I/K : Rotate around Y (-/+)
U/O : Rotate around Z (-/+)
C/V : Close/Open Gripper
Ctrl-C to quit
----------------------------------------
step_pos采用0.01
step_rot采用0.05

初始笛卡尔空间坐标左右臂均为[0.307,  0.0, 0.487]
获取键盘输入产生笛卡尔空间的新坐标后，通过pin-pink库做逆运动学解算获得对应的关节空间坐标
对于新生成的关节空间坐标，进行正运动学解算，获得笛卡尔空间坐标，验证是否与预期一致
机器人模型请阅读src/multipanda_ros2/my_task_description/launch/my_task_sim.launch.py查找

注意：
1. 使用source ~/myenv/bin/activate进入虚拟环境
