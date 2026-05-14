目前已经能够正常实现点云渲染，操作方法如下：
step1:
启动仿真，同时打开rviz面板
step2:
切换坐标系到相机坐标系left_arm_cam_optical_frame...（后续会调整tf树使得相机对准）
切换topic到对应相机
切换模式到best effort
 step3:
 实现点云渲染