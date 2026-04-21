# 运行步骤

1. **编译工作空间**
   在工作空间根目录下运行（如果刚执行过则可跳过）：
   ```bash
   colcon build --packages-select my_task_description
   ```

2. **重新加载环境变量**
   ```bash
   source install/setup.bash
   ```

3. **终端1：启动带有三个新相机的双臂仿真环境**
   ```bash
   ros2 launch my_task_description my_task_sim.launch.py
   ```
   你可以检查这个窗口里 MuJoCo 右下角的 `Camera` 选项卡，会多出三个选项 `fixed_cam`, `left_arm_cam`, `right_arm_cam`。不过此时因为懒加载的缘故，如果不开启任何监听节点，它们并不会渲染降低性能。

4. **终端2：新开一个终端，启动多相机联合监视器**
   ```bash
   source install/setup.bash
   ros2 run my_task_description camera_viewer.py
   ```
   这个脚本包含我们提到的懒加载“触发”并拼接窗口，运行后将弹出一个名为 `MuJoCo Multi-Camera Monitor` 的 OpenCV 窗口，从左到右依次为左臂相机画面、全局相机画面和右臂相机画面。
