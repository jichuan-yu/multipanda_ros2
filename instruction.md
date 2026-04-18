# 任务环境 my_task_description 相机配置指南

## 目标描述
在保留原有用于窗口交互展示画面的可控视角相机不变的情况下，为双臂仿真环境新增三个固定/移动相机，并开启一个专用的监控窗口来并排显示这三个相机的实时画面。
1. **固定相机 (Fixed Camera)**：位于两个机械臂连线的中垂线上，面向两台机械臂的正面，提供全局监控视角。
2. **左臂移动相机 (Left Arm Camera)**：位于左臂末端，与末端固连，始终聚焦/观察左臂末端操作区域。
3. **右臂移动相机 (Right Arm Camera)**：位于右臂末端，与末端固连，始终聚焦/观察右臂末端操作区域。
4. **画面展示窗口**：新开启一个展示窗口，按“左臂相机 | 固定相机 | 右臂相机”的顺序，依次横向排列这 3 个相机的画面。

## 技术实现细节与约束 (基于项目内的 `mujoco_ros` 特性)

根据前期对代码库的探索与测试，后续实现需严格遵循以下机制：

1. **原生相机打标签 (XML 定义)**：
   - MuJoCo 原生支持多相机，在生成 `my_task_scene_dynamic.xml` 时引入。
   - **固定相机**：直接放置在 `<worldbody>` 第一层，设置适当的 `pos` 和 `euler` 或 `xyaxes`。
   - **移动相机**：需要找到左臂和右臂的末端刚体（如 `hand` 节点），将相机标签 `<camera>` 嵌套进入该 `<body>` 内，从而实现刚性固连并随手部移动。
   - **必须命名**：每个相机必须赋予唯一的 `name` 属性（如 `fixed_cam`, `left_arm_cam`, `right_arm_cam`）。

2. **ROS 2 话题发布与“懒加载”渲染机制**：
   - `mujoco_ros` 包内的 `OffscreenCamera` 模块会自动扫描模型中所有的 `<camera>` 并为其创建 ROS 2 影像发布话题，路径通常为 `.../cameras/CAMERA_NAME/rgb/image_raw`。
   - 默认情况下会开启 15 Hz 的 RGB 渲染。可通过设置 `cam_config/CAMERA_NAME/` 相关的 ROS parameters 覆盖设定。
   - **极其重要**：`mujoco_ros` 离屏渲染是**懒加载 (Lazy rendering)**。只要图像传输话题没有任何订阅者（Subscriber），就不会触发系统的 OpenGL 渲染以节省性能。因此，我们编写的相机画面显示窗口也充当了“激活这些相机渲染”的触发器。

3. **执行步骤计划**：
   - **Step 1**：修改 `my_task_sim.launch.py`，在 XML 生成块中注入这三个 `<camera>` 节点。可以通过阅读 `mj_dual.xml` 获取左右臂末端抓手的 `body` 名称然后使用 `<body>` 或在对应位姿附着相机。
   - **Step 2**：编写一个专门的 Python `ui_node`，利用 `cv2` (OpenCV) 或 `rqt` 以及 `message_filters::TimeSynchronizer` 订阅这 3 个 `image_raw` 话题。
   - **Step 3**：在该节点中水平拼接 (hstack) 图像数据并刷新显示，生成左右臂和固定视角合一的联合监控画面。
