#!/usr/bin/env python3
"""场景注入脚本。

在启动时把选定场景的任务物体 XML 注入到 my_task_scene 包装文件里，
使不同场景各自独立、互不干扰（不修改共享的 franka_description/.../objects.xml）。

新增一个场景：
    在 mujoco/scenes/ 下添加 <scene_name>_objects.xml（mujocoinclude 结构），
    即可通过 `ros2 launch ... scene:=<scene_name>` 选用，无需改动其他文件。
"""

import os

# 可用场景名；'none' 表示回退到默认 objects.xml（保持旧行为）
KNOWN_SCENES = ['shelf', 'chair', 'none']
DEFAULT_SCENE = 'shelf'


def inject_scene(scene, scenes_dir, default_objects):
    """返回包装 XML 中应写入的 <include> 片段。

    参数:
        scene:           启动参数 scene 的值
        scenes_dir:      场景文件所在目录（share/my_task_description/mujoco/scenes）
        default_objects: 默认 objects.xml 的绝对路径（scene='none' 时回退）
    返回:
        str，形如 '<include file="<场景文件绝对路径>"/>'
    异常:
        FileNotFoundError: 场景不存在
    """
    if scene == 'none':
        return f'<include file="{default_objects}"/>'

    scene_file = os.path.join(scenes_dir, f'{scene}_objects.xml')
    if not os.path.exists(scene_file):
        raise FileNotFoundError(
            f"场景 '{scene}' 不存在。可用场景: {KNOWN_SCENES}；"
            f"如需新增，请在 {scenes_dir} 下添加 {scene}_objects.xml")
    return f'<include file="{scene_file}"/>'
