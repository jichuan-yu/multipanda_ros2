# Environment Config Convention

## Overview

This directory contains YAML configurations for primitive and rigid compound objects. The generator outputs MuJoCo XML for all objects, and MPRC / cuRobo YAML for objects selected by `include_in_controller_collision` (default: `true`), independently of `movable`.

---

## Shape Conventions

### Box (长方体)

```yaml
- id: "box_01"
  type: "Box"
  dimensions: [length, width, height]  # 沿XYZ轴的尺寸 (米)
  pose:
    position: [x, y, z]                # 中心位置 (米)
    orientation: [x, y, z, w]           # 四元数 (可选，默认 [0,0,0,1])
  material:                            # 可选
    friction: [2, 0.005, 0.0001]      # 滑动/扭转/滚动摩擦
    rgba: [1, 0, 0, 1]                 # 颜色 (R,G,B,A)
```

**MuJoCo Note:** `size` = [length/2, width/2, height/2]

---

### Cylinder (圆柱)

```yaml
- id: "cylinder_01"
  type: "Cylinder"
  dimensions: [radius, height]         # 半径和高度 (米)
  pose:
    position: [x, y, z]                # 几何中心位置 (米)
    orientation: [x, y, z, w]           # 四元数 (可选，默认 [0,0,0,1])
  material:                            # 可选
    friction: [2, 0.005, 0.0001]
    rgba: [0, 1, 0, 1]
```

**MuJoCo Note:** `size` = [radius, height/2], 默认Z轴朝上

---

### Sphere (球体)

```yaml
- id: "sphere_01"
  type: "Sphere"
  dimensions: [radius]                  # 半径 (米)
  pose:
    position: [x, y, z]                # 球心位置 (米)
    orientation: [x, y, z, w]           # 球体忽略方向，但保留字段
  material:                            # 可选
    friction: [2, 0.005, 0.0001]
    rgba: [0, 0, 1, 1]
```

**MuJoCo Note:** `size` = [radius]

---

## Full Config Example

```yaml
# my_environment.yaml
# 环境描述（可选）
description: "Simple test environment with a table and some obstacles"

# 碰撞物体列表
collision_objects:
  # 工作台面
  - id: "table_surface"
    type: "Box"
    dimensions: [0.6, 0.8, 0.05]      # 60cm x 80cm x 5cm
    pose:
      position: [0.4, 0.0, 0.0]      # 中心位置
      orientation: [0, 0, 0, 1]

  # 圆柱障碍物
  - id: "pillar_01"
    type: "Cylinder"
    dimensions: [0.05, 0.4]          # r=5cm, h=40cm
    pose:
      position: [0.5, 0.2, 0.2]      # 几何中心
      orientation: [0, 0, 0, 1]
    material:
      rgba: [0.2, 0.4, 0.8, 1]

  # 球体障碍物
  - id: "ball_01"
    type: "Sphere"
    dimensions: [0.08]               # r=8cm
    pose:
      position: [0.6, -0.2, 0.08]    # 球心位置
    material:
      rgba: [1, 0.5, 0, 1]

  # 方块
  - id: "box_01"
    type: "Box"
    dimensions: [0.1, 0.1, 0.1]
    pose:
      position: [0.5, 0.0, 0.05]
      orientation: [0, 0, 0, 1]
```

---

## Usage

```bash
# 从工作空间根目录运行 (推荐)
python3 src/multipanda_ros2/tools/env_generator.py --config env_config/my_environment.yaml

# 从 tools 目录运行
cd src/multipanda_ros2/tools
python3 env_generator.py --config env_config/my_environment.yaml

# 自定义输出路径
python3 src/multipanda_ros2/tools/env_generator.py \
    --config env_config/my_environment.yaml \
    --xml-output /path/to/objects.xml \
    --yaml-output /path/to/collision_env_sim.yaml

# 预览 (不写入文件)
python3 src/multipanda_ros2/tools/env_generator.py --config env_config/my_environment.yaml --dry-run
```

---

## Default Output Paths

- **MuJoCo XML:** `src/multipanda_ros2/franka_description/mujoco/franka/objects.xml`
- **MPRC YAML:** `src/dualarm_mprc/dualarm_reactive_control/config/collision_env_my_task.yaml`

---

## Coordinate System

- **X**: Forward direction (away from robot base)
- **Y**: Left/Right direction
- **Z**: Upward direction
- **Origin**: Robot base position

All positions are in meters (m).


## 固定拼接复合物体（Compound）

使用 `type: Compound` 和非空 `parts` 列表，将多个基本几何体固定为一个刚体。
`Compound` 不填写 `dimensions`。完整示例见 [compound_example.yaml](compound_example.yaml)。

```yaml
collision_objects:
  - id: l_shape
    type: Compound
    movable: true
    density: 500.0
    pose:
      position: [0.4, 0.0, 0.15]
      orientation: [0, 0, 0, 1]
    parts:
      - id: horizontal
        type: Box
        dimensions: [0.2, 0.04, 0.04]
      - id: vertical
        type: Box
        dimensions: [0.04, 0.04, 0.16]
        pose:
          position: [0.08, 0.0, 0.1]
```

- 根对象的 `pose` 相对于世界坐标系；每个部件的 `pose` 相对于直接父对象。
  位置始终是几何中心，长度单位为米，四元数顺序为 `[x, y, z, w]`，应为单位四元数。
- 支持嵌套 `Compound`。整个复合体在 MuJoCo 中使用同一个 `body` 和多个 `geom`，
  内部没有关节；即使部件几何上不接触，也保持固定的相对位姿。
- `movable: false`（默认）固定在世界中；`movable: true` 仅在根刚体添加一个 `freejoint`，
  整体具有六个自由度。`movable` 和 `gravcomp` 的非默认值只能设在根对象。
- 部件继承父对象的 `material` 和 `density`，也可单独覆盖；密度参与 MuJoCo 的质量和惯量计算。
  几何重叠区域不会做布尔合并，因此重叠部件的质量会重复计入。
- 被选入控制器碰撞环境的复合体在 MPRC / cuRobo 中展开为独立碰撞几何体，使用组合后的世界位姿。
  名称为 `父id__子id`，嵌套时继续追加。重复名称及展开后的名称冲突会被拒绝。
- 单体和复合体均由 `include_in_controller_collision` 决定是否进入两份规划 YAML，与可移动性无关。
- MuJoCo 和 MPRC 保留 Box / Cylinder / Sphere 原始形状。
  cuRobo 输出使用 `cuboid`：圆柱使用 `[2r, 2r, h]`，球体使用 `[2r, 2r, 2r]`
  的保守包围盒，可能比实际形状阻挡更多空间。
- 原有单体配置保持可用；MuJoCo 四元数输出已改为 `[w, x, y, z]`。

在工作区根目录预览（不会写文件）：

```bash
python3 src/multipanda_ros2/tools/env_generator.py \
  --config env_config/compound_example.yaml --dry-run
```

去掉 `--dry-run` 会覆盖默认的三个输出文件；也可以通过 `--xml-output`、
`--yaml-output`、`--curobo-output` 指定各自路径。

运行回归测试：

```bash
python3 -m unittest discover -s src/multipanda_ros2/tools/tests -v
```

## 清扫场景（cleaning_scene.yaml）

[cleaning_scene.yaml](cleaning_scene.yaml) 包含 1 把扫帚、1 个簸箕、8 个边长
18–25 mm 的可移动小立方体，以及唯一的固定靠柱；另外两个障碍物已移除。
使用现有 `dual_scene.xml` 的 `z=0` 地面，不另外生成重复地板。

左右方向以机械臂世界坐标为准：左为 +Y，右为 -Y，不随观察相机改变。
扫帚和簸箕均为可移动复合刚体。靠柱 `fixed_block` 的长宽由 14 cm 缩至
8 cm，高度保持 40 cm，中心约为 `[0.44337, 0.34, 0.20]` m。
扫帚位于靠柱右侧（-Y），刷头原点约为 `[0.44337, 0.22663, 0.04148]` m，
手柄偏离竖直方向 10°、朝 +Y 倾斜。刷毛接地，长 0.55 m 的手柄靠在柱体
-Y 侧的顶部棱边；扫帚保留自由关节，未固定或禁用重力。

簸箕原点为 `[0.307, -0.255, 0]` m，绕 Z 轴旋转 -90°，开口朝左（+Y）。
底板厚 3 mm，顶部敞开。圆柱柄竖直朝 +Z，安装在后挡板顶面（局部 z=0.06 m），
局部中心为 `[0.105, 0, 0.285]` m，长 0.45 m，顶部离地 0.51 m。
柄中心的世界坐标为 `[0.307, -0.36, 0.285]` m。
根据 `my_task_sim.launch.py` 默认初始关节角，右臂末端约为
`[0.30702, -0.26, 0.48677]` m，与圆柱柄轴线最近距离约 10 cm。
垃圾块分散在 x=0.30–0.82 m、y=-0.32–0.18 m 范围；
`debris_02` 移至 `[0.57, -0.23, 0.0125]` m，避开重新摆放的簸箕。
扫帚刷毛是刚性几何体，不能模拟柔性弯曲；簸箕入口为简单薄板边缘，没有斜坡。
这是场景和碰撞几何示例，不包含抓取、扫地或倾倒控制策略，也未验证机械臂全程可达性。
本示例的可移动工具和碎屑显式设置了 `include_in_controller_collision: false`，不会出现在规划 YAML 中，工具附着和动态碰撞需要控制流程另行处理。

在 `multipanda-container` 的 `myenv` 中预览：

```bash
docker exec multipanda-container /home/developer/myenv/bin/python \
  /home/xiaozy24/dual_panda_ws/src/multipanda_ros2/tools/env_generator.py \
  --config env_config/cleaning_scene.yaml --dry-run
```

去掉 `--dry-run` 可以替换默认的仿真和规划环境文件。
如仅想导出独立文件，保留配置参数并指定以下三个参数（路径在容器内）：

```text
--xml-output /tmp/cleaning_scene/objects.xml
--yaml-output /tmp/cleaning_scene/collision_env.yaml
--curobo-output /tmp/cleaning_scene/scene.yml
```

运行包含 MuJoCo 编译、30 秒静置仿真和双臂启动姿态碰撞检查的测试：

```bash
docker exec multipanda-container /home/developer/myenv/bin/python -m unittest discover \
  -s /home/xiaozy24/dual_panda_ws/src/multipanda_ros2/tools/tests -v
```

验证环境为 Python 3.10、PyYAML 6.0.3、MuJoCo 3.15.0。
已验证全部 11 个原有配置及新增示例可编译，并验证清扫环境与双 Panda 模型共同加载。
旧配置的字段和物体语义保持兼容，但生成文件不保证逐字相同：
四元数输出顺序修正、位姿精度提高、空列表显式输出、cuRobo 曲面几何使用保守包围盒。

调整后布局另经 30 秒静置仿真验证：扫帚仍约倾斜 10°，保持地面和靠柱接触；
按启动文件默认关节角设置双臂后，没有新物体初始穿透；若覆盖启动关节角，应重新检查。

## 托盘搬运场景（tray_transport.yaml）

[tray_transport.yaml](tray_transport.yaml) 是独立场景，左右以世界坐标 +Y / -Y 为准，
机械臂前方为 +X；沿用主场景 z=0 的地面。

| 物体 | 位置和尺寸 |
| --- | --- |
| 左侧固定小板凳 | 中心 x=0.60、y=0.35 m；凳面 25×15 cm；总高 20 cm |
| 右侧固定大板凳 | 中心 x=0.60、y=-0.35 m；凳面 25×20 cm；总高 35 cm |
| 可移动托盘 | 放在小板凳中央，底板 30×20 cm、厚 1 cm；四面挡板高 2 cm、厚 1 cm |
| 六个可移动方块 | 每个 3×3×3 cm；盘中心单列叠放 6 层，每层 1 个 |

两个板凳和托盘均绕 Z 轴旋转 90°，长边沿世界 Y 方向、短边沿 X 方向，中心位置不变。
两个板凳均为固定复合物体，每个由 3 cm 厚的凳面和四条 4×4 cm 的腿组成。
板凳高度指地面到凳面上表面的总高度。托盘底面 z=0.20 m，盘内底面 z=0.21 m，
挡板顶部 z=0.23 m。六个方块中心的 x、y 均为 0.60、0.35 m，
中心高度依次为 z=0.225、0.255、0.285、0.315、0.345、0.375 m，
逐层直接叠放，最上层顶部离地 0.39 m。托盘及六个方块分别拥有自由关节，
未将方块固定在托盘上。托盘大于凳面，放置时有对称悬伸。

MPRC/cuRobo 的规划环境包含两个板凳展开后的 10 个几何体，以及 `floating_sphere`。
球体半径 5 cm，初始位置 `[0.8, 0.0, 0.45]`，可移动且 `gravcomp: 1.0` 完全补偿重力；
省略 `gravcomp` 时默认值为 `0.0`，物体受重力影响。
托盘和方块的抓取、附着碰撞及搬运控制需由任务控制流程处理。
已通过尺寸检查、30 秒静置堆叠接触检查和双臂启动默认姿态下的初始碰撞检查；
未执行完整搬运轨迹。

预览：

```bash
docker exec multipanda-container /home/developer/myenv/bin/python \
  /home/xiaozy24/dual_panda_ws/src/multipanda_ros2/tools/env_generator.py \
  --config env_config/tray_transport.yaml --dry-run
```

去掉 `--dry-run` 将把默认环境输出替换为托盘搬运场景。

### 暂停恢复时堆叠物体弹开（MuJoCo 3.2.0）

已复现：同一模型直接运行保持稳定，但先暂停、反复执行 `mj_forward` 再恢复运行，
六层方块可能散落。原因定位到暂停计算留下的约束加速度热启动缓存
`qacc_warmstart`；本场景较硬的接触约束对该状态敏感。

修复位于 `mujoco_ros_pkgs/mujoco_ros/src/physics.cpp` 的 `SimPausedPhysics`：
在仅供暂停显示使用的 `mj_forward` 前后清零热启动缓存，避免把未经过积分的
暂停求解结果传入下一物理步。正常运行期间仍保留热启动，不改变模型尺寸、
碰撞开关或物体自由关节，也不需要把方块固定在一起。

修改核心源码后，需重新构建并安装 `mujoco_ros`，然后重启仿真进程；
仅重新生成 `objects.xml` 不能应用此修复。本次已在 `multipanda-container`
中完成仿真库的构建和安装。

回归测试使用实际引擎版本 MuJoCo 3.2.0、2 ms 步长，并覆盖暂停的前向计算后
恢复运行。MuJoCo 3.15.0 的普通静置测试不能替代实际 ROS 暂停恢复验证。

完整 ROS 回归已通过：以暂停模式启动，连续执行两轮“暂停 3 秒→运行 12 秒”，
通过 `/get_body_state` 采样托盘和六个方块，最大位移约 0.04 mm，未出现散落。
在容器的工作区根目录重新加载构建环境并重启即可生效：

```bash
source install/setup.bash
ros2 launch my_task_description my_task_sim.launch.py use_rviz:=true
```

### 托盘夹持参数与验证

`franka_description/mujoco/franka/mj_dual.xml` 的双夹爪使用限力位置伺服：
总开口 0–8 cm 对应控制量 0–255，腱空间刚度 10000 N/m、阻尼
100 N·s/m，执行器力限制为 ±30 N（通过系数 0.5 分配到各手指）。
原刚度 100 N/m 在夹住 1 cm 挡板时仅产生约 0.5 N 腱力，难以承载托盘。
手指碰撞启用 6 维接触及扭转/滚动摩擦，并使用 implicitfast 积分处理较高阻尼。
这些是仿真参数，不代表真实 Panda 的标定值；动作服务的 force 参数仍是停止阈值，
不是持续的闭环力目标。改变动作参数不能绕过模型的 30 N 上限。

`tools/tests/test_gripper_tray_mujoco.py` 使用真实手指几何与模型参数，
以运动夹具驱动双手，从托盘两侧夹住挡板，验证自由托盘抬升至少 9 cm 后保持 120 s，保持期间高度变化小于 2 mm。
该测试没有固定托盘或方块，不覆盖机械臂轨迹可达性，也不保证六层方块不会倾倒。
实际操作应双侧夹持、缓慢同步抬升，避免手掌碰到中央方块堆。
修改模型后需重新启动仿真，使动态场景重新生成。

持续夹持还需要摩擦约束的稳定求解：模型使用 `cone="elliptic"` 和
`noslip_iterations="10"`，减少软接触在静态载荷下的残余滑移。
仅增加夹持力或仅切换摩擦锥，在 120 s 回归实验中仍会滑落。
该设置保留有限摩擦和独立自由物体，不会把托盘绑定到手指；接触求解开销会增加。


### 是否计入控制器碰撞环境

每个物体可设置布尔参数 `include_in_controller_collision`，默认 `true`：

```yaml
  - id: grasp_object
    type: Box
    movable: true
    include_in_controller_collision: false
    dimensions: [0.03, 0.03, 0.03]
    pose:
      position: [0.6, 0.0, 0.1]
```

- `movable` 只控制 MuJoCo 中是否可移动，不再影响碰撞环境导出。
- `include_in_controller_collision` 同时控制 MPRC 和 cuRobo YAML，
  不改变 MuJoCo 的几何、接触和动力学。
- 复合物体根对象设为 `false` 会排除整个复合体；根对象为 `true` 时，
  子部件也可用 `false` 单独排除。
- 本目录全部基本 YAML 已显式配置该字段：原可移动物体为 `false`，
  原固定物体为 `true`，保持原有输出语义。
- 旧的外部配置若省略该字段，可移动物体现在也会被导出；要保持旧行为，
  请为它们补上 `include_in_controller_collision: false`。
- 导出的 YAML 位姿是初始世界位姿；MuJoCo → MPRC/RViz 的运行时位姿由下述同步插件更新。cuRobo 的离线 YAML 不会自动跟踪运动。


### 可移动碰撞物体实时同步

生成器额外生成 `objects.sync.yaml`，默认与输出 XML 同目录（`--sync-output` 可覆盖）。
清单来自基本 YAML：只同步 `movable: true` 且 `include_in_controller_collision: true`
的物体或复合部件。不要手改此生成文件。`my_task_sim.launch.py` 自动读取安装后的清单。
基本配置顶层可选 `dynamic_collision_publish_rate: 100.0`（Hz，默认 100）。

`mujoco_ros::DynamicObstaclesPlugin` 只读取真实 geom 世界位姿、线速度和角速度，
通过 `/dynamic_obstacle` 更新 MPRC 与 RViz；它不驱动物体，不覆盖 qpos/qvel，
不依赖 `dyn_sphere_` 名称。复合部件沿用生成的 `根__部件` ID，保留局部位姿和旋转。
暂停时支持位姿编辑反馈，并以低频重复状态供后启动的订阅者接收。

新增/移除动态物体后重新运行生成器，构建 `franka_description`，重启仿真。
首次使用此功能还需构建 `mujoco_ros2_dummy_plugin`、`my_task_description` 和
`dual_arm_reactive_control`（扩大多物体反馈接收队列），并重启控制器和可视化节点。
不要同时运行旧的随机移动障碍物发布器来更新相同 ID。

集成回归测试（在容器内 source ROS 和工作区后，选择未使用的 ROS_DOMAIN_ID）：

```bash
export LD_LIBRARY_PATH=/home/xiaozy24/dual_panda_ws/src/dualarm_mprc/coal/build/lib:$LD_LIBRARY_PATH
ROS_DOMAIN_ID=99 python3 src/multipanda_ros2/tools/tests/dynamic_sync/check_sync.py
```

测试会在独立域启动临时 MuJoCo、真实 MPRC 控制器和标记发布节点，验证暂停位置修改、
运行中位姿/速度、复合部件旋转、控制器最近碰撞点、RViz 标记和重置后的反馈。
无界面 GLFW 仿真的 ROS 回调执行器也已补齐，确保该模式能够响应物体状态服务。

### 运行中拖动后停止

MuJoCo 查看器 `Option → Stop on release` 默认开启。左键双击选择自由物体，
按住 Ctrl + 右键拖动，松开鼠标后清零该自由刚体的线速度、角速度和对应求解预热状态。
Ctrl + 左键旋转同样支持松手停止。取消勾选可恢复原来的惯性拖动。
该操作不暂停仿真、不改变松手时的位置、不锁定物体，也不会清零机器人关节速度。
`gravcomp: 1.0` 的无重力球在无其他外力时会停留；普通物体仍会下落，碰撞仍可使其运动。
