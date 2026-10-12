# Environment Config Convention

## Overview

This directory contains YAML configurations for primitive and rigid compound objects. The generator outputs MuJoCo XML for all objects, and MPRC / cuRobo YAML for static objects only.

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
- 静态复合体在 MPRC / cuRobo 中展开为独立碰撞几何体，使用组合后的世界位姿。
  名称为 `父id__子id`，嵌套时继续追加。重复名称及展开后的名称冲突会被拒绝。
- 可移动复合体和可移动单体一样，不会出现在两份规划 YAML 中。
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
可移动工具和碎屑不会出现在静态规划 YAML 中，工具附着和动态碰撞需要控制流程另行处理。

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
