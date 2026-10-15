# multipanda_ros2 — 仿真与底层控制提供者

`tmp/tray-repro-no-curobo` 是从 `srt_xzy` 建立的临时复现分支。配合
`dualarm_mprc` 同名分支的 [托盘复现指南](https://github.com/jichuan-yu/dualarm_mprc/blob/tmp/tray-repro-no-curobo/docs/TRAY_REPRODUCTION.md)
使用；`tools/install_tray_libraries.sh` 安装固定版本 CPU 库，不需要 cuRobo/CUDA。

本仓库维护 Franka 硬件接口、机器人模型、MuJoCo 集成、通用 ros2_control 控制器与夹爪桥接。
任务场景、遥操作、环境生成和实验记录已迁入相邻的 `dualarm_mprc` 仓库。

- 项目工作流与迁移入口：[dualarm_mprc 工作区指南](../dualarm_mprc/docs/WORKSPACE.md)
- 上游安装和通用功能：[README_ORIGIN.md](README_ORIGIN.md)
- 底层架构：[docs/main.md](docs/main.md)
- 中文上游说明：[README_ZH.md](README_ZH.md)

`franka_description` 的默认双臂场景仅包含机器人和地面。实验物体及可调控制参数属于
`dualarm_task_bringup`，不再通过脚本覆盖本仓库的 `objects.xml`。
原项目教程保存在 `dualarm_mprc/docs/legacy/`；Git 历史中保留原始实现。

真正的子模块只有 `mujoco_ros_pkgs`。Franka、MuJoCo 库通过外部安装前缀提供；
已移除无 `.gitmodules` 配置且未使用的空 libfranka、mujoco、lodepng gitlink。
MuJoCo 子模块在 `srt_xzy` 上保留原有历史和已验证的拖动释放行为；任务动态障碍插件已迁入
`dualarm_mprc/dualarm_sim_plugins`。克隆时使用 `git submodule update --init --recursive`
检出父仓库锁定的版本，避免单独切换子模块到其他版本。
