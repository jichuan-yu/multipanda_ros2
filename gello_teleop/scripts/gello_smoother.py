import numpy as np


class Smoother:
    def __init__(
        self,
        publish_hz: float = 100.0,
        max_velocity: float = 0.1,
        position_tolerance: float = 0.05,
        joint_limits: np.ndarray = None,
        logger=None
    ):
        if joint_limits is None:
            joint_limits = np.array([
                [-2.8973, 2.8973],
                [-1.7628, 1.7628],
                [-2.8973, 2.8973],
                [-3.0718, -0.0698],
                [-2.8973, 2.8973],
                [-0.0175, 3.7525],
                [-2.8973, 2.8973],
            ], dtype=float)

        self.publish_period = 1.0 / publish_hz
        self.max_velocity = max_velocity
        self.position_tolerance = position_tolerance
        self.joint_limits = np.asarray(joint_limits, dtype=float)
        self.logger = logger

        # 原始目标（来自 GELLO 或键盘）
        self.q_target_raw = np.zeros(7, dtype=float)
        # 平滑后的命令（由 update() 自己演化，不会被实际位置覆盖）
        self.q_target_smooth = np.zeros(7, dtype=float)
        # 实际关节位置（仅存储，不干扰平滑输出）
        self.q_actual = np.zeros(7, dtype=float)

        # 状态标志
        self.follower_enabled = False
        self.target_received = False
        self.in_full_tracking = False

    # ---------- 位置设置 ----------
    def set_initial_position(self, initial_pos: np.ndarray):
        """初始化平滑器起点（通常在收到首次实际关节反馈时调用）"""
        pos = np.asarray(initial_pos, dtype=float)
        self.q_target_raw = pos.copy()
        self.q_target_smooth = pos.copy()
        self.q_actual = pos.copy()

    def set_actual_position(self, actual_pos: np.ndarray):
        """更新实际关节位置（仅记录，不影响平滑命令）"""
        self.q_actual = np.asarray(actual_pos, dtype=float)

    def set_target(self, target: np.ndarray):
        """设置原始目标（来自主端设备）"""
        self.q_target_raw = np.asarray(target, dtype=float)
        self.target_received = True

    def set_follower_enabled(self, enabled: bool):
        """使能或禁用平滑跟随"""
        self.follower_enabled = enabled

    # ---------- 核心更新 ----------
    def update(self) -> np.ndarray:
        """返回平滑后的关节命令"""
        if not self.target_received:
            return self.q_target_smooth.copy()

        if not self.follower_enabled:
            return self.q_target_smooth.copy()

        if self.in_full_tracking:
            self.q_target_smooth = self.q_target_raw.copy()
            return self.q_target_smooth.copy()

        error = self.q_target_raw - self.q_target_smooth
        error_norm = np.linalg.norm(error)

        # 全跟踪：误差足够小，直接锁定目标
        if error_norm < self.position_tolerance:
            self.in_full_tracking = True
            if self.logger:
                self.logger.info('Entering full tracking mode')
            self.q_target_smooth = self.q_target_raw.copy()
            return self.q_target_smooth.copy()

        # 匀速移动，最后一步直接到位
        step = self.max_velocity * self.publish_period
        move = np.zeros(7)
        for i in range(7):
            if abs(error[i]) < step:          # 不足一步，直接取误差值
                move[i] = error[i]
            elif abs(error[i]) > 1e-6:
                direction = np.sign(error[i])
                move[i] = direction * step

        self.q_target_smooth += move

        # 限位裁剪
        for i in range(7):
            self.q_target_smooth[i] = np.clip(
                self.q_target_smooth[i],
                self.joint_limits[i, 0],
                self.joint_limits[i, 1]
            )

        return self.q_target_smooth.copy()

    # ---------- 状态查询 ----------
    def current_mode(self) -> str:
        """返回当前工作模式（用于调试显示）"""
        if not self.target_received:
            return "idle"
        if not self.follower_enabled:
            return "waiting"

        error_norm = np.linalg.norm(self.q_target_raw - self.q_target_smooth)
        if error_norm < self.position_tolerance:
            return "full_tracking"
        return "tracking"