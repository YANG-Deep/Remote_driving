"""全局配置：CARLA 远程驾驶辅助 HUD 的全部可调参数。

所有模块统一从这里读取常量，避免魔法数字散落在实现代码中。
"""

from __future__ import annotations

import math
import os

# ---------------------------------------------------------------------------
# CARLA 连接
# ---------------------------------------------------------------------------
CARLA_HOST: str = "127.0.0.1"
CARLA_PORT: int = 2000
CARLA_TIMEOUT_S: float = 20.0
MAP_NAME: str = "Town10HD"
WEATHER: str = "ClearNoon"

TICK_HZ: float = 20.0
FIXED_DELTA_SECONDS: float = 1.0 / TICK_HZ

# ---------------------------------------------------------------------------
# 车辆
# 尺寸取自 CARLA 蓝图表 blueprint_sizes_vehicle.json 的 extent（半尺寸）：
# extent=[3.9679, 1.4455, 1.7310], origin=[0.0127, 0.0053, 1.7306]
# 因此车辆全长为 7.94m、全宽 2.89m、全高 3.46m。
# 轴距不硬编码，spawn 后从 VehiclePhysicsControl 读取。
# ---------------------------------------------------------------------------
VEHICLE_BLUEPRINT: str = "vehicle.carlamotors.european_hgv"
VEHICLE_HALF_LENGTH_M: float = 3.97
VEHICLE_HALF_WIDTH_M: float = 1.45
VEHICLE_HEIGHT_M: float = 3.46
VEHICLE_SPAWN_INDEX: int = 0
VEHICLE_SPAWN_RETRIES: int = 5
VEHICLE_SPAWN_Z_STEP_M: float = 0.2
VEHICLE_Z_OFFSET_M: float = 0.6

# 车辆控制
CONTROL_THROTTLE_STEP: float = 0.08
CONTROL_STEER_STEP: float = 0.05
CONTROL_BRAKE_STEP: float = 0.12
CONTROL_REVERSE_SPEED_EPS: float = 0.8

# ---------------------------------------------------------------------------
# 相机
#
# 安装参数由离线覆盖率搜索确定（见 README「安装参数推导」）：
# 以「车身矩形外扩 0.15m ~ 4.0m 的环带」为覆盖目标，扫描
# FOV ∈ {110,130,150,160} × 高度 ∈ {2.6,3.0,3.5} × pitch ∈ {-30,-45,-60}
# × 横向偏移 ∈ {1.55,2.0}，要求在 640×640 图像与 IPM 裁剪约束下无盲区。
#
# 结论：FOV=130 / 高度 3.0m / pitch=-45° / 前后相机伸出车体上缘 0.25m /
#       侧相机横向偏移 1.55m —— 环带覆盖率 100%（FOV 110 时仅 99.0%，
#       且边角出现空隙）。
#
# 相机必须挂在车体外（前/后沿 x 超出半车长），否则会被自车车顶遮挡。
# ---------------------------------------------------------------------------
CAMERA_WIDTH_PX: int = 960
CAMERA_HEIGHT_PX: int = 600
CAMERA_FOV_DEG: float = 130.0
CAMERA_MOUNT_HEIGHT_M: float = 3.0
CAMERA_PITCH_DEG: float = -45.0
CAMERA_LONG_OFFSET_M: float = 0.25  # 前/后相机超出车身半长的量
CAMERA_LATERAL_OFFSET_M: float = 1.55

_LONG_X = VEHICLE_HALF_LENGTH_M + CAMERA_LONG_OFFSET_M

# key -> (location(x 前, y 右, z 上), rotation(pitch, yaw, roll))
CAMERA_MOUNTS: dict[
    str, tuple[tuple[float, float, float], tuple[float, float, float]]
] = {
    "front": ((_LONG_X, 0.0, CAMERA_MOUNT_HEIGHT_M), (CAMERA_PITCH_DEG, 0.0, 0.0)),
    "rear": ((-_LONG_X, 0.0, CAMERA_MOUNT_HEIGHT_M), (CAMERA_PITCH_DEG, 180.0, 0.0)),
    "left": (
        (0.0, -CAMERA_LATERAL_OFFSET_M, CAMERA_MOUNT_HEIGHT_M),
        (CAMERA_PITCH_DEG, -90.0, 0.0),
    ),
    "right": (
        (0.0, CAMERA_LATERAL_OFFSET_M, CAMERA_MOUNT_HEIGHT_M),
        (CAMERA_PITCH_DEG, 90.0, 0.0),
    ),
}
CAMERA_KEYS: tuple[str, ...] = ("front", "rear", "left", "right")
CAMERA_QUEUE_SIZE: int = 2

# IPM 只使用图像下方这部分行，避免接近地平线的像素被无限拉伸
IPM_CROP_BOTTOM_RATIO: float = 0.60

# ---------------------------------------------------------------------------
# BEV 画布
# 货车全长 7.94m，±6m 画布只剩 2m 前余量，故取 ±9m（前后各留约 5m）。
# ---------------------------------------------------------------------------
BEV_RANGE_M: float = 9.0
BEV_SIZE_PX: int = 620
BEV_PX_PER_M: float = BEV_SIZE_PX / (2.0 * BEV_RANGE_M)
BEV_EMPTY_COLOR: tuple[int, int, int] = (26, 26, 30)
BEV_FEATHER_PX: float = 45.0

# ---------------------------------------------------------------------------
# 轨迹线（Ackermann 圆弧模型）
# 轨迹长度只需覆盖 BEV 画布可见范围：画布 ±9m，车身半长 3.97m，
# 因此从后轴算起最多约 12m 即到画布边缘；叠加层会再按画布裁剪。
# ---------------------------------------------------------------------------
TRAJECTORY_LENGTH_M: float = 12.0
TRAJECTORY_STEP_M: float = 0.2
TRAJECTORY_TICK_INTERVAL_M: float = 1.0
TRAJECTORY_STRAIGHT_THRESHOLD_DEG: float = 1.0
TRAJECTORY_NEAR_COLOR: tuple[int, int, int] = (245, 245, 245)
TRAJECTORY_FAR_COLOR: tuple[int, int, int] = (255, 170, 40)
TRAJECTORY_LINE_WIDTH_PX: int = 3

# ---------------------------------------------------------------------------
# 障碍测距
# ---------------------------------------------------------------------------
OBSTACLE_LABELS: tuple[int, ...] = (
    14,  # CityObjectLabel.Car
    15,  # CityObjectLabel.Truck
    16,  # CityObjectLabel.Bus
    18,  # CityObjectLabel.Motorcycle
    19,  # CityObjectLabel.Bicycle
)
OBSTACLE_RAY_COUNT: int = 24
OBSTACLE_MAX_RANGE_M: float = 8.0
OBSTACLE_LABEL_RANGE_M: float = 4.0
OBSTACLE_RAY_HEIGHT_M: float = 1.2
OBSTACLE_START_CLEARANCE_M: float = 0.35
OBSTACLE_UPDATE_INTERVAL_FRAMES: int = 2

# 8 个扇区，角度以车辆 +X 前进方向为 0°，逆时针为正（世界坐标系下）
OBSTACLE_SECTORS: tuple[tuple[str, float], ...] = (
    ("front", 0.0),
    ("front_left", 45.0),
    ("left", 90.0),
    ("rear_left", 135.0),
    ("rear", 180.0),
    ("rear_right", 225.0),
    ("right", 270.0),
    ("front_right", 315.0),
)
OBSTACLE_SECTOR_HALF_WIDTH_DEG: float = 22.5

# 距离分级配色（远 -> 近）
DISTANCE_COLOR_FAR: tuple[int, int, int] = (225, 225, 225)
DISTANCE_COLOR_MID: tuple[int, int, int] = (255, 200, 60)
DISTANCE_COLOR_NEAR: tuple[int, int, int] = (255, 90, 40)
DISTANCE_MID_THRESHOLD_M: float = 2.0
DISTANCE_NEAR_THRESHOLD_M: float = 1.0

# BEV 距离圈
BEV_RING_RADII_M: tuple[float, ...] = (1.0, 2.0, 3.0, 5.0)

# 距离标签样式
DISTANCE_LABEL_OFFSET_M: float = 0.9  # 标签离车身轮廓向外的偏移
DISTANCE_LABEL_FONT_SCALE: float = 0.46
DISTANCE_LABEL_PAD_X_PX: int = 8
DISTANCE_LABEL_PAD_Y_PX: int = 5
DISTANCE_LABEL_BG_ALPHA: float = 0.72

# 车身轮廓上的近距提示弧
OBSTACLE_MAP_RANGE_M: float = 4.0  # 超过该距离不画提示弧
OBSTACLE_ARC_OFFSET_M: float = 0.25  # 提示弧离车身轮廓的距离

# ---------------------------------------------------------------------------
# HUD 布局（1920x1080）
# ---------------------------------------------------------------------------
WINDOW_WIDTH_PX: int = 1920
WINDOW_HEIGHT_PX: int = 1080
WINDOW_TITLE: str = "CARLA 远程驾驶辅助 - 360环视 / 轨迹 / 距离"

HUD_BG_COLOR: tuple[int, int, int] = (18, 18, 22)
HUD_PANEL_COLOR: tuple[int, int, int] = (30, 30, 36)
HUD_BORDER_COLOR: tuple[int, int, int] = (70, 70, 82)
HUD_TEXT_COLOR: tuple[int, int, int] = (232, 232, 236)
HUD_ACCENT_COLOR: tuple[int, int, int] = (58, 130, 246)
HUD_WARN_COLOR: tuple[int, int, int] = (255, 170, 40)
HUD_MARGIN_PX: int = 16
HUD_HEADER_HEIGHT_PX: int = 44
HUD_STATUSBAR_HEIGHT_PX: int = 46
HUD_BEV_SIZE_PX: int = 620
HUD_MAIN_BOTTOM_HEIGHT_PX: int = 380

# ---------------------------------------------------------------------------
# 字体
# ---------------------------------------------------------------------------
_FONT_CANDIDATES: tuple[str, ...] = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
)


def resolve_cjk_font_path() -> str | None:
    """返回第一个存在的 CJK 字体文件路径，找不到时返回 None。"""
    for path in _FONT_CANDIDATES:
        if os.path.exists(path):
            return path
    return None


FONT_PATH: str | None = resolve_cjk_font_path()

# ---------------------------------------------------------------------------
# 调试 / 输出
# ---------------------------------------------------------------------------
DEBUG_OUTPUT_DIR: str = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "_debug_out"
)
DEBUG_SAVE_FIRST_FRAME: bool = False

# 便于其他模块直接使用的换算常量
DEG2RAD: float = math.pi / 180.0
RAD2DEG: float = 180.0 / math.pi

# Radar measurements are the sole production range source.
RADAR_HORIZONTAL_FOV_DEG: float = 170.0
RADAR_VERTICAL_FOV_DEG: float = 35.0
RADAR_POINTS_PER_SECOND: int = 12000
RADAR_RANGE_M: float = 12.0
RADAR_MOUNT_HEIGHT_M: float = 0.8
RADAR_MIN_HEIGHT_M: float = 0.15
RADAR_STALE_S: float = 0.2
RADAR_EXPIRE_S: float = 0.5
SENSOR_WAIT_S: float = 0.05
SENSOR_FRAME_BUFFER: int = 8
HUD_BEV_WIDTH_RATIO: float = 0.34
VALIDATION_INTERVAL_FRAMES: int = 10

# Measured HGV collision hull protrudes beyond nominal rear bbox; avoid self returns.
RADAR_REAR_CLEARANCE_M: float = 0.15
BEV_RENDER_WIDTH_PX: int = 400

# Conservative rear collision envelope, calibrated in isolated live HGV radar sweep.
HGV_REAR_COLLISION_EXTENSION_M: float = 0.55
