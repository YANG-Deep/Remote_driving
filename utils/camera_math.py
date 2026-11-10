"""相机几何工具：内参、CARLA 安装变换、IPM 单应矩阵与像素↔地面坐标换算。

坐标约定（全项目统一，与 CARLA 的左手系一致）：

* **车辆坐标系**：原点为车辆 actor 原点，+X 指向车头，+Y 指向车身右侧，
  +Z 向上。
* **CARLA 相机坐标系**：+X 向右、+Y 向下、+Z 向前（与 OpenCV 相同）。
* **BEV 像素坐标**：以画布中心为原点、等比例的俯视平面，u 向右（对应车辆 +Y），
  v 向下（对应车辆 -X，即车尾方向）。这样 BEV 画布上"上方=车头"。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """针孔相机内参矩阵及派生量。

    Attributes:
        width: 图像宽度（像素）。
        height: 图像高度（像素）。
        fov_deg: 水平视场角（度）。
        fx: x 方向焦距（像素）。
        fy: y 方向焦距（像素）。
        cx: 主点 x（像素）。
        cy: 主点 y（像素）。
    """

    width: int
    height: int
    fov_deg: float
    fx: float
    fy: float
    cx: float
    cy: float

    def matrix(self) -> np.ndarray:
        """返回 3x3 内参矩阵 K。"""
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


def make_intrinsics(width: int, height: int, fov_deg: float) -> CameraIntrinsics:
    """由图像尺寸与水平 FOV 构造内参。

    CARLA 的 `fov` 属性表示水平视场角，且像素为正方形，
    因此 fx == fy == (width / 2) / tan(fov / 2)。

    Args:
        width: 图像宽度（像素）。
        height: 图像高度（像素）。
        fov_deg: 水平视场角（度）。

    Returns:
        CameraIntrinsics 实例。
    """
    focal = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return CameraIntrinsics(
        width=width,
        height=height,
        fov_deg=fov_deg,
        fx=focal,
        fy=focal,
        cx=width / 2.0,
        cy=height / 2.0,
    )


def carla_rotation_to_basis(
    pitch_deg: float, yaw_deg: float, roll_deg: float = 0.0
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """将 CARLA 的欧拉角（度）转成车辆坐标系下的 (forward, right, up) 单位向量。

    与 CARLA `Transform.transform()` 的实际作用保持一致；部分构建的 get_matrix 绑定存在旋转方向差异：
    矩阵的三列依次为 forward、right、up。

    Args:
        pitch_deg: 俯仰角（度），负值表示朝下。
        yaw_deg: 偏航角（度）。
        roll_deg: 翻滚角（度）。

    Returns:
        (forward, right, up) 三元组，每个都是长度为 3 的 numpy 数组。
    """
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    roll = math.radians(roll_deg)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cr, sr = math.cos(roll), math.sin(roll)

    forward = np.array([cp * cy, cp * sy, sp], dtype=np.float64)
    right = np.array(
        [cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, -cp * sr], dtype=np.float64
    )
    up = np.array(
        [-cy * sp * cr - sy * sr, -sy * sp * cr + cy * sr, cp * cr], dtype=np.float64
    )
    return forward, right, up


def pixel_to_ground(
    pixel_u: float,
    pixel_v: float,
    intrinsics: CameraIntrinsics,
    mount_location: tuple[float, float, float],
    mount_rotation: tuple[float, float, float],
) -> np.ndarray | None:
    """把车辆坐标系下的相机像素反投影到地面平面 z=0。

    Args:
        pixel_u: 像素列坐标（向右为正）。
        pixel_v: 像素行坐标（向下为正）。
        intrinsics: 相机内参。
        mount_location: 相机在车辆坐标系中的安装位置 (x, y, z)，单位米。
        mount_rotation: 相机安装角 (pitch, yaw, roll)，单位度。

    Returns:
        地面交点 (x, y, 0.0)；若该像素射线朝向天空或地面在无穷远，返回 None。
    """
    forward, right, up = carla_rotation_to_basis(*mount_rotation)
    down = -up
    ray = (
        forward
        + right * ((pixel_u - intrinsics.cx) / intrinsics.fx)
        + down * ((pixel_v - intrinsics.cy) / intrinsics.fy)
    )
    if ray[2] >= -1e-6:
        return None
    scale = -mount_location[2] / ray[2]
    origin = np.array(mount_location, dtype=np.float64)
    hit = origin + ray * scale
    return np.array([hit[0], hit[1], 0.0], dtype=np.float64)


def ground_to_pixel(
    x: float,
    y: float,
    intrinsics: CameraIntrinsics,
    mount_location: tuple[float, float, float],
    mount_rotation: tuple[float, float, float],
    *,
    ground_z_m: float = 0.0,
    clip: bool = True,
) -> tuple[float, float] | None:
    """把车辆坐标系下的地面点投影到相机像素。

    Args:
        x: 车辆坐标系 x（米，+X 车头）。
        y: 车辆坐标系 y（米，+Y 车身右侧）。
        intrinsics: 相机内参。
        mount_location: 相机安装位置 (x, y, z)，单位米。
        mount_rotation: 相机安装角 (pitch, yaw, roll)，单位度。

    Returns:
        (u, v) 像素坐标；若点在相机后方或射线不落在画幅内，返回 None。
    """
    forward, right, up = carla_rotation_to_basis(*mount_rotation)
    down = -up
    delta = np.array([x, y, ground_z_m], dtype=np.float64) - np.array(
        mount_location, dtype=np.float64
    )
    depth = float(delta @ forward)
    if depth <= 0.02:
        return None
    u = intrinsics.cx + intrinsics.fx * float(delta @ right) / depth
    v = intrinsics.cy + intrinsics.fy * float(delta @ down) / depth
    if clip and not (0.0 <= u < intrinsics.width and 0.0 <= v < intrinsics.height):
        return None
    return u, v


def compute_ipm_homography(
    intrinsics: CameraIntrinsics,
    mount_location: tuple[float, float, float],
    mount_rotation: tuple[float, float, float],
    canvas_size_px: int,
    px_per_m: float,
    crop_bottom_ratio: float,
) -> tuple[np.ndarray, int]:
    """计算「裁剪后的相机图像 → 正射 BEV 画布」的单应矩阵。

    做法是取裁剪图像四角、反投影到地面、再映射到 BEV 像素坐标，
    用四组对应点解出 homography。这样得到的变换严格保持地面平面上的
    米制比例，不会因透视而拉伸。

    注意返回的矩阵作用在**裁剪后**的图像坐标上（左上角为原点），
    可直接交给 ``cv2.warpPerspective`` 处理裁剪图。

    Args:
        intrinsics: 相机内参。
        mount_location: 相机安装位置，单位米。
        mount_rotation: 相机安装角，单位度。
        canvas_size_px: BEV 画布边长（像素）。
        px_per_m: BEV 画布每米对应的像素数。
        crop_bottom_ratio: 参与 IPM 的图像下方高度比例，(0, 1]。

    Returns:
        ``(homography, crop_top_px)``：3x3 单应矩阵，以及裁剪起始行号。

    Raises:
        ValueError: 裁剪区的上边缘仍在地平线以上（无法构成有效的平面映射）。
    """
    crop_top = int(round(intrinsics.height * (1.0 - crop_bottom_ratio)))
    crop_bottom = float(intrinsics.height)
    # 在「裁剪后图像」坐标系里取四角，行坐标从 0 开始
    corners_cropped = [
        (0.0, 0.0),
        (float(intrinsics.width), 0.0),
        (float(intrinsics.width), crop_bottom - crop_top),
        (0.0, crop_bottom - crop_top),
    ]

    src: list[tuple[float, float]] = []
    dst: list[tuple[float, float]] = []
    for u, v_cropped in corners_cropped:
        v_original = v_cropped + crop_top
        ground = pixel_to_ground(
            u, v_original, intrinsics, mount_location, mount_rotation
        )
        if ground is None:
            raise ValueError(
                f"IPM 裁剪区上边缘（原始行 {v_original:.0f}）位于地平线以上，"
                f"请增大 crop_bottom_ratio 或减小俯仰角绝对值"
            )
        bu, bv = ground_to_bev_pixel(ground[0], ground[1], canvas_size_px, px_per_m)
        src.append((u, v_cropped))
        dst.append((bu, bv))

    src_arr = np.array(src, dtype=np.float32)
    dst_arr = np.array(dst, dtype=np.float32)
    return _solve_homography(src_arr, dst_arr), crop_top


def _solve_homography(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """用标准的四点 DLT 解单应矩阵（不依赖 OpenCV 即可单测）。"""
    a_rows = []
    for (x, y), (u, v) in zip(src, dst):
        a_rows.append([-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u])
        a_rows.append([0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v])
    matrix = np.array(a_rows, dtype=np.float64)
    _, _, vt = np.linalg.svd(matrix)
    homography = vt[-1].reshape(3, 3)
    if abs(homography[2, 2]) > 1e-12:
        homography /= homography[2, 2]
    return homography


def ground_to_bev_pixel(
    x: float, y: float, canvas_size_px: int, px_per_m: float
) -> tuple[float, float]:
    """把车辆坐标系地面点换算成 BEV 画布像素。

    BEV 画布以车辆 actor 原点为原点，+v 向下对应车尾方向（即 -X）。

    Args:
        x: 车辆坐标系 x（米，+X 车头）。
        y: 车辆坐标系 y（米，+Y 车身右侧）。
        canvas_size_px: BEV 画布边长（像素）。
        px_per_m: 每米像素数。

    Returns:
        (u, v) 画布像素坐标。
    """
    center = canvas_size_px / 2.0
    return center + y * px_per_m, center - x * px_per_m


def bev_pixel_to_ground(
    u: float, v: float, canvas_size_px: int, px_per_m: float
) -> tuple[float, float]:
    """``ground_to_bev_pixel`` 的逆变换。

    Args:
        u: BEV 画布列坐标。
        v: BEV 画布行坐标。
        canvas_size_px: BEV 画布边长（像素）。
        px_per_m: 每米像素数。

    Returns:
        (x, y) 车辆坐标系地面坐标，单位米。
    """
    center = canvas_size_px / 2.0
    return (center - v) / px_per_m, (u - center) / px_per_m


def trapezoid_curve(
    wheelbase_m: float,
    steer_deg: float,
    length_m: float,
    step_m: float,
    half_width_m: float,
    reverse: bool,
    start_offset_m: float = 0.0,
) -> list[list[tuple[float, float]]]:
    """按 Ackermann 自行车模型生成左右两条行进轨迹边界。

    符号约定（与 CARLA 一致）：

    * ``steer_deg > 0`` 表示**左转**（``control.steer > 0`` 会让 yaw 增大）。
    * 车辆坐标系 +X 朝车头、**+Y 朝车身右侧**。

    因此左转时后轴中心向 **-Y** 方向偏移。圆心在左侧，后轴轨迹为

    ``x = R·sin(θ), y = -R·(1 - cos(θ))``，其中 ``R = wheelbase / |tan δ|``。

    Args:
        wheelbase_m: 车辆轴距（米）。
        steer_deg: 前轮转角（度），正值左转。
        length_m: 轨迹延伸长度（米，不含起始偏移）。
        step_m: 采样步长（米）。
        half_width_m: 车身半宽（米），用于左右边界偏移。
        reverse: True 表示倒车，轨迹向后延伸。
        start_offset_m: 起始点沿行驶方向相对后轴的偏移（米）。

    Returns:
        长度为 2 的列表，依次为左边界、右边界的点序列；
        每个点是 ``(x, y)`` 车辆坐标系地面坐标。
    """
    if abs(steer_deg) < 1e-6:
        radius = math.inf
    else:
        radius = wheelbase_m / abs(math.tan(math.radians(steer_deg)))

    # 左转（steer>0）时后轴沿 -Y 偏移，故 y 方向取负
    lateral_sign = -1.0 if steer_deg > 0 else 1.0
    if math.isinf(radius):
        lateral_sign = 0.0

    direction = -1.0 if reverse else 1.0
    sample_count = max(int(round(length_m / step_m)), 1)

    centerline: list[tuple[float, float]] = []
    headings: list[float] = []
    for index in range(sample_count + 1):
        arc = direction * (start_offset_m + step_m * index)
        if math.isinf(radius):
            centerline.append((arc, 0.0))
            headings.append(0.0)
        else:
            angle = arc / radius
            centerline.append(
                (
                    radius * math.sin(angle),
                    lateral_sign * radius * (1.0 - math.cos(angle)),
                )
            )
            headings.append(angle)

    if math.isinf(radius):
        return [
            [(x, y - half_width_m) for x, y in centerline],
            [(x, y + half_width_m) for x, y in centerline],
        ]

    # 沿切向法向偏移，保证圆弧段边界严格等宽
    left: list[tuple[float, float]] = []
    right: list[tuple[float, float]] = []
    for (x, y), heading in zip(centerline, headings):
        # 转向未翻转时：+Y 为车身右侧，法向取切向顺时针旋转 90°
        normal_x = math.sin(heading) * lateral_sign
        normal_y = -math.cos(heading) * lateral_sign
        # 令法向指向车身左侧（用于"左边界"）
        left.append((x - normal_x * half_width_m, y - normal_y * half_width_m))
        right.append((x + normal_x * half_width_m, y + normal_y * half_width_m))
    return [left, right]
