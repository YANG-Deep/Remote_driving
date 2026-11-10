"""行进轨迹辅助线绘制。

轨迹由 ``camera_math.trapezoid_curve`` 按 Ackermann 模型生成，
既画在 BEV 画布上，也通过单应逆变换投到主摄像头的透视画面上。
"""

from __future__ import annotations

import cv2
import numpy as np

import config
from utils import camera_math


def _color_lerp(
    near: tuple[int, int, int], far: tuple[int, int, int], ratio: float
) -> tuple[int, int, int]:
    """按比例在两个 RGB 颜色之间线性插值。

    Args:
        near: 起点颜色（RGB）。
        far: 终点颜色（RGB）。
        ratio: 插值比例，0 取 near，1 取 far。

    Returns:
        插值后的 RGB 颜色。
    """
    clamped = min(max(ratio, 0.0), 1.0)
    return tuple(
        int(round(near[index] + (far[index] - near[index]) * clamped))
        for index in range(3)
    )


def _to_bgr(color: tuple[int, int, int]) -> tuple[int, int, int]:
    """RGB 转 BGR（OpenCV 线框绘制需要）。"""
    return color[2], color[1], color[0]


def draw_trajectory_on_bev(
    canvas: np.ndarray,
    boundaries: list[list[tuple[float, float]]],
    stitcher,
    length_m: float = config.TRAJECTORY_LENGTH_M,
) -> None:
    """在 BEV 画布上绘制左右两条轨迹边界与距离刻度。

    Args:
        canvas: BEV 画布（BGR，原地修改）。
        boundaries: ``trapezoid_curve`` 返回的左右边界点序列（车辆坐标系）。
        stitcher: 提供 ``ground_to_canvas`` 的 ``BevStitcher``。
        length_m: 轨迹总长度（米），用于颜色渐变归一化。
    """
    if not boundaries or not boundaries[0]:
        return

    for boundary in boundaries:
        points_px = [stitcher.ground_to_canvas(x, y) for x, y in boundary]
        _draw_gradient_polyline(
            canvas, points_px, length_m=length_m, step_m=config.TRAJECTORY_STEP_M
        )

    _draw_distance_ticks(canvas, boundaries, stitcher, length_m)


def _draw_gradient_polyline(
    canvas: np.ndarray,
    points_px: list[tuple[int, int]],
    length_m: float,
    step_m: float,
) -> None:
    """逐段绘制带颜色渐变的折线。

    Args:
        canvas: 目标画布（BGR）。
        points_px: 折线的像素点序列。
        length_m: 折线代表的总长度（米）。
        step_m: 每个采样点间隔（米）。
    """
    for index in range(len(points_px) - 1):
        travelled = index * step_m
        ratio = travelled / max(length_m, 1e-6)
        color = _color_lerp(
            config.TRAJECTORY_NEAR_COLOR, config.TRAJECTORY_FAR_COLOR, ratio
        )
        cv2.line(
            canvas,
            points_px[index],
            points_px[index + 1],
            _to_bgr(color),
            thickness=config.TRAJECTORY_LINE_WIDTH_PX,
            lineType=cv2.LINE_AA,
        )


def _draw_distance_ticks(
    canvas: np.ndarray,
    boundaries: list[list[tuple[float, float]]],
    stitcher,
    length_m: float,
) -> None:
    """每隔固定距离画一条横跨两条边界的刻度线。

    Args:
        canvas: 目标画布（BGR）。
        boundaries: 左右边界点序列。
        stitcher: 提供坐标换算的 ``BevStitcher``。
        length_m: 轨迹总长度（米）。
    """
    if len(boundaries) < 2:
        return
    left, right = boundaries[0], boundaries[1]
    # 被画布裁剪后两条边界可能不等长，取较短者确定可用采样点
    usable = min(len(left), len(right))
    interval_m = config.TRAJECTORY_TICK_INTERVAL_M
    step_m = config.TRAJECTORY_STEP_M
    steps_per_tick = max(int(round(interval_m / step_m)), 1)

    tick_index = 1
    while True:
        sample = tick_index * steps_per_tick
        if sample >= usable:
            break
        start = stitcher.ground_to_canvas(*left[sample])
        end = stitcher.ground_to_canvas(*right[sample])
        ratio = (tick_index * interval_m) / max(length_m, 1e-6)
        color = _color_lerp(
            config.TRAJECTORY_NEAR_COLOR, config.TRAJECTORY_FAR_COLOR, ratio
        )
        cv2.line(canvas, start, end, _to_bgr(color), thickness=1, lineType=cv2.LINE_AA)
        tick_index += 1


def draw_trajectory_on_camera(
    image: np.ndarray,
    boundaries: list[list[tuple[float, float]]],
    camera_key: str,
    length_m: float = config.TRAJECTORY_LENGTH_M,
    *,
    mounts: dict | None = None,
    ground_z_m: float = 0.0,
) -> None:
    """把轨迹线投影到某路相机的透视画面上。

    只保留落在该相机画幅内的线段，避免画出画面外导致连线穿越。

    Args:
        image: 相机画面（BGR，原地修改）。
        boundaries: 左右边界点序列（车辆坐标系）。
        camera_key: 相机名，用于取安装位姿。
        length_m: 轨迹总长度（米）。
    """
    mount = (mounts or config.CAMERA_MOUNTS).get(camera_key)
    if mount is None:
        return
    location, rotation = mount
    intrinsics = camera_math.make_intrinsics(
        config.CAMERA_WIDTH_PX, config.CAMERA_HEIGHT_PX, config.CAMERA_FOV_DEG
    )

    for boundary in boundaries:
        pixels: list[tuple[int, int] | None] = []
        for x, y in boundary:
            point = camera_math.ground_to_pixel(
                x, y, intrinsics, location, rotation, ground_z_m=ground_z_m, clip=False
            )
            pixels.append(
                None
                if point is None
                else (
                    int(np.clip(round(point[0]), -1000000, 1000000)),
                    int(np.clip(round(point[1]), -1000000, 1000000)),
                )
            )

        for index in range(len(pixels) - 1):
            start, end = pixels[index], pixels[index + 1]
            if start is None or end is None:
                continue
            visible, start, end = cv2.clipLine(
                (0, 0, image.shape[1], image.shape[0]), start, end
            )
            if not visible:
                continue
            ratio = (index * config.TRAJECTORY_STEP_M) / max(length_m, 1e-6)
            color = _color_lerp(
                config.TRAJECTORY_NEAR_COLOR, config.TRAJECTORY_FAR_COLOR, ratio
            )
            cv2.line(
                image,
                start,
                end,
                _to_bgr(color),
                thickness=config.TRAJECTORY_LINE_WIDTH_PX,
                lineType=cv2.LINE_AA,
            )


def build_trajectory(
    geometry,
    steer_deg: float,
    reverse: bool,
    body_center_x_m: float = 0.0,
) -> list[list[tuple[float, float]]]:
    """按车辆几何与实时转角生成轨迹边界。

    轨迹从车身端部的保险杠处开始绘制（而不是从后轴冒出），
    完整保留米制轨迹，由各个视图独立裁剪。

    Args:
        geometry: ``VehicleGeometry``，提供轴距与半轮距。
        steer_deg: 前轮转角（度），正值右转。
        reverse: 是否倒车。
        body_center_x_m: 兼容旧调用；实际车身中心由 geometry 提供。

    Returns:
        行进方向保险杠两角的 actor 局部坐标序列。
    """
    del body_center_x_m  # Geometry contains the actual actor-local body centre.
    footprints = build_swept_footprints(geometry, steer_deg, reverse)
    indices = (3, 2) if reverse else (0, 1)
    return [[polygon[index] for polygon in footprints] for index in indices]


def build_swept_footprints(geometry, steer_deg: float, reverse: bool):
    """Rigid-body swept footprint sequence, including rear overhang swing."""
    from utils.trajectory import predict_footprints

    return predict_footprints(
        geometry,
        steer_deg,
        reverse,
        config.TRAJECTORY_LENGTH_M,
        config.TRAJECTORY_STEP_M,
    )


def _clip_to_bev_range(
    boundaries: list[list[tuple[float, float]]],
) -> list[list[tuple[float, float]]]:
    """把轨迹点裁剪到 BEV 画布覆盖的地面范围内。

    画布为 ±``BEV_RANGE_M`` 的方形，遇到第一个越界点即截断该边界
    （轨迹是单调延伸的，越界之后不会再回来）。

    Args:
        boundaries: 左右边界点序列。

    Returns:
        截断后的边界点序列；至少保留 2 个点，否则返回空列表。
    """
    limit = config.BEV_RANGE_M
    clipped: list[list[tuple[float, float]]] = []
    for boundary in boundaries:
        kept: list[tuple[float, float]] = []
        for x, y in boundary:
            if abs(x) > limit or abs(y) > limit:
                break
            kept.append((x, y))
        if len(kept) >= 2:
            clipped.append(kept)
    return clipped


def build_frame_overlays(
    geometry,
    steer_deg: float,
    image_reverse: bool,
    current_reverse: bool,
    image_age_s: float,
):
    """Never attach opposite-gear or expired predictions to an image group."""
    if image_reverse != current_reverse or image_age_s >= config.RADAR_STALE_S:
        return [], []
    footprints = build_swept_footprints(geometry, steer_deg, image_reverse)
    indices = (3, 2) if image_reverse else (0, 1)
    return [[polygon[index] for polygon in footprints] for index in indices], footprints
