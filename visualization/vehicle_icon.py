"""BEV 画布上的半透明车身示意。

参考理想泊车界面：车身用深灰半透明填充 + 浅色描边，四角有轮位标记，
并被一圈距离环包围。
"""

from __future__ import annotations

import cv2
import numpy as np

import config


def draw_vehicle_icon(
    canvas: np.ndarray,
    center_px: tuple[int, int],
    length_px: float,
    width_px: float,
    fill_color: tuple[int, int, int] = (200, 205, 215),
    outline_color: tuple[int, int, int] = (245, 245, 250),
    alpha: float = 0.35,
) -> None:
    """在 BEV 画布中心绘制半透明车身（原地修改 canvas）。

    canvas 使用 BGR 顺序（与 OpenCV 一致），传入的 RGB 颜色会先转换。

    Args:
        canvas: 目标画布 ``(H, W, 3)`` uint8。
        center_px: 车身中心像素坐标 ``(u, v)``。
        length_px: 车身长度（像素，纵向）。
        width_px: 车身宽度（像素，横向）。
        fill_color: 填充色（RGB）。
        outline_color: 描边色（RGB）。
        alpha: 填充透明度，0 为全透明。
    """
    cx, cy = center_px
    half_l = length_px / 2.0
    half_w = width_px / 2.0
    top_left = (int(round(cx - half_w)), int(round(cy - half_l)))
    bottom_right = (int(round(cx + half_w)), int(round(cy + half_l)))

    overlay = canvas.copy()
    fill_bgr = (fill_color[2], fill_color[1], fill_color[0])
    outline_bgr = (outline_color[2], outline_color[1], outline_color[0])

    cv2.rectangle(overlay, top_left, bottom_right, fill_bgr, thickness=-1)
    cv2.addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0.0, dst=canvas)
    cv2.rectangle(
        canvas, top_left, bottom_right, outline_bgr, thickness=2, lineType=cv2.LINE_AA
    )

    _draw_wheel_marks(canvas, cx, cy, half_l, half_w, outline_bgr)
    _draw_heading_marker(canvas, cx, cy, half_l, outline_bgr)


def _draw_wheel_marks(
    canvas: np.ndarray,
    cx: float,
    cy: float,
    half_l: float,
    half_w: float,
    color: tuple[int, int, int],
) -> None:
    """在车身四角画轮位短标记，强化朝向感知。

    Args:
        canvas: 目标画布（BGR）。
        cx: 车身中心 x。
        cy: 车身中心 y。
        half_l: 车身半长（像素）。
        half_w: 车身半宽（像素）。
        color: 标记颜色（BGR）。
    """
    mark_len = max(half_l * 0.16, 6.0)
    inset = half_l * 0.62
    for sign_y in (-1.0, 1.0):
        for sign_x in (-1.0, 1.0):
            x = int(round(cx + sign_y * half_w))
            y = int(round(cy + sign_x * inset))
            cv2.line(
                canvas,
                (x, int(round(y - mark_len / 2))),
                (x, int(round(y + mark_len / 2))),
                color,
                thickness=3,
                lineType=cv2.LINE_AA,
            )


def _draw_heading_marker(
    canvas: np.ndarray,
    cx: float,
    cy: float,
    half_l: float,
    color: tuple[int, int, int],
) -> None:
    """在车头一端画一个三角箭头，指示前进方向。

    Args:
        canvas: 目标画布（BGR）。
        cx: 车身中心 x。
        cy: 车身中心 y。
        half_l: 车身半长（像素）。
        color: 颜色（BGR）。
    """
    tip_y = cy - half_l
    base_y = cy - half_l + max(half_l * 0.16, 8.0)
    half_base = max(half_l * 0.10, 5.0)
    points = np.array(
        [
            [int(round(cx)), int(round(tip_y))],
            [int(round(cx - half_base)), int(round(base_y))],
            [int(round(cx + half_base)), int(round(base_y))],
        ],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(canvas, points, color, lineType=cv2.LINE_AA)


def draw_distance_rings(
    canvas: np.ndarray,
    center_px: tuple[int, int],
    meters_to_px,
    radii_m: tuple[float, ...] = config.BEV_RING_RADII_M,
    color: tuple[int, int, int] = (120, 130, 150),
    thickness: int = 1,
) -> None:
    """在 BEV 画布上画一圈同心距离环（仅地面投影为圆的近似）。

    实际相机覆盖区呈矩形，这里用椭圆环更贴合画布比例。

    Args:
        canvas: 目标画布（BGR）。
        center_px: 圆心像素坐标。
        meters_to_px: 米 → 像素的换算函数。
        radii_m: 各环半径（米）。
        color: 环线颜色（RGB）。
        thickness: 线宽（像素）。
    """
    bgr = (color[2], color[1], color[0])
    for radius_m in radii_m:
        radius_px = meters_to_px(radius_m)
        if radius_px <= 1.0:
            continue
        cv2.circle(
            canvas,
            center_px,
            int(round(radius_px)),
            bgr,
            thickness=thickness,
            lineType=cv2.LINE_AA,
        )
