"""障碍距离标签的绘制。

参考理想泊车界面的 "92cm / 83cm" 样式：在 BEV 车身边缘外侧放一个
黑底圆角药丸标签，并按距离分级换色。
"""

from __future__ import annotations

import math

import cv2
import numpy as np

import config
from core.models import RangeObservation, RangeState

ObstacleHit = RangeObservation

_SECTOR_LABELS: dict[str, str] = {
    "front": "前",
    "front_left": "左前",
    "left": "左",
    "rear_left": "左后",
    "rear": "后",
    "rear_right": "右后",
    "right": "右",
    "front_right": "右前",
}


def _to_bgr(color: tuple[int, int, int]) -> tuple[int, int, int]:
    """RGB 转 BGR（OpenCV 绘制需要）。"""
    return color[2], color[1], color[0]


def distance_color(distance: float) -> tuple[int, int, int]:
    if distance <= config.DISTANCE_NEAR_THRESHOLD_M:
        return config.DISTANCE_COLOR_NEAR
    if distance <= config.DISTANCE_MID_THRESHOLD_M:
        return config.DISTANCE_COLOR_MID
    return config.DISTANCE_COLOR_FAR


def _anchor(sector: str, stitcher, offset: float) -> tuple[int, int]:
    g = stitcher.geometry
    xmin, xmax, ymin, ymax = g.bounds
    x = (
        xmax + offset
        if "front" in sector
        else xmin - offset if "rear" in sector else g.body_center_x_m
    )
    y = (
        ymin - offset
        if "left" in sector
        else ymax + offset if "right" in sector else g.body_center_y_m
    )
    return stitcher.ground_to_canvas(x, y)


def draw_distance_labels(
    canvas,
    hits: list[RangeObservation],
    stitcher,
    max_range_m: float = config.OBSTACLE_LABEL_RANGE_M,
) -> None:
    for hit in hits:
        if hit.distance_m is not None and hit.distance_m > max_range_m:
            continue
        valid = hit.state == RangeState.VALID and hit.distance_m is not None
        if hit.distance_m is None:
            text = "--" if hit.state == RangeState.NO_RETURN else "N/A"
        else:
            text = f"{hit.distance_m:.1f}m" + ("*" if not valid else "")
        color = _to_bgr(distance_color(hit.distance_m)) if valid else (155, 155, 155)
        draw_pill(canvas, _anchor(hit.sector, stitcher, 0.65), text, color)


def draw_pill(
    canvas: np.ndarray,
    center: tuple[int, int],
    text: str,
    text_bgr: tuple[int, int, int],
    font_scale: float = config.DISTANCE_LABEL_FONT_SCALE,
    thickness: int = 1,
) -> None:
    """绘制黑底圆角药丸 + 居中文字。

    Args:
        canvas: 目标画布（BGR）。
        center: 药丸中心像素坐标。
        text: 文本内容。
        text_bgr: 文字颜色（BGR）。
        font_scale: OpenCV 字体缩放。
        thickness: 文字线宽。
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    pad_x = config.DISTANCE_LABEL_PAD_X_PX
    pad_y = config.DISTANCE_LABEL_PAD_Y_PX
    pill_w = text_w + pad_x * 2
    pill_h = text_h + baseline + pad_y * 2

    cx, cy = center
    left = cx - pill_w // 2
    top = cy - pill_h // 2

    height, width = canvas.shape[:2]
    if left + pill_w < 0 or top + pill_h < 0 or left > width or top > height:
        return

    x0, y0 = max(0, left), max(0, top)
    x1, y1 = min(width, left + pill_w + 1), min(height, top + pill_h + 1)
    if x0 >= x1 or y0 >= y1:
        return
    region = canvas[y0:y1, x0:x1]
    overlay = region.copy()
    radius = max(pill_h // 2, 1)
    lx, ty = left - x0, top - y0
    cv2.rectangle(
        overlay, (lx + radius, ty), (lx + pill_w - radius, ty + pill_h), (0, 0, 0), -1
    )
    cv2.circle(overlay, (lx + radius, ty + radius), radius, (0, 0, 0), -1)
    cv2.circle(overlay, (lx + pill_w - radius, ty + radius), radius, (0, 0, 0), -1)
    cv2.addWeighted(
        overlay,
        config.DISTANCE_LABEL_BG_ALPHA,
        region,
        1.0 - config.DISTANCE_LABEL_BG_ALPHA,
        0.0,
        dst=region,
    )

    cv2.putText(
        canvas,
        text,
        (left + pad_x, top + pad_y + text_h),
        font,
        font_scale,
        text_bgr,
        thickness,
        cv2.LINE_AA,
    )


def draw_proximity_arcs(
    canvas,
    hits: list[RangeObservation],
    stitcher,
    max_range_m: float = config.OBSTACLE_MAP_RANGE_M,
) -> None:
    for hit in hits:
        if (
            hit.state != RangeState.VALID
            or hit.distance_m is None
            or hit.distance_m > max_range_m
        ):
            continue
        u, v = _anchor(hit.sector, stitcher, 0.22)
        color = _to_bgr(distance_color(hit.distance_m))
        half = round(stitcher.meters_to_px(0.4))
        vertical = hit.sector in ("left", "right")
        start, end = (
            ((u, v - half), (u, v + half))
            if vertical
            else ((u - half, v), (u + half, v))
        )
        cv2.line(canvas, start, end, color, 4, cv2.LINE_AA)
        if hit.point:
            point = stitcher.ground_to_canvas(*hit.point[:2])
            cv2.circle(canvas, point, 4, color, -1, cv2.LINE_AA)


def sector_label(sector: str) -> str:
    """返回扇区的中文名。

    Args:
        sector: 扇区 key。

    Returns:
        中文标签；未知扇区原样返回。
    """
    return _SECTOR_LABELS.get(sector, sector)


def format_sector_summary(hits: list[RangeObservation]) -> str:
    valid = [
        h for h in hits if h.state == RangeState.VALID and h.distance_m is not None
    ]
    parts = [
        f"{sector_label(h.sector)} {h.distance_m:.1f}m"
        for h in sorted(valid, key=lambda h: h.distance_m)[:3]
    ]
    unknown = len(hits) - len(valid)
    if unknown:
        parts.append(f"{unknown}区无有效测距")
    return " / ".join(parts) if parts else "无有效测距"


__all__ = [
    "draw_distance_labels",
    "draw_proximity_arcs",
    "draw_pill",
    "sector_label",
    "format_sector_summary",
]
