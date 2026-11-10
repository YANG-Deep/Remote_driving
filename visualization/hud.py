"""pygame HUD：主窗口布局、图像贴图、状态栏与覆盖层合成。

布局分为三块：
    * 左侧：360° 正射俯视 BEV（含车模、距离圈、轨迹线、距离标签）
    * 右上：主摄像头（前进看前方、倒车看后方），叠加同一条轨迹线
    * 右下：左右两路相机常驻
    * 底部：状态栏（速度 / 挡位 / 转角 / 帧率 / 最近障碍摘要）

所有 CV 图像在这一层统一做 BGR→RGB 转换并转成 pygame Surface。
"""

from __future__ import annotations

import numpy as np
import pygame

import config
from perception.bev_stitcher import resize_contain
from utils.font_utils import FontRenderer
from visualization.distance_overlay import (
    draw_distance_labels,
    draw_proximity_arcs,
    format_sector_summary,
)
from visualization.trajectory_overlay import draw_trajectory_on_camera


class Hud:
    """负责整个 pygame 窗口的绘制。"""

    def __init__(
        self,
        width: int = config.WINDOW_WIDTH_PX,
        height: int = config.WINDOW_HEIGHT_PX,
        title: str = config.WINDOW_TITLE,
    ) -> None:
        """初始化 pygame 窗口与字体渲染器。

        Args:
            width: 窗口宽度（像素）。
            height: 窗口高度（像素）。
            title: 窗口标题。
        """
        pygame.init()
        pygame.display.set_caption(title)
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        self.fonts = FontRenderer()

        self._margin = config.HUD_MARGIN_PX
        content_top = config.HUD_HEADER_HEIGHT_PX + self._margin
        content_bottom = height - config.HUD_STATUSBAR_HEIGHT_PX - self._margin

        # 左右两栏等宽，BEV 是一个正方形，独占左栏
        column_width = round((width - self._margin * 3) * config.HUD_BEV_WIDTH_RATIO)
        self._bev_rect = pygame.Rect(
            self._margin, content_top, column_width, content_bottom - content_top
        )

        right_x = column_width + self._margin * 2
        right_w = width - right_x - self._margin
        content_height = content_bottom - content_top
        main_h = round(content_height * 0.64)
        self._main_rect = pygame.Rect(right_x, content_top, right_w, main_h)
        bottom_y = self._main_rect.bottom + self._margin
        side_w = (right_w - self._margin) // 2
        side_h = content_bottom - bottom_y
        self._left_rect = pygame.Rect(right_x, bottom_y, side_w, side_h)
        self._right_rect = pygame.Rect(
            right_x + side_w + self._margin, bottom_y, side_w, side_h
        )

    # ------------------------------------------------------------------
    # 绘制入口
    # ------------------------------------------------------------------
    def render(
        self,
        bev_image: np.ndarray,
        frames: dict[str, np.ndarray],
        trajectory: list[list[tuple[float, float]]],
        hits,
        status: dict[str, str],
        stitcher,
        reverse: bool,
        *,
        image_age_s: float = 0.0,
        overlay_hits=None,
    ) -> None:
        """绘制完整的一帧。

        Args:
            bev_image: 已包含车模/距离环的 BEV 画布（BGR）。
            frames: ``{相机名: RGB 数组}`` 的原始透视画面。
            trajectory: 左右边界点序列（车辆坐标系）。
            hits: 障碍测距结果列表。
            status: 状态栏键值对。
            stitcher: 提供坐标换算的 ``BevStitcher``。
            reverse: 是否倒车（决定主摄像头朝向）。
        """
        self._camera_mounts = stitcher.mounts
        self._ground_z_m = stitcher.geometry.ground_z_m
        self.screen.fill(config.HUD_BG_COLOR)
        self._draw_header(reverse=reverse)

        # BEV：先画提示弧与标签，再贴到屏幕
        image_fresh = image_age_s < config.RADAR_STALE_S
        if image_fresh:
            draw_proximity_arcs(
                bev_image, overlay_hits if overlay_hits is not None else hits, stitcher
            )
            draw_distance_labels(
                bev_image, overlay_hits if overlay_hits is not None else hits, stitcher
            )
        else:
            trajectory = []
        self._blit_array(bev_image, self._bev_rect)
        self._draw_panel_border(
            self._bev_rect,
            "360° 环视" if image_fresh else "影像已过期 · 实时测距见底栏",
        )

        main_key = "rear" if reverse else "front"
        main_label = "后视" if reverse else "前视"
        self._draw_camera_panel(
            frames, main_key, self._main_rect, main_label, trajectory
        )
        self._draw_camera_panel(frames, "left", self._left_rect, "左视", None)
        self._draw_camera_panel(frames, "right", self._right_rect, "右视", None)

        legend = self.fonts.render(
            "斜纹=不可见  --=无回波  N/A=不可用  *=过期", 15, (185, 185, 195)
        )
        self.screen.blit(legend, (self._bev_rect.x + 12, self._bev_rect.bottom - 25))
        self._draw_status_bar(status, hits)
        pygame.display.flip()

    # ------------------------------------------------------------------
    # 面板
    # ------------------------------------------------------------------
    def _draw_camera_panel(
        self,
        frames: dict[str, np.ndarray],
        key: str,
        rect: pygame.Rect,
        label: str,
        trajectory: list[list[tuple[float, float]]] | None,
    ) -> None:
        """绘制一路相机面板（含可选轨迹线叠加）。

        Args:
            frames: 相机帧字典。
            key: 相机名。
            rect: 面板在窗口中的位置。
            label: 面板标题（中文）。
            trajectory: 需要叠加的轨迹线；None 表示不叠加。
        """
        frame = frames.get(key)
        if frame is None:
            pygame.draw.rect(self.screen, config.HUD_PANEL_COLOR, rect)
            self._draw_panel_border(rect, label)
            return

        # CARLA 输出的 raw 前三个通道即 BGR 顺序，与 OpenCV 叠加约定一致
        canvas = frame.copy() if trajectory else frame
        if trajectory:
            draw_trajectory_on_camera(
                canvas,
                trajectory,
                key,
                mounts=self._camera_mounts,
                ground_z_m=self._ground_z_m,
            )
        pygame.draw.rect(self.screen, config.HUD_PANEL_COLOR, rect)
        self._blit_array(canvas, rect)
        self._draw_panel_border(rect, label)

    def _draw_panel_border(self, rect: pygame.Rect, title: str) -> None:
        """绘制圆角边框与左上角标题。

        Args:
            rect: 面板矩形。
            title: 标题文本。
        """
        pygame.draw.rect(
            self.screen, config.HUD_BORDER_COLOR, rect, width=1, border_radius=10
        )
        title_surface = self.fonts.render(title, 20, config.HUD_TEXT_COLOR)
        badge_w = title_surface.get_width() + 20
        badge_h = title_surface.get_height() + 10
        badge = pygame.Rect(rect.x + 12, rect.y + 12, badge_w, badge_h)
        overlay = pygame.Surface((badge_w, badge_h), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 130))
        self.screen.blit(overlay, badge.topleft)
        self.screen.blit(
            title_surface,
            (badge.x + 10, badge.y + (badge_h - title_surface.get_height()) // 2),
        )

    def _draw_header(self, reverse: bool) -> None:
        """绘制顶部标题与挡位指示。

        Args:
            reverse: 是否倒车。
        """
        title = self.fonts.render(config.WINDOW_TITLE, 24, config.HUD_TEXT_COLOR)
        self.screen.blit(
            title,
            (self._margin, (config.HUD_HEADER_HEIGHT_PX - title.get_height()) // 2),
        )

        gear_text = "R 倒车辅助" if reverse else "D 前进辅助"
        gear_color = config.HUD_WARN_COLOR if reverse else config.HUD_ACCENT_COLOR
        gear = self.fonts.render(gear_text, 22, gear_color)
        gear_badge_w = gear.get_width() + 28
        gear_badge_h = gear.get_height() + 12
        badge = pygame.Rect(
            self.width - self._margin - gear_badge_w,
            (config.HUD_HEADER_HEIGHT_PX - gear_badge_h) // 2,
            gear_badge_w,
            gear_badge_h,
        )
        pygame.draw.rect(self.screen, config.HUD_PANEL_COLOR, badge, border_radius=8)
        pygame.draw.rect(self.screen, gear_color, badge, width=1, border_radius=8)
        self.screen.blit(gear, (badge.x + 14, badge.y + 6))

    def _draw_status_bar(self, status: dict[str, str], hits) -> None:
        """绘制底部状态栏。

        Args:
            status: 状态键值对。
            hits: 测距结果，用于生成障碍摘要。
        """
        bar = pygame.Rect(
            self._margin,
            self.height - config.HUD_STATUSBAR_HEIGHT_PX - self._margin // 2,
            self.width - self._margin * 2,
            config.HUD_STATUSBAR_HEIGHT_PX,
        )
        pygame.draw.rect(self.screen, config.HUD_PANEL_COLOR, bar, border_radius=10)
        pygame.draw.rect(
            self.screen, config.HUD_BORDER_COLOR, bar, width=1, border_radius=10
        )

        chunks = [f"{name} {value}" for name, value in status.items()]
        chunks.append(f"障碍 {format_sector_summary(hits)}")
        text = "    |    ".join(chunks)
        size = 19
        surface = self.fonts.render(text, size, config.HUD_TEXT_COLOR)
        while surface.get_width() > bar.width - 36 and size > 11:
            size -= 1
            surface = self.fonts.render(text, size, config.HUD_TEXT_COLOR)
        self.screen.blit(
            surface,
            (bar.x + 18, bar.y + (bar.height - surface.get_height()) // 2),
        )

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    def _blit_array(self, image: np.ndarray, rect: pygame.Rect) -> None:
        """把 numpy 图像（BGR）等比填充并裁剪到指定矩形（object-fit: cover）。

        Args:
            image: ``(H, W, 3)`` BGR 图像。
            rect: 目标矩形。
        """
        if image.ndim != 3 or image.shape[2] != 3:
            return
        scaled = resize_contain(image, rect.width, rect.height)
        self.screen.blit(self._to_surface(scaled), rect.topleft)

    @staticmethod
    def _to_surface(bgr_image: np.ndarray) -> pygame.Surface:
        """把 BGR numpy 图像转成 pygame Surface。

        Args:
            bgr_image: ``(H, W, 3)`` BGR 图像。

        Returns:
            RGB 顺序的 pygame Surface。
        """
        return pygame.image.frombuffer(
            np.ascontiguousarray(bgr_image).tobytes(),
            (bgr_image.shape[1], bgr_image.shape[0]),
            "BGR",
        ).convert()

    def pump_events(self) -> list[pygame.event.Event]:
        """取出本帧所有 pygame 事件。

        Returns:
            事件列表。
        """
        return list(pygame.event.get())

    def tick(self, fps: float = config.TICK_HZ) -> float:
        """限制帧率并返回实际帧间隔（秒）。

        Args:
            fps: 目标帧率。

        Returns:
            距上一帧的间隔（秒）。
        """
        return self.clock.tick(int(fps)) / 1000.0

    def current_fps(self) -> float:
        """返回最近一帧的实际帧率。"""
        return self.clock.get_fps()

    def close(self) -> None:
        """关闭 pygame 窗口。"""
        pygame.quit()
