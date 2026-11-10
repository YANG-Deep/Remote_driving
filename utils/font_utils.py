"""中文字体工具：用 Pillow 渲染文本并转为 pygame Surface，带缓存。

pygame 自带的 ``font`` 模块无法稳定渲染中文，因此这里统一走
Pillow 的 FreeType 后端：先渲染成 RGBA 位图，再包装成 pygame Surface。
渲染结果按 (文本, 字号, 颜色) 缓存，避免每帧重复光栅化。
"""

from __future__ import annotations

from collections import OrderedDict

import numpy as np
import pygame
from PIL import Image, ImageDraw, ImageFont

import config


class FontRenderer:
    """Pillow 文本 → pygame Surface 的渲染器，带 LRU 缓存。"""

    def __init__(self, font_path: str | None = None, cache_size: int = 512) -> None:
        """初始化渲染器。

        Args:
            font_path: TrueType/OpenType 字体路径；None 时使用 config 中解析出的字体。
            cache_size: 缓存的文本纹理数量上限。
        """
        self._font_path = font_path or config.FONT_PATH
        self._fonts: dict[int, ImageFont.FreeTypeFont] = {}
        self._cache: OrderedDict[
            tuple[str, int, tuple[int, int, int]], pygame.Surface
        ] = OrderedDict()
        self._cache_size = cache_size
        if self._font_path is None:
            print("[FontRenderer] ⚠️ 未找到中文字体，中文可能显示为方块")

    def _get_font(self, size: int) -> ImageFont.FreeTypeFont:
        """按字号缓存 Pillow 字体对象。"""
        font = self._fonts.get(size)
        if font is None:
            if self._font_path is None:
                font = ImageFont.load_default()
            else:
                font = ImageFont.truetype(self._font_path, size)
            self._fonts[size] = font
        return font

    def render(
        self, text: str, size: int, color: tuple[int, int, int] = config.HUD_TEXT_COLOR
    ) -> pygame.Surface:
        """渲染一行文本为带 alpha 的 pygame Surface。

        Args:
            text: 待渲染文本（支持中文）。
            size: 字号（像素）。
            color: RGB 颜色。

        Returns:
            pygame Surface，已 ``convert_alpha()``，可直接 blit。
        """
        cache_key = (text, size, color)
        cached = self._cache.get(cache_key)
        if cached is not None:
            self._cache.move_to_end(cache_key)
            return cached

        font = self._get_font(size)
        # 先取文本包围盒，避免生成过大的空白纹理
        left, top, right, bottom = font.getbbox(text)
        width = max(right - left, 1)
        height = max(bottom - top, 1)
        image = Image.new("RGBA", (width, height), (0, 0, 0, 0))
        draw = ImageDraw.Draw(image)
        draw.text((-left, -top), text, font=font, fill=(*color, 255))

        rgba = np.array(image, dtype=np.uint8)
        surface = pygame.image.frombuffer(
            rgba.tobytes(), (width, height), "RGBA"
        ).convert_alpha()

        self._cache[cache_key] = surface
        if len(self._cache) > self._cache_size:
            self._cache.popitem(last=False)
        return surface

    def measure(self, text: str, size: int) -> tuple[int, int]:
        """返回文本渲染后的像素尺寸，无需真正生成纹理。

        Args:
            text: 待测量文本。
            size: 字号（像素）。

        Returns:
            (宽度, 高度)。
        """
        surface = self.render(text, size)
        return surface.get_width(), surface.get_height()
