"""障碍物测距：绕车身发射水平射线，取最近命中距离。

使用 ``world.cast_ray`` 做几何求交，不依赖光照或材质，比深度相机稳定。
射线从车身包围盒外缘出发，避免命中自车。

坐标约定：射线的方位角以车辆 +X（车头）为 0°，逆时针为正，
即在车辆局部平面上 ``方向 = (cos θ, sin θ)``，与 ``camera_math`` 一致。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import carla

import config


@dataclass(frozen=True)
class ObstacleHit:
    """一次射线命中的结果。

    Attributes:
        sector: 命中所属的扇区名（front/front_left/...）。
        distance_m: 从射线起点到命中点的距离（米）。
        bearing_deg: 射线在车辆坐标系中的方位角（度，车头为 0）。
    """

    sector: str
    distance_m: float
    bearing_deg: float


class ObstacleSensor:
    """基于环形射线投射的障碍测距器。"""

    def __init__(
        self, world: carla.World, ray_count: int = config.OBSTACLE_RAY_COUNT
    ) -> None:
        """初始化测距器。

        Args:
            world: CARLA 世界对象。
            ray_count: 绕车身一圈的射线数量。
        """
        self._world = world
        self._ray_count = ray_count
        self._frame_counter = 0
        self._last_hits: list[ObstacleHit] = []

    # ------------------------------------------------------------------
    # 射线起点
    # ------------------------------------------------------------------
    def _ray_origins(self) -> list[tuple[float, float, float]]:
        """计算各射线在车辆局部坐标系中的起点。

        起点位于车身包围盒外缘再外扩 ``OBSTACLE_START_CLEARANCE_M``，
        这样射线不会从车体内部出发而命中自车。

        Returns:
            与射线方向一一对应的局部坐标 ``(x, y, 0)``；x/y 单位米。
        """
        clearance = config.OBSTACLE_START_CLEARANCE_M
        half_length = config.VEHICLE_HALF_LENGTH_M + clearance
        half_width = config.VEHICLE_HALF_WIDTH_M + clearance

        origins: list[tuple[float, float, float]] = []
        for index in range(self._ray_count):
            bearing = 360.0 * index / self._ray_count
            cosine = math.cos(math.radians(bearing))
            sine = math.sin(math.radians(bearing))
            # 从车身中心沿该方向走到矩形边界（比例形式的射线-矩形求交）
            scale = min(
                half_length / max(abs(cosine), 1e-6),
                half_width / max(abs(sine), 1e-6),
            )
            origins.append((cosine * scale, sine * scale, 0.0))
        return origins

    def _to_world(
        self, transform: carla.Transform, local_x: float, local_y: float, local_z: float
    ) -> carla.Location:
        """把车辆局部坐标变换到世界坐标。

        Args:
            transform: 车辆当前位姿。
            local_x: 局部 x（米，+X 车头方向）。
            local_y: 局部 y（米，+Y 车身右侧）。
            local_z: 相对地面的高度（米）。

        Returns:
            世界坐标 Location。
        """
        yaw = math.radians(transform.rotation.yaw)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        world_x = transform.location.x + local_x * cosine - local_y * sine
        world_y = transform.location.y + local_x * sine + local_y * cosine
        world_z = transform.location.z + local_z
        return carla.Location(x=world_x, y=world_y, z=world_z)

    # ------------------------------------------------------------------
    # 测距主流程
    # ------------------------------------------------------------------
    def update(
        self, transform: carla.Transform, force: bool = False
    ) -> list[ObstacleHit]:
        """按间隔执行一次环形测距。

        Args:
            transform: 车辆当前位姿。
            force: True 时忽略更新间隔，立即测距。

        Returns:
            每个扇区一条记录（无命中则不出现），``distance_m`` 为该扇区最小值。
        """
        self._frame_counter += 1
        if not force and self._frame_counter % config.OBSTACLE_UPDATE_INTERVAL_FRAMES:
            return self._last_hits

        raw_hits = self._cast_all_rays(transform)
        self._last_hits = self._aggregate_sectors(raw_hits)
        return self._last_hits

    def _cast_all_rays(self, transform: carla.Transform) -> list[tuple[float, float]]:
        """发射全部射线，返回 (方位角, 距离) 列表。

        方位角在**车辆局部坐标系**中定义：0° 指向车头（+X），
        顺时针为正（+90° 指向车身右侧 +Y），与 ``config.OBSTACLE_SECTORS``
        以及 ``camera_math`` 的 +Y=右侧约定一致。

        Args:
            transform: 车辆当前位姿。

        Returns:
            命中的 ``(方位角, 距离)`` 对；未命中该方向的射线不出现。
        """
        ray_height = config.OBSTACLE_RAY_HEIGHT_M
        origins = self._ray_origins()
        hits: list[tuple[float, float]] = []

        for index, (local_x, local_y, _) in enumerate(origins):
            bearing = 360.0 * index / self._ray_count
            cosine = math.cos(math.radians(bearing))
            sine = math.sin(math.radians(bearing))
            origin = self._to_world(transform, local_x, local_y, ray_height)
            # 沿同一方位角再向外延伸 MAX_RANGE 米
            target = self._to_world(
                transform,
                local_x + cosine * config.OBSTACLE_MAX_RANGE_M,
                local_y + sine * config.OBSTACLE_MAX_RANGE_M,
                ray_height,
            )

            hit_points = self._world.cast_ray(origin, target)
            for hit in hit_points:
                if hit.label not in config.OBSTACLE_LABELS:
                    continue
                distance = origin.distance(hit.location)
                if distance <= 1e-3:
                    continue
                hits.append((bearing, distance))
                break
        return hits

    def _aggregate_sectors(
        self, raw_hits: list[tuple[float, float]]
    ) -> list[ObstacleHit]:
        """把逐射线结果按扇区聚合，取每扇区最小距离。

        Args:
            raw_hits: ``(方位角, 距离)`` 列表，方位角顺时针对应车身右侧。

        Returns:
            每个有命中的扇区一条 ``ObstacleHit``，按扇区定义顺序排列。
        """
        best: dict[str, float] = {}
        half_width = config.OBSTACLE_SECTOR_HALF_WIDTH_DEG

        for bearing, distance in raw_hits:
            if distance > config.OBSTACLE_MAX_RANGE_M:
                continue
            for sector_name, sector_bearing in config.OBSTACLE_SECTORS:
                delta = (bearing - sector_bearing + 180.0) % 360.0 - 180.0
                if abs(delta) <= half_width:
                    current = best.get(sector_name)
                    if current is None or distance < current:
                        best[sector_name] = distance
                    break

        results: list[ObstacleHit] = []
        for sector_name, sector_bearing in config.OBSTACLE_SECTORS:
            if sector_name not in best:
                continue
            results.append(
                ObstacleHit(
                    sector=sector_name,
                    distance_m=best[sector_name],
                    bearing_deg=sector_bearing,
                )
            )
        return results

        results: list[ObstacleHit] = []
        for sector_name, sector_bearing in config.OBSTACLE_SECTORS:
            if sector_name not in best:
                continue
            results.append(
                ObstacleHit(
                    sector=sector_name,
                    distance_m=best[sector_name][1],
                    bearing_deg=sector_bearing,
                )
            )
        return results

    @staticmethod
    def distance_color(distance_m: float) -> tuple[int, int, int]:
        """按距离返回提示色（远=白、中=黄、近=橙红）。

        Args:
            distance_m: 到障碍物的距离（米）。

        Returns:
            RGB 颜色。
        """
        if distance_m <= config.DISTANCE_NEAR_THRESHOLD_M:
            return config.DISTANCE_COLOR_NEAR
        if distance_m <= config.DISTANCE_MID_THRESHOLD_M:
            return config.DISTANCE_COLOR_MID
        return config.DISTANCE_COLOR_FAR

    @staticmethod
    def format_distance(distance_m: float) -> str:
        """把米制距离格式化成参考图那样的整数厘米标签。

        Args:
            distance_m: 距离（米）。

        Returns:
            如 ``92cm``。
        """
        return f"{int(round(distance_m * 100))}cm"
