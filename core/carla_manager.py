"""CARLA 连接与世界管理：地图加载、同步模式、tick 驱动、资源清理。

本模块把「连接 + 世界设置 + 同步模式」收敛到一个上下文管理器里，
保证无论正常退出还是异常退出，都能销毁残留 actor 并还原世界设置。
"""

from __future__ import annotations


import carla

import config


class CarlaManager:
    """封装 CARLA 客户端与世界，负责同步模式下的 tick 循环。"""

    def __init__(
        self,
        host: str = config.CARLA_HOST,
        port: int = config.CARLA_PORT,
        map_name: str = config.MAP_NAME,
        timeout_s: float = config.CARLA_TIMEOUT_S,
        tick_hz: float = config.TICK_HZ,
    ) -> None:
        """连接 CARLA 并加载地图，随即切到同步模式。

        Args:
            host: CARLA 服务端地址。
            port: CARLA 服务端端口。
            map_name: 要加载的地图名，如 ``Town10HD``。
            timeout_s: 客户端超时时间（秒）。
            tick_hz: 同步模式下的仿真频率。

        Raises:
            RuntimeError: 无法连接到 CARLA 服务端或地图加载失败。
        """
        self._host = host
        self._port = port
        self._map_name = map_name
        self._tick_hz = tick_hz
        self._original_settings: carla.WorldSettings | None = None
        self._spawned_actors: list[carla.Actor] = []

        print(f"[CarlaManager] 连接 {host}:{port} ...")
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout_s)
        try:
            self.world = self.client.get_world()
        except RuntimeError as exc:
            raise RuntimeError(
                f"无法连接 CARLA 服务端 {host}:{port}，请先启动 CarlaUE4.sh"
            ) from exc

        current_map = self.world.get_map().name
        if map_name not in current_map:
            print(f"[CarlaManager] 加载地图 {map_name} ...")
            self.world = self.client.load_world(map_name)
        print(f"[CarlaManager] 当前地图: {self.world.get_map().name}")

        self._original_settings = self.world.get_settings()
        self._original_weather = (
            self.world.get_weather() if hasattr(self.world, "get_weather") else None
        )
        try:
            self._enable_synchronous_mode()
            self._apply_weather(config.WEATHER)
        except Exception:
            self.destroy()
            raise

    # ------------------------------------------------------------------
    # 世界设置
    # ------------------------------------------------------------------
    def _enable_synchronous_mode(self) -> None:
        """切换到同步模式并固定步长，保证多路传感器同帧。"""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / self._tick_hz
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)
        print(
            f"[CarlaManager] 同步模式已开启，{self._tick_hz:.0f} Hz "
            f"(delta={config.FIXED_DELTA_SECONDS:.4f}s)"
        )

    def _apply_weather(self, preset_name: str) -> None:
        """应用命名天气预设，找不到时保持当前天气。

        Args:
            preset_name: ``carla.WeatherParameters`` 上的预设属性名，如 ``ClearNoon``。
        """
        preset = getattr(carla.WeatherParameters, preset_name, None)
        if preset is None:
            print(f"[CarlaManager] ⚠️ 未知天气预设 {preset_name}，保持默认")
            return
        self.world.set_weather(preset)

    # ------------------------------------------------------------------
    # tick 与 actor 管理
    # ------------------------------------------------------------------
    def tick(self) -> int:
        """推进一帧同步仿真。

        Returns:
            本帧的 frame 编号。
        """
        return self.world.tick()

    def track_actor(self, actor: carla.Actor | None) -> carla.Actor | None:
        """登记需要在本管理器退出时销毁的 actor。

        Args:
            actor: 需要托管的 CARLA actor；None 会被忽略。

        Returns:
            原样返回该 actor，便于链式调用。
        """
        if actor is not None:
            self._spawned_actors.append(actor)
        return actor

    def destroy(self) -> None:
        """销毁全部托管的 actor 并还原世界设置（幂等）。"""
        for actor in reversed(self._spawned_actors):
            try:
                if actor is not None and actor.is_alive:
                    if actor.type_id.startswith("sensor."):
                        try:
                            actor.stop()
                        except RuntimeError:
                            pass
                    actor.destroy()
            except RuntimeError as exc:
                print(f"[CarlaManager] ⚠️ 销毁 actor 失败: {exc}")
        self._spawned_actors.clear()
        if getattr(self, "_original_weather", None) is not None:
            try:
                self.world.set_weather(self._original_weather)
            except RuntimeError as exc:
                print(f"[CarlaManager] weather restore failed: {exc}")
            self._original_weather = None

        if self._original_settings is not None:
            try:
                self.world.apply_settings(self._original_settings)
                print("[CarlaManager] 世界设置已还原")
            except RuntimeError as exc:
                print(f"[CarlaManager] ⚠️ 还原世界设置失败: {exc}")
            self._original_settings = None

    # ------------------------------------------------------------------
    # 上下文管理器
    # ------------------------------------------------------------------
    def __enter__(self) -> "CarlaManager":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.destroy()
