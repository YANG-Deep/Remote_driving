"""车辆管理：spawn HGV、缓存车辆几何参数、键盘操控。

轴距等参数在 CARLA 里没有直接字段，需要从 VehiclePhysicsControl 推算。
``get_physics_control()`` 会真实查询模拟器，因此只在启动时调用一次并缓存。
"""

from __future__ import annotations

import math
import json
from dataclasses import replace
from pathlib import Path
from core.models import VehicleGeometry

import carla
import pygame

import config
from utils.transforms import inverse_transform_matrix


class VehicleManager:
    """负责 spawn 货车、读取实时转向角、把键盘输入转成 VehicleControl。"""

    def __init__(self, carla_manager) -> None:
        """在 ``CarlaManager`` 的世界中 spawn 货车并缓存几何参数。

        Args:
            carla_manager: 已初始化的 ``core.carla_manager.CarlaManager`` 实例。

        Raises:
            RuntimeError: 所有 spawn 尝试都失败。
        """
        self._world = carla_manager.world
        self.vehicle = self._spawn_vehicle()
        carla_manager.track_actor(self.vehicle)
        # spawn 后立刻 tick 一帧，让 actor 的世界位姿生效，
        # 否则 get_transform() 可能仍返回原点，导致几何推算错误。
        self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
        for settle_tick in range(60):
            carla_manager.tick()
            velocity = self.vehicle.get_velocity()
            if settle_tick >= 20 and abs(velocity.z) < 0.02:
                break
        self.geometry = self._read_geometry()
        profile_path = (
            Path(__file__).resolve().parents[1]
            / "calibration/european_hgv-3b938d7.json"
        )
        if profile_path.exists():
            profile = json.loads(profile_path.read_text())
            if (
                profile["vehicle"] == self.vehicle.type_id
                and profile["server_build"] == carla_manager.client.get_server_version()
            ):
                self.geometry = replace(
                    self.geometry,
                    curvature_samples=tuple(tuple(x) for x in profile["samples"]),
                    rear_slip_samples=tuple(
                        tuple(x) for x in profile["rear_slip_samples"]
                    ),
                )

        self._throttle = 0.0
        self._steer = 0.0
        self._brake = 0.0
        self._reverse = False

        print(
            f"[VehicleManager] 轴距 {self.geometry.wheelbase_m:.2f}m, "
            f"半轮距 {self.geometry.half_track_m:.2f}m, "
            f"最大转角 {self.geometry.max_steer_deg:.1f}°"
        )

    # ------------------------------------------------------------------
    # spawn
    # ------------------------------------------------------------------
    def _spawn_vehicle(self) -> carla.Vehicle:
        """按配置的蓝图 spawn 车辆。

        先尝试配置指定的出生点（含逐步抬高重试），失败后依次尝试其余
        出生点，避免因某个点被地图自带车辆占用而整体失败。

        Returns:
            spawn 成功的车辆 actor。

        Raises:
            RuntimeError: 全部出生点都失败。
        """
        blueprint_library = self._world.get_blueprint_library()
        blueprint = blueprint_library.filter(config.VEHICLE_BLUEPRINT)
        if not blueprint:
            available = [
                candidate.id for candidate in blueprint_library.filter("vehicle.*")
            ]
            raise RuntimeError(
                f"蓝图 {config.VEHICLE_BLUEPRINT} 不存在。可用示例: {available[:5]}"
            )
        vehicle_blueprint = blueprint[0]

        spawn_points = self._world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("当前地图没有任何 spawn point")

        count = len(spawn_points)
        preferred = config.VEHICLE_SPAWN_INDEX % count
        # 先试偏好点，再按顺序遍历其余点
        order = [preferred] + [i for i in range(count) if i != preferred]

        for index in order:
            base = spawn_points[index]
            for attempt in range(config.VEHICLE_SPAWN_RETRIES):
                transform = carla.Transform(
                    base.location
                    + carla.Location(
                        z=config.VEHICLE_Z_OFFSET_M
                        + attempt * config.VEHICLE_SPAWN_Z_STEP_M
                    ),
                    base.rotation,
                )
                vehicle = self._world.try_spawn_actor(vehicle_blueprint, transform)
                if vehicle is not None:
                    print(
                        f"[VehicleManager] 车辆已 spawn (id={vehicle.id}) @ "
                        f"出生点 {index} ({transform.location.x:.1f}, "
                        f"{transform.location.y:.1f})"
                    )
                    return vehicle
            if index == preferred:
                print(f"[VehicleManager] ⚠️ 出生点 {index} 被占用，改试其他出生点")

        raise RuntimeError("所有出生点都无法 spawn 货车，请检查地图是否被完全占满")

    # ------------------------------------------------------------------
    # 几何参数
    # ------------------------------------------------------------------
    def _read_geometry(self) -> VehicleGeometry:
        """读取物理参数并推算轴距、轮距。

        注意三个坑：

        1. ``WheelPhysicsControl.position`` 返回的是**世界坐标**（厘米），
           不是车辆局部坐标，需用车辆位姿逆变换转到局部。
        2. 货车（如 European HGV）有 6 个轮子（前轴 2 + 后双轴 4），
           不能假定恰好 4 个。转向轮由 ``max_steer_angle > 0`` 判定。
        3. 刚 spawn 尚未 tick 时 ``get_transform()`` 可能是全零位姿，
           必须在 tick 之后调用本方法（构造函数已保证）。

        Returns:
            填充好的 ``VehicleGeometry``。

        Raises:
            RuntimeError: 车辆位姿尚未就绪，或无法识别出前/后轴。
        """
        physics = self.vehicle.get_physics_control()
        wheels = physics.wheels
        if len(wheels) < 4:
            raise RuntimeError(f"车辆只有 {len(wheels)} 个轮子，无法推算轴距")

        transform = self.vehicle.get_transform()
        if abs(transform.location.x) < 1e-6 and abs(transform.location.y) < 1e-6:
            raise RuntimeError(
                "车辆位姿仍为原点，几何参数不可信；请确保在 world.tick() 之后读取"
            )

        local = self._wheels_to_local(wheels, transform)
        steerable = [
            point for point, wheel in zip(local, wheels) if wheel.max_steer_angle > 1.0
        ]
        if not steerable:
            raise RuntimeError("未识别到转向轮，无法推算轴距")

        front_axle_x = max(x for x, _ in steerable)
        rear_points = [
            x for (x, _), wheel in zip(local, wheels) if wheel.max_steer_angle <= 1.0
        ]
        rear_axle_x = sum(rear_points) / len(rear_points)
        wheelbase_m = abs(front_axle_x - rear_axle_x)
        half_track_m = max(abs(y) for x, y in steerable if abs(x - front_axle_x) < 0.05)
        max_steer_deg = max(float(w.max_steer_angle) for w in wheels)

        if wheelbase_m <= 0.5:
            raise RuntimeError(f"推算出的轴距异常: {wheelbase_m:.3f}m")
        if half_track_m <= 0.1:
            raise RuntimeError(f"推算出的半轮距异常: {half_track_m:.3f}m")
        # Wheel centres minus radii give the settled ground plane; bbox bottom is not contact ground.
        inverse = inverse_transform_matrix(transform)
        contacts = [
            (
                inverse
                @ [
                    w.position.x / 100.0,
                    w.position.y / 100.0,
                    w.position.z / 100.0,
                    1.0,
                ]
            )[2]
            - w.radius / 100.0
            for w in wheels
        ]
        ground_z = sum(contacts) / len(contacts)
        return VehicleGeometry(
            wheelbase_m=wheelbase_m,
            rear_collision_extension_m=(
                config.HGV_REAR_COLLISION_EXTENSION_M
                if self.vehicle.type_id == "vehicle.carlamotors.european_hgv"
                else 0.0
            ),
            half_track_m=half_track_m,
            max_steer_deg=max_steer_deg,
            rear_axle_x_m=rear_axle_x,
            body_center_x_m=float(self.vehicle.bounding_box.location.x),
            body_center_y_m=float(self.vehicle.bounding_box.location.y),
            half_length_m=float(self.vehicle.bounding_box.extent.x),
            half_width_m=float(self.vehicle.bounding_box.extent.y),
            height_m=float(self.vehicle.bounding_box.extent.z * 2),
            ground_z_m=float(ground_z),
        )

    @staticmethod
    def _wheels_to_local(wheels, transform) -> list[tuple[float, float]]:
        """把各轮子的世界坐标（厘米）换算成车辆局部坐标（米）。

        Args:
            wheels: ``VehiclePhysicsControl.wheels`` 序列。
            transform: 车辆当前世界位姿。

        Returns:
            与 ``wheels`` 等长的 ``(x, y)`` 局部坐标列表，单位米。
        """
        yaw = math.radians(transform.rotation.yaw)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        origin = transform.location

        local: list[tuple[float, float]] = []
        for wheel in wheels:
            delta_x = wheel.position.x / 100.0 - origin.x
            delta_y = wheel.position.y / 100.0 - origin.y
            local.append(
                (
                    delta_x * cosine + delta_y * sine,
                    -delta_x * sine + delta_y * cosine,
                )
            )
        return local

    # ------------------------------------------------------------------
    # 实时状态
    # ------------------------------------------------------------------
    def get_steer_angle_deg(self) -> float:
        """返回前轮真实物理转角（度），正值表示右转。"""
        return float(
            self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        )

    @property
    def is_reverse(self) -> bool:
        """当前是否处于倒车状态。"""
        return self._reverse

    def get_speed_kmh(self) -> float:
        """返回车速（km/h，带符号，倒车为负）。"""
        velocity = self.vehicle.get_velocity()
        speed = 3.6 * (velocity.x**2 + velocity.y**2) ** 0.5
        return -speed if self._reverse else speed

    def get_transform(self) -> carla.Transform:
        """返回车辆当前的世界位姿。"""
        return self.vehicle.get_transform()

    @property
    def body_center_x_m(self) -> float:
        """车身包围盒中心相对 actor 原点的 x 偏移，单位米。

        用于在 BEV 画布上把车身示意画在正确位置。
        """
        return float(self.vehicle.bounding_box.location.x)

    # ------------------------------------------------------------------
    # 键盘控制
    # ------------------------------------------------------------------
    def handle_key_event(self, event: pygame.event.Event) -> bool:
        """处理一次性按键事件（如切挡）。

        Args:
            event: pygame 事件。

        Returns:
            若事件已被消费返回 True。
        """
        if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
            if abs(self.get_speed_kmh()) > config.CONTROL_REVERSE_SPEED_EPS:
                return True
            self._reverse = not self._reverse
            self._throttle = 0.0
            self._brake = 0.0
            gear = "R" if self._reverse else "D"
            print(f"[VehicleManager] 切换挡位 → {gear}")
            return True
        return False

    def apply_keyboard(self, keys) -> None:
        """根据当前按住的键更新控制量并下发给车辆。

        Args:
            keys: ``pygame.key.get_pressed()`` 返回的按键状态序列。
        """
        # 转向：松手时向零回正，保持平滑
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            self._steer -= config.CONTROL_STEER_STEP
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            self._steer += config.CONTROL_STEER_STEP
        else:
            self._steer *= 0.85
        self._steer = float(max(-1.0, min(1.0, self._steer)))

        accelerating = keys[pygame.K_w] or keys[pygame.K_UP]
        decelerating = keys[pygame.K_s] or keys[pygame.K_DOWN]
        braking = keys[pygame.K_SPACE]

        if braking:
            self._throttle = 0.0
            self._brake = float(min(1.0, self._brake + config.CONTROL_BRAKE_STEP))
        elif accelerating:
            self._brake = 0.0
            self._throttle = float(
                min(1.0, self._throttle + config.CONTROL_THROTTLE_STEP)
            )
        elif decelerating:
            self._throttle = 0.0
            self._brake = float(min(1.0, self._brake + config.CONTROL_BRAKE_STEP))
        else:
            self._throttle = max(0.0, self._throttle - config.CONTROL_THROTTLE_STEP)
            self._brake = max(0.0, self._brake - config.CONTROL_BRAKE_STEP)

        control = carla.VehicleControl()
        control.throttle = self._throttle
        control.steer = self._steer
        control.brake = self._brake
        control.reverse = self._reverse
        control.hand_brake = False
        control.manual_gear_shift = False
        self.vehicle.apply_control(control)
