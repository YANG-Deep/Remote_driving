"""Four-radar ranging; independent validation is deliberately not imported here."""

from __future__ import annotations

import queue
from dataclasses import dataclass, replace
import numpy as np
from utils.transforms import transform_matrix, inverse_transform_matrix
import config
from core.models import VehicleGeometry, RangeObservation, RangeState, SECTOR_BEARINGS


def radar_points_to_vehicle(
    detections, capture_matrix: np.ndarray, current_inverse: np.ndarray
) -> np.ndarray:
    """Polar returns -> capture-world -> requested vehicle pose, metres."""
    # Named attributes avoid relying on version-specific raw buffer ordering.
    polar = np.asarray(
        [(d.depth, d.azimuth, d.altitude) for d in detections], dtype=float
    ).reshape(-1, 3)
    polar = polar[
        np.isfinite(polar).all(axis=1)
        & (polar[:, 0] > 0.0)
        & (polar[:, 0] <= config.RADAR_RANGE_M)
    ]
    depth, azimuth, altitude = polar.T
    horizontal = depth * np.cos(altitude)
    local = np.column_stack(
        (
            horizontal * np.cos(azimuth),
            horizontal * np.sin(azimuth),
            depth * np.sin(altitude),
            np.ones(len(polar)),
        )
    )
    return (local @ (current_inverse @ capture_matrix).T)[:, :3]


def align_observation_points(observations, source_pose, image_pose):
    """Align marker positions to image pose while retaining current measured clearances."""
    matrix = inverse_transform_matrix(image_pose) @ transform_matrix(source_pose)
    return [
        (
            replace(obs, point=tuple((matrix @ [*obs.point, 1.0])[:3]))
            if obs.point is not None
            else obs
        )
        for obs in observations
    ]


@dataclass(frozen=True)
class _Packet:
    points: np.ndarray  # current actor coordinates, metres
    frame: int
    timestamp: float
    nearest: dict[str, tuple[float, np.ndarray]]


class RadarTracker:
    """Aggregate nearest detected surface points; no return never means safe."""

    def __init__(self, geometry: VehicleGeometry, keys: tuple[str, ...]):
        self.geometry = geometry
        self.keys = keys
        self._packets: dict[str, _Packet] = {}

    def update(
        self, key: str, points: np.ndarray, frame: int, timestamp: float
    ) -> None:
        old = self._packets.get(key)
        if old and frame < old.frame:
            return
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        finite = np.isfinite(points).all(axis=1)
        height = points[:, 2] - self.geometry.ground_z_m
        useful = (
            finite
            & (height >= config.RADAR_MIN_HEIGHT_M)
            & (height <= self.geometry.height_m)
        )
        points = points[useful]
        nearest = {}
        for point in points:
            distance = self.geometry.clearance(*point[:2])
            if distance <= 0.02:
                continue
            sector = self.geometry.sector(*point[:2])
            if sector not in nearest or distance < nearest[sector][0]:
                nearest[sector] = (distance, point)
        self._packets[key] = _Packet(points, frame, timestamp, nearest)

    def observations(self, now: float) -> dict[str, RangeObservation]:
        result = {}
        for sector in SECTOR_BEARINGS:
            owners = [key for key in self.keys if key in sector.split("_")]
            packets = [self._packets[key] for key in owners if key in self._packets]
            candidates = []
            for packet in packets:
                age = max(0.0, now - packet.timestamp)
                if age >= config.RADAR_EXPIRE_S:
                    continue
                nearest = packet.nearest.get(sector)
                if nearest:
                    candidates.append((nearest[0], packet, nearest[1]))
            if candidates:
                distance, packet, point = min(candidates, key=lambda entry: entry[0])
                age = max(0.0, now - packet.timestamp)
                state = (
                    RangeState.STALE
                    if age >= config.RADAR_STALE_S
                    else RangeState.VALID
                )
                result[sector] = RangeObservation(
                    sector,
                    distance,
                    state,
                    packet.timestamp,
                    packet.frame,
                    tuple(point),
                    age_s=age,
                )
            else:
                if not owners or len(packets) < len(owners):
                    state = RangeState.UNAVAILABLE
                elif any(now - p.timestamp >= config.RADAR_STALE_S for p in packets):
                    state = RangeState.STALE
                else:
                    state = RangeState.NO_RETURN
                latest = max(packets, key=lambda p: p.timestamp) if packets else None
                result[sector] = RangeObservation(
                    sector,
                    None,
                    state,
                    latest.timestamp if latest else now,
                    latest.frame if latest else -1,
                    age_s=max(0.0, now - latest.timestamp) if latest else 0.0,
                )
        return result


class RadarRig:
    """Spawn exactly four radars; register before listening for exception-safe cleanup."""

    def __init__(self, manager, vehicle, geometry: VehicleGeometry):
        import carla

        self._streams = {}
        self._latest = {}
        self._tracker = RadarTracker(geometry, config.CAMERA_KEYS)
        xmin, xmax, ymin, ymax = geometry.bounds
        cx, cy = geometry.body_center_x_m, geometry.body_center_y_m
        z = geometry.ground_z_m + config.RADAR_MOUNT_HEIGHT_M
        self.mounts = {
            "front": ((xmax + 0.05, cy, z), 0.0),
            "rear": ((xmin - config.RADAR_REAR_CLEARANCE_M, cy, z), 180.0),
            "left": ((cx, ymin - 0.05, z), -90.0),
            "right": ((cx, ymax + 0.05, z), 90.0),
        }
        blueprint = manager.world.get_blueprint_library().find("sensor.other.radar")
        for attribute, value in {
            "horizontal_fov": config.RADAR_HORIZONTAL_FOV_DEG,
            "vertical_fov": config.RADAR_VERTICAL_FOV_DEG,
            "points_per_second": config.RADAR_POINTS_PER_SECOND,
            "range": config.RADAR_RANGE_M,
            "sensor_tick": 0.0,
        }.items():
            blueprint.set_attribute(attribute, str(value))
        for key, (location, yaw) in self.mounts.items():
            actor = manager.world.spawn_actor(
                blueprint,
                carla.Transform(carla.Location(*location), carla.Rotation(yaw=yaw)),
                attach_to=vehicle,
            )
            manager.track_actor(actor)
            inbox = queue.Queue(maxsize=2)
            self._streams[key] = (actor, inbox)
            actor.listen(
                lambda measurement, target=inbox: self._enqueue(target, measurement)
            )

    @staticmethod
    def _enqueue(inbox, measurement) -> None:
        try:
            inbox.put_nowait(measurement)
        except queue.Full:
            try:
                inbox.get_nowait()
            except queue.Empty:
                pass
            try:
                inbox.put_nowait(measurement)
            except queue.Full:
                pass

    def read(self, now: float, vehicle_transform) -> list[RangeObservation]:
        inverse = inverse_transform_matrix(vehicle_transform)
        for key, (_, inbox) in self._streams.items():
            while True:
                try:
                    packet = inbox.get_nowait()
                except queue.Empty:
                    break
                if key not in self._latest or packet.frame >= self._latest[key].frame:
                    self._latest[key] = packet
            packet = self._latest.get(key)
            if packet is None:
                continue
            converted = radar_points_to_vehicle(
                packet, transform_matrix(packet.transform), inverse
            )
            self._tracker.update(key, converted, packet.frame, packet.timestamp)
        return list(self._tracker.observations(now).values())

    def coverage_report(self) -> dict:
        """Analytic conic horizontal FOV only; this does not guarantee detections."""
        g = self._tracker.geometry
        x, y = np.meshgrid(np.arange(-9.0, 9.01, 0.1), np.arange(-7.0, 7.01, 0.1))
        xmin, xmax, ymin, ymax = g.bounds
        d = np.hypot(
            np.maximum.reduce([xmin - x, np.zeros_like(x), x - xmax]),
            np.maximum.reduce([ymin - y, np.zeros_like(y), y - ymax]),
        )
        ring = (d >= 0.15) & (d <= 4.0)
        covered = np.zeros_like(ring)
        sides = {
            "front": x >= xmax,
            "rear": x <= xmin,
            "left": y <= ymin,
            "right": y >= ymax,
        }
        for key, (location, yaw) in self.mounts.items():
            dx, dy = x - location[0], y - location[1]
            angle = (np.degrees(np.arctan2(dy, dx)) - yaw + 180) % 360 - 180
            covered |= (np.abs(angle) <= config.RADAR_HORIZONTAL_FOV_DEG / 2) & sides[
                key
            ]
        return {
            "model": "horizontal_conic_FOV_without_occlusion_or_sampling",
            "ring_m": [0.15, 4.0],
            "covered_fraction": float(covered[ring].mean()),
            "note": "four radars have corner/near-side blind zones; no return is unknown",
        }

    def stop(self) -> None:
        for actor, _ in self._streams.values():
            try:
                if actor.is_alive:
                    actor.stop()
            except RuntimeError as exc:
                print(f"[RadarRig] stop: {exc}")
