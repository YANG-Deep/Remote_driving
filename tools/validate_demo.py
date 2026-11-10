"""Bounded CARLA acceptance runner. Stores measurements, never converts failures to passes."""

from __future__ import annotations
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import traceback
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utils.transforms import transform_matrix, inverse_transform_matrix
import carla
import config
from core.carla_manager import CarlaManager
from core.vehicle_manager import VehicleManager
from core.sensor_manager import SensorManager
from core.models import RangeState, SECTOR_BEARINGS
from perception.bev_stitcher import BevStitcher, camera_mounts
from perception.radar_sensor import RadarRig
from perception.validation import ReferenceValidator, compare_ranges
from utils.trajectory import predict_footprints


def fixture_center(g, box, sector: str, gap: float):
    """Aligned rectangles: requested gap between nearest projected surfaces."""
    xmin, xmax, ymin, ymax = g.bounds
    diagonal = "_" in sector
    offset = gap / (2**0.5) if diagonal else gap
    x = (
        xmax + box.extent.x + offset
        if "front" in sector
        else xmin - box.extent.x - offset if "rear" in sector else g.body_center_x_m
    )
    y = (
        ymax + box.extent.y + offset
        if "right" in sector
        else ymin - box.extent.y - offset if "left" in sector else g.body_center_y_m
    )
    return x - box.location.x, y - box.location.y


class AcceptanceRunner:
    def __init__(self, manager, output: Path):
        self.manager, self.output = manager, output
        self.vehicle = VehicleManager(manager)
        self.g = self.vehicle.geometry
        self.mounts = camera_mounts(self.g)
        self.cameras = SensorManager(manager, self.vehicle.vehicle, self.mounts)
        self.radar = RadarRig(manager, self.vehicle.vehicle, self.g)
        self.validator = ReferenceValidator(manager.world, self.g)
        self.base = self.vehicle.get_transform()
        self.rows = []

    def tick(self):
        started = time.monotonic()
        frame = self.manager.tick()
        bundle = self.cameras.get_bundle()
        snapshot = self.manager.world.get_snapshot()
        now = snapshot.timestamp.elapsed_seconds
        observations = self.radar.read(now, self.vehicle.get_transform())
        return observations, {
            "frame": frame,
            "timestamp": now,
            "camera_frame": bundle.frame if bundle else None,
            "processing_s": time.monotonic() - started,
        }

    def stationary_case(self, blueprint, sector, gap):
        ego = self.vehicle.vehicle
        ego.set_simulate_physics(False)
        actor = None
        try:
            # Spawn away first to inspect actual dimensions; failure is reported explicitly.
            far = self.base.transform(carla.Location(x=30, z=1))
            actor = self.manager.world.try_spawn_actor(
                blueprint, carla.Transform(far, self.base.rotation)
            )
            if actor is None:
                return {
                    "case": sector,
                    "gap_m": gap,
                    "status": "fixture_spawn_failed",
                    "blueprint": blueprint.id,
                }
            self.manager.track_actor(actor)
            actor.set_simulate_physics(False)
            x, y = fixture_center(self.g, actor.bounding_box, sector, gap)
            z = (
                self.g.ground_z_m
                - actor.bounding_box.location.z
                + actor.bounding_box.extent.z
                + 0.02
            )
            location = self.base.transform(carla.Location(x=x, y=y, z=z))
            actor.set_transform(carla.Transform(location, self.base.rotation))
            measurements = []
            samples = []
            for index in range(16):
                observations, telemetry = self.tick()
                if index < 5:
                    continue
                measurement = next(o for o in observations if o.sector == sector)
                samples.append(asdict(measurement))
                if (
                    measurement.state == RangeState.VALID
                    and measurement.distance_m is not None
                ):
                    measurements.append(measurement.distance_m)
            errors = [v - gap for v in measurements]
            return {
                "case": sector,
                "gap_m": gap,
                "blueprint": blueprint.id,
                "status": "measured" if measurements else "miss",
                "detections": len(measurements),
                "samples": samples,
                "worst_abs_error_m": max(map(abs, errors)) if errors else None,
                "criterion_pass": bool(errors)
                and len(errors) == len(samples)
                and max(map(abs, errors)) <= 0.15,
                "reference": compare_ranges(observations, self.validator.sample(ego)),
                "telemetry": telemetry,
            }
        finally:
            if actor and actor.is_alive:
                actor.destroy()

    def corridor_case(self, blueprint, gap: float = 1.0, parallel: bool = False):
        """Two-sided gap; optional kinematic translation tests stable parallel clearance."""
        ego = self.vehicle.vehicle
        ego.set_simulate_physics(False)
        ego.set_transform(self.base)
        actors = []
        try:
            for index, sector in enumerate(("left", "right")):
                location = self.base.transform(carla.Location(x=30 + index * 15, z=1))
                actor = self.manager.world.try_spawn_actor(
                    blueprint, carla.Transform(location, self.base.rotation)
                )
                if actor is None:
                    return {
                        "case": "parallel_approach" if parallel else "narrow_corridor",
                        "status": "fixture_spawn_failed",
                        "criterion_pass": False,
                    }
                self.manager.track_actor(actor)
                actors.append(actor)
                actor.set_simulate_physics(False)
                x, y = fixture_center(self.g, actor.bounding_box, sector, gap)
                z = (
                    self.g.ground_z_m
                    - actor.bounding_box.location.z
                    + actor.bounding_box.extent.z
                    + 0.02
                )
                actor.set_transform(
                    carla.Transform(
                        self.base.transform(carla.Location(x=x, y=y, z=z)),
                        self.base.rotation,
                    )
                )
            errors = []
            misses = 0
            for index in range(26):
                if parallel:
                    location = self.base.transform(
                        carla.Location(x=max(0, index - 5) * 0.1)
                    )
                    ego.set_transform(carla.Transform(location, self.base.rotation))
                observations, telemetry = self.tick()
                if index < 5:
                    continue
                for sector in ("left", "right"):
                    obs = next(o for o in observations if o.sector == sector)
                    if obs.state == RangeState.VALID and obs.distance_m is not None:
                        errors.append(abs(obs.distance_m - gap))
                    else:
                        misses += 1
            return {
                "case": "parallel_approach" if parallel else "narrow_corridor",
                "status": "measured",
                "gap_m": gap,
                "misses": misses,
                "worst_abs_error_m": max(errors) if errors else None,
                "criterion_pass": bool(errors) and misses == 0 and max(errors) <= 0.15,
                "note": "parallel case translates frozen ego 2m; tests ranging, not vehicle dynamics",
            }
        finally:
            for actor in actors:
                if actor.is_alive:
                    actor.destroy()
            ego.set_transform(self.base)

    def motion_case(self, steer: float, reverse: bool):
        ego = self.vehicle.vehicle
        ego.set_simulate_physics(True)
        ego.set_transform(self.base)
        ego.set_target_velocity(carla.Vector3D())
        ego.set_target_angular_velocity(carla.Vector3D())
        for _ in range(20):
            ego.apply_control(
                carla.VehicleControl(steer=steer, brake=1.0, reverse=reverse)
            )
            self.tick()
        origin = ego.get_transform()
        inverse = inverse_transform_matrix(origin)
        last = origin.transform(carla.Location(x=self.g.rear_axle_x_m))
        travelled = 0.0
        path_errors = []
        angles = []
        right_angles = []
        start = time.monotonic()
        for _ in range(300):
            speed = abs(self.vehicle.get_speed_kmh())
            ego.apply_control(
                carla.VehicleControl(
                    throttle=(
                        (0.4 if reverse and abs(steer) >= 0.9 and speed < 2 else 0.25)
                        if speed < 5
                        else 0.0
                    ),
                    brake=0.15 if speed > 5 else 0.0,
                    steer=steer,
                    reverse=reverse,
                )
            )
            self.tick()
            angles.append(self.vehicle.get_steer_angle_deg())
            right_angles.append(
                ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
            )
            pose = ego.get_transform()
            current = pose.transform(carla.Location(x=self.g.rear_axle_x_m))
            travelled += current.distance(last)
            last = current
            prefix = predict_footprints(
                self.g, float(np.mean(angles)), reverse, travelled, max(travelled, 0.1)
            )[-1]
            prefix_matrix = inverse @ transform_matrix(pose)
            xmin, xmax, ymin, ymax = self.g.bounds
            for (x, y), pred in zip(
                [(xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin)], prefix
            ):
                actual = (prefix_matrix @ [x, y, 0.0, 1.0])[:2]
                path_errors.append(float(np.linalg.norm(actual - pred)))
            if travelled >= 3.0:
                break
        angle = float(np.mean(angles))
        prediction = predict_footprints(self.g, angle, reverse, travelled, 0.1)[-1]
        xmin, xmax, ymin, ymax = self.g.bounds
        error = []
        matrix = inverse @ transform_matrix(ego.get_transform())
        for (x, y), predicted in zip(
            [(xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin)], prediction
        ):
            actual = (matrix @ np.array([x, y, 0.0, 1.0]))[:2]
            error.append(float(np.linalg.norm(actual - predicted)))
        actual_heading = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
        rear_delta = (matrix @ [self.g.rear_axle_x_m, 0.0, 0.0, 1.0])[:2] - [
            self.g.rear_axle_x_m,
            0.0,
        ]
        direction = -1.0 if reverse else 1.0
        slip = float(
            np.arctan2(direction * rear_delta[1], direction * rear_delta[0])
            - actual_heading / 2
        )
        ego.apply_control(carla.VehicleControl(brake=1.0))
        return {
            "case": "motion",
            "steer_control": steer,
            "reverse": reverse,
            "travel_m": travelled,
            "mean_wheel_angle_deg": angle,
            "mean_right_wheel_angle_deg": float(np.mean(right_angles)),
            "actual_heading_rad": actual_heading,
            "rear_slip_angle_rad": slip,
            "actual_actor_translation_xy": matrix[:2, 3].tolist(),
            "worst_corner_error_m": max(error),
            "worst_path_corner_error_m": max(path_errors),
            "elapsed_s": time.monotonic() - start,
            "criterion_pass": travelled >= 3.0 and max(path_errors) <= 0.3,
            "note": "mean-angle low-speed prediction; collision/background objects may invalidate this fixture",
        }

    def run(self):
        self.cameras.wait_first_frames()
        library = self.manager.world.get_blueprint_library()
        car = library.find("vehicle.tesla.model3")
        for sector in SECTOR_BEARINGS:
            for gap in (0.5, 1.0, 2.0):
                row = self.stationary_case(car, sector, gap)
                self.rows.append(row)
                print(sector, gap, row["status"], flush=True)
        for pattern, sector in [
            ("walker.pedestrian.*", "front"),
            ("static.prop.trafficcone*", "front"),
            ("static.prop.streetbarrier*", "left"),
            ("static.prop.*wall*", "left"),
            ("static.prop.*curb*", "front"),
        ]:
            matches = library.filter(pattern)
            self.rows.append(
                self.stationary_case(matches[0], sector, 1.0)
                if matches
                else {"case": pattern, "status": "blueprint_unavailable"}
            )
        self.rows.append(self.corridor_case(car))
        self.rows.append(self.corridor_case(car, parallel=True))
        for steer, reverse in [
            (0.0, False),
            (0.25, False),
            (-0.25, False),
            (0.0, True),
            (0.25, True),
            (-0.25, True),
        ]:
            self.rows.append(self.motion_case(steer, reverse))
        bundle = self.cameras.last_bundle
        if bundle:
            from main import build_bev_base, overlay_bev, save_debug_frames
            from visualization.trajectory_overlay import build_trajectory

            stitcher = BevStitcher(
                width_px=620, height_px=940, geometry=self.g, mounts=self.mounts
            )
            bev = overlay_bev(
                stitcher.stitch(bundle.images),
                build_bev_base(stitcher),
                build_trajectory(self.g, 0.0, False),
                stitcher,
            )
            save_debug_frames(bundle.images, bev, self.output.parent / "live-images")
            self.rows.append(
                {
                    "case": "ground_marker_alignment",
                    "status": "requires_visual_review",
                    "images": str(self.output.parent / "live-images"),
                    "note": "view ground markings and seams; screenshot export is not a pass",
                }
            )
        return self.rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map", default=config.MAP_NAME)
    parser.add_argument(
        "--output", type=Path, default=Path("artifacts/live-acceptance.json")
    )
    args = parser.parse_args()
    report = {"status": "unverified", "cases": []}
    manager = runner = None
    code = 1
    try:
        # Read-only probe before world changes.
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        report["client_version"] = client.get_client_version()
        report["server_version"] = client.get_server_version()
        manager = CarlaManager(args.host, args.port, args.map)
        before = {a.id for a in manager.world.get_actors()}
        runner = AcceptanceRunner(manager, args.output)
        report["geometry"] = asdict(runner.g)
        report["cases"] = runner.run()
        report["status"] = "measured"
        code = (
            0
            if all(case.get("criterion_pass", False) for case in report["cases"])
            else 2
        )
    except Exception as exc:
        report["error"] = str(exc)
        if runner:
            report["cases"] = runner.rows
        traceback.print_exc()
    finally:
        if runner:
            runner.radar.stop()
            runner.cameras.stop()
        if manager:
            manager.destroy()
            manager.world.wait_for_tick(
                3.0
            )  # refresh client actor snapshot after deferred destruction
            remaining = {a.id for a in manager.world.get_actors()}
            report["remaining_new_actor_ids"] = sorted(remaining - before)
            if report["remaining_new_actor_ids"]:
                code = 2
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return code


if __name__ == "__main__":
    sys.exit(main())
