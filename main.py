"""CARLA low-speed HGV surround assistance; radar ranges, optional independent reference."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import traceback
import cv2
import numpy as np
import pygame
import config
from core.carla_manager import CarlaManager
from core.sensor_manager import SensorManager
from core.vehicle_manager import VehicleManager
from core.models import VehicleGeometry
from perception.bev_stitcher import BevStitcher, camera_mounts
from perception.radar_sensor import RadarRig, align_observation_points
from perception.validation import ReferenceValidator, compare_ranges
from visualization.hud import Hud
from visualization.trajectory_overlay import (
    build_frame_overlays,
    draw_trajectory_on_bev,
)
from visualization.vehicle_icon import draw_vehicle_icon


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="CARLA 低速远程驾驶辅助（四摄 + 四雷达）"
    )
    parser.add_argument("--host", default=config.CARLA_HOST)
    parser.add_argument("--port", type=int, default=config.CARLA_PORT)
    parser.add_argument("--map", default=config.MAP_NAME)
    parser.add_argument(
        "--save-frame", action="store_true", help="保存首组完整图像与 HUD"
    )
    parser.add_argument(
        "--validate", action="store_true", help="独立射线校验，结果只写诊断日志"
    )
    parser.add_argument(
        "--max-frames", type=int, default=0, help="有限帧运行，0 表示持续运行"
    )
    parser.add_argument("--output", type=Path, default=Path("_debug_out"))
    return parser.parse_args()


def build_bev_base(
    stitcher: BevStitcher, body_center_x_m: float | None = None
) -> np.ndarray:
    """Runtime body rectangle at correct actor origin; no distracting concentric rings."""
    g = stitcher.geometry
    overlay = np.zeros(stitcher.canvas_shape, np.uint8)
    xmin, xmax, ymin, ymax = g.bounds
    draw_vehicle_icon(
        overlay,
        center_px=stitcher.ground_to_canvas((xmin + xmax) / 2, (ymin + ymax) / 2),
        length_px=stitcher.meters_to_px(xmax - xmin),
        width_px=stitcher.meters_to_px(2 * g.half_width_m),
    )
    stitcher.static_overlay_mask = np.any(overlay > 0, axis=2)
    return overlay


def overlay_bev(stitched, static_layer, trajectory, stitcher, footprints=None):
    canvas = stitched.copy()
    if footprints:
        sweep = np.zeros(canvas.shape[:2], np.uint8)
        for polygon in footprints:
            pts = np.array(
                [stitcher.ground_to_canvas(x, y) for x, y in polygon], np.int32
            )
            cv2.fillPoly(sweep, [pts], 255)
        tinted = canvas.copy()
        tinted[sweep > 0] = (150, 110, 55)
        canvas = cv2.addWeighted(canvas, 0.82, tinted, 0.18, 0)
        contours, _ = cv2.findContours(
            sweep, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(canvas, contours, -1, (160, 140, 85), 1, cv2.LINE_AA)
    draw_trajectory_on_bev(canvas, trajectory, stitcher)
    mask = stitcher.static_overlay_mask
    canvas[mask] = static_layer[mask]
    return canvas


def compose_status(speed_kmh, reverse, steer_deg, fps, hits=None):
    return {
        "速度": f"{abs(speed_kmh):.1f} km/h",
        "挡位": "R" if reverse else "D",
        "转角": f"{steer_deg:+.1f}°",
        "帧率": f"{fps:.0f}",
    }


def save_debug_frames(frames, bev, output: Path | None = None, hud: Hud | None = None):
    output = output or Path(config.DEBUG_OUTPUT_DIR)
    output.mkdir(parents=True, exist_ok=True)
    for key, frame in frames.items():
        cv2.imwrite(str(output / f"cam_{key}.png"), frame)
    cv2.imwrite(str(output / "bev.png"), bev)
    if hud:
        pygame.image.save(hud.screen, str(output / "hud.png"))


def run(args: argparse.Namespace) -> int:
    hud = manager = cameras = radar = log = None
    try:
        manager = CarlaManager(host=args.host, port=args.port, map_name=args.map)
        print(
            f"[CARLA] client={manager.client.get_client_version()} server={manager.client.get_server_version()}"
        )
        vehicle = VehicleManager(manager)
        geometry = vehicle.geometry
        mounts = camera_mounts(geometry)
        cameras = SensorManager(manager, vehicle.vehicle, mounts)
        radar = RadarRig(manager, vehicle.vehicle, geometry)
        validator = (
            ReferenceValidator(manager.world, geometry) if args.validate else None
        )
        hud = Hud()
        stitcher = BevStitcher(
            width_px=config.BEV_RENDER_WIDTH_PX,
            height_px=round(
                config.BEV_RENDER_WIDTH_PX * hud._bev_rect.height / hud._bev_rect.width
            ),
            geometry=geometry,
            mounts=mounts,
        )
        static = build_bev_base(stitcher)
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "setup.json").write_text(
            json.dumps(
                {
                    "geometry": asdict(geometry),
                    "camera_mounts": mounts,
                    "radar_mounts": radar.mounts,
                    "client_version": manager.client.get_client_version(),
                    "server_version": manager.client.get_server_version(),
                    "range_definition": "horizontal point-to-body rectangle clearance",
                    "radar_coverage": radar.coverage_report(),
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        log = (args.output / "telemetry.jsonl").open("w", encoding="utf-8")
        frames = cameras.wait_first_frames()
        saved, count, running = False, 0, True
        frame_states = {}
        while running and (not args.max_frames or count < args.max_frames):
            for event in hud.pump_events():
                if event.type == pygame.QUIT or (
                    event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
                ):
                    running = False
                else:
                    vehicle.handle_key_event(event)
            if not running:
                break
            started = time.monotonic()
            vehicle.apply_keyboard(pygame.key.get_pressed())
            frame = manager.tick()
            snapshot = manager.world.get_snapshot()
            now = snapshot.timestamp.elapsed_seconds
            steering, reverse = vehicle.get_steer_angle_deg(), vehicle.is_reverse
            current_pose = vehicle.get_transform()
            frame_states[frame] = (steering, reverse, current_pose)
            frame_states = {
                key: value for key, value in frame_states.items() if key >= frame - 128
            }
            new_bundle = cameras.get_bundle(wait_s=0.0)
            bundle = new_bundle or cameras.last_bundle
            if new_bundle:
                frames = new_bundle.images
            age = max(0.0, now - bundle.timestamp)
            hits = radar.read(now, current_pose)
            trajectory, sweep = [], []
            captured_state = frame_states.get(bundle.frame)
            if new_bundle and captured_state:
                trajectory, sweep = build_frame_overlays(
                    geometry, captured_state[0], captured_state[1], reverse, age
                )
            overlay_hits = (
                align_observation_points(hits, current_pose, captured_state[2])
                if captured_state
                else []
            )
            bev = overlay_bev(
                stitcher.stitch(frames), static, trajectory, stitcher, sweep
            )
            status = compose_status(
                vehicle.get_speed_kmh(), reverse, steering, hud.current_fps()
            )
            status["相机"] = f"{age*1000:.0f}ms" + (
                " 过期" if age >= config.RADAR_STALE_S else ""
            )
            status["测距"] = "雷达"
            if abs(vehicle.get_speed_kmh()) > 5:
                status["提示"] = "轨迹超出已验证速度范围"
            if abs(vehicle.get_speed_kmh()) > 10:
                status["提示"] = "超出低速演示范围"
            hud.render(
                bev,
                frames,
                trajectory,
                hits,
                status,
                stitcher,
                reverse,
                image_age_s=age,
                overlay_hits=overlay_hits,
            )
            if args.save_frame and not saved:
                save_debug_frames(frames, bev, args.output, hud)
                saved = True
            row = {
                "frame": frame,
                "wall_time_s": started,
                "timestamp": now,
                "camera_frame": bundle.frame,
                "camera_age_s": age,
                "speed_kmh": vehicle.get_speed_kmh(),
                "steer_deg": steering,
                "ranges": [asdict(hit) for hit in hits],
                "processing_s": time.monotonic() - started,
            }
            if count % config.VALIDATION_INTERVAL_FRAMES == 0:
                row["camera_coverage"] = stitcher.coverage_report()
                if validator:
                    row["validation"] = compare_ranges(
                        hits, validator.sample(vehicle.vehicle)
                    )
            row["processing_s"] = time.monotonic() - started
            log.write(json.dumps(row) + "\n")
            if count % 20 == 0:
                log.flush()
            count += 1
            hud.tick()
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        if log:
            log.close()
        if radar:
            radar.stop()
        if cameras:
            cameras.stop()
        if hud:
            hud.close()
        if manager:
            manager.destroy()


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    sys.exit(main())
