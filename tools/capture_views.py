"""Capture real CARLA HUD states with stationary steering and direction changes."""

from pathlib import Path
import os, sys, json

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import carla, pygame
import config
from core.carla_manager import CarlaManager
from tools.validate_demo import AcceptanceRunner
from main import build_bev_base, overlay_bev, save_debug_frames, compose_status
from visualization.hud import Hud
from visualization.trajectory_overlay import build_trajectory, build_swept_footprints
from perception.bev_stitcher import BevStitcher


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=Path("artifacts/live-session/views")
    )
    output = parser.parse_args().output
    output.mkdir(parents=True, exist_ok=True)
    runner = hud = None
    with CarlaManager() as manager:
        try:
            runner = AcceptanceRunner(manager, output / "capture.json")
            runner.cameras.wait_first_frames()
            hud = Hud()
            stitcher = BevStitcher(
                width_px=config.BEV_RENDER_WIDTH_PX,
                height_px=round(
                    config.BEV_RENDER_WIDTH_PX
                    * hud._bev_rect.height
                    / hud._bev_rect.width
                ),
                geometry=runner.g,
                mounts=runner.mounts,
            )
            static = build_bev_base(stitcher)
            rows = []
            for name, steer, reverse in [
                ("forward", 0.0, False),
                ("turn_right", 0.25, False),
                ("turn_left", -0.25, False),
                ("reverse", 0.25, True),
            ]:
                for _ in range(30):
                    runner.vehicle.vehicle.apply_control(
                        carla.VehicleControl(brake=1.0, steer=steer, reverse=reverse)
                    )
                    hits, telemetry = runner.tick()
                bundle = runner.cameras.last_bundle
                angle = runner.vehicle.get_steer_angle_deg()
                paths = build_trajectory(runner.g, angle, reverse)
                bev = overlay_bev(
                    stitcher.stitch(bundle.images),
                    static,
                    paths,
                    stitcher,
                    build_swept_footprints(runner.g, angle, reverse),
                )
                status = compose_status(
                    runner.vehicle.get_speed_kmh(), reverse, angle, 0.0
                )
                status.pop("帧率")
                status["数据"] = "CARLA实测静态视图"
                hud.render(bev, bundle.images, paths, hits, status, stitcher, reverse)
                save_debug_frames(bundle.images, bev, output / name, hud)
                rows.append({"view": name, "steer_deg": angle, **telemetry})
            (output / "capture.json").write_text(json.dumps(rows, indent=2))
        finally:
            if hud:
                hud.close()
            if runner:
                runner.radar.stop()
                runner.cameras.stop()


if __name__ == "__main__":
    main()
