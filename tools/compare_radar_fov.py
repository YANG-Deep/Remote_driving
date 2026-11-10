"""Online conic-FOV experiment with isolated suspended targets (no background returns)."""

from pathlib import Path
import sys, argparse, json

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import carla
import config
from core.carla_manager import CarlaManager
from tools.validate_demo import AcceptanceRunner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fov", type=float, required=True)
    parser.add_argument("--points", type=int, default=12000)
    parser.add_argument("--target", default="static.prop.container")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config.RADAR_HORIZONTAL_FOV_DEG = args.fov
    config.RADAR_POINTS_PER_SECOND = args.points
    report = {
        "fov": args.fov,
        "points_per_second": args.points,
        "fixture": "frozen actors 30m above road, tests sensor geometry only",
        "cases": [],
    }
    runner = None
    with CarlaManager() as manager:
        before = {a.id for a in manager.world.get_actors()}
        try:
            runner = AcceptanceRunner(manager, args.output)
            ego = runner.vehicle.vehicle
            ego.set_simulate_physics(False)
            pose = ego.get_transform()
            pose.location.z += 30
            ego.set_transform(pose)
            manager.tick()
            runner.base = ego.get_transform()
            bp = manager.world.get_blueprint_library().find(args.target)
            for sector in (
                "front",
                "front_right",
                "right",
                "rear_right",
                "rear",
                "rear_left",
                "left",
                "front_left",
            ):
                for gap in (0.5, 1.0, 2.0):
                    row = runner.stationary_case(bp, sector, gap)
                    report["cases"].append(row)
                    print(
                        sector, gap, row["status"], row["worst_abs_error_m"], flush=True
                    )
            report["analytic_coverage"] = runner.radar.coverage_report()
        finally:
            if runner:
                runner.radar.stop()
                runner.cameras.stop()
    manager.world.wait_for_tick(3.0)
    report["remaining_new_actor_ids"] = sorted(
        {a.id for a in manager.world.get_actors()} - before
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
