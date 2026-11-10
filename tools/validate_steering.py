"""Validate HGV swept-body prediction at selected signed steering controls, in both gears."""

from __future__ import annotations
import argparse, json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import carla
from core.carla_manager import CarlaManager
from tools.validate_demo import AcceptanceRunner


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--controls",
        type=float,
        nargs="+",
        default=[-0.95, -0.85, -0.6, -0.4, 0.4, 0.6, 0.85, 0.95],
    )
    parser.add_argument(
        "--forward-offset",
        type=float,
        default=8.0,
        help="Move to open intersection for steering tests",
    )
    parser.add_argument(
        "--output", type=Path, default=Path("artifacts/steering-validation.json")
    )
    args = parser.parse_args()
    if any(abs(x) > 1 for x in args.controls):
        parser.error("controls must lie in [-1,1]")
    rows = []
    with CarlaManager() as manager:
        runner = None
        try:
            runner = AcceptanceRunner(manager, args.output)
            runner.cameras.wait_first_frames()
            pose = runner.base
            runner.base = carla.Transform(
                pose.transform(carla.Location(x=args.forward_offset)), pose.rotation
            )
            for reverse in (False, True):
                for steer in args.controls:
                    row = runner.motion_case(steer, reverse)
                    rows.append(row)
                    print(json.dumps(row), flush=True)
        finally:
            if runner:
                runner.radar.stop()
                runner.cameras.stop()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(rows, indent=2))
    return 0 if all(row["criterion_pass"] for row in rows) else 2


if __name__ == "__main__":
    sys.exit(main())
