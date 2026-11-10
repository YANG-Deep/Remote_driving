"""Fit measured curvature/slip anchors from a dedicated steering training JSON; never use validation data."""

from __future__ import annotations
import argparse, json, math
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("training", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--server-build", required=True)
    args = parser.parse_args()
    groups = {}
    for row in json.loads(args.training.read_text()):
        if row["travel_m"] <= 0.5:
            continue
        groups.setdefault(abs(row["steer_control"]), []).append(row)
    samples, slips = [(0.0, 0.0)], [(0.0, 0.0)]
    for control, rows in sorted(groups.items()):
        if control == 0:
            continue
        angle = sum(abs(r["mean_wheel_angle_deg"]) for r in rows) / len(rows)
        samples.append(
            (
                angle,
                sum(abs(r["actual_heading_rad"]) / r["travel_m"] for r in rows)
                / len(rows),
            )
        )
        slips.append(
            (
                angle,
                sum(
                    r["rear_slip_angle_rad"]
                    * math.copysign(1, r["mean_wheel_angle_deg"])
                    for r in rows
                )
                / len(rows),
            )
        )
    profile = {
        "vehicle": "vehicle.carlamotors.european_hgv",
        "server_build": args.server_build,
        "samples": samples,
        "rear_slip_samples": slips,
        "training_logs": [str(args.training)],
        "conditions": "Low-speed locally flat calibration; separately validate held-out angles and speeds.",
        "limits": "Not a full physical tire/load model. Short training arcs do not count as acceptance passes.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(profile, indent=2))


if __name__ == "__main__":
    main()
