"""Run CARLA checks sequentially; preserve failed criteria and process exit codes."""

from __future__ import annotations
import json, subprocess, sys, time
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    output = root / "artifacts/live-final"
    output.mkdir(parents=True, exist_ok=True)
    tasks = [
        (
            "planar",
            [
                sys.executable,
                "tools/compare_radar_fov.py",
                "--fov",
                "170",
                "--points",
                "12000",
                "--output",
                str(output / "planar.json"),
            ],
        ),
        (
            "field",
            [
                sys.executable,
                "tools/validate_demo.py",
                "--output",
                str(output / "field.json"),
            ],
        ),
        (
            "views",
            [
                sys.executable,
                "tools/capture_views.py",
                "--output",
                str(output / "views"),
            ],
        ),
        (
            "gui",
            [
                sys.executable,
                "main.py",
                "--max-frames",
                "200",
                "--save-frame",
                "--output",
                str(output / "gui"),
            ],
        ),
    ]
    records = []
    for name, command in tasks:
        started = time.monotonic()
        with (output / f"{name}.log").open("w") as log:
            result = subprocess.run(
                command, cwd=root, stdout=log, stderr=subprocess.STDOUT
            )
        row = {
            "task": name,
            "command": command,
            "exit_code": result.returncode,
            "wall_s": time.monotonic() - started,
        }
        records.append(row)
        print(json.dumps(row), flush=True)
        (output / "execution.json").write_text(json.dumps(records, indent=2))
    # field includes bbox-based diagnostics and explicit visual-review/unavailable cases.
    # Preserve its nonzero result rather than labeling all scenarios as passed.
    return 0 if all(row["exit_code"] == 0 for row in records) else 2


if __name__ == "__main__":
    sys.exit(main())
