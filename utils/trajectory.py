"""Low-speed bicycle prediction in CARLA actor coordinates (+Y right)."""

from __future__ import annotations
import math
from core.models import VehicleGeometry


def predict_footprints(
    geometry: VehicleGeometry,
    steer_deg: float,
    reverse: bool,
    length_m: float,
    step_m: float,
) -> list[list[tuple[float, float]]]:
    """Move a rigid body about its equivalent rear axle; positive steer turns right."""
    if step_m <= 0 or length_m < 0 or geometry.wheelbase_m <= 0:
        raise ValueError("step/wheelbase must be positive and length nonnegative")
    if not all(math.isfinite(v) for v in (steer_deg, length_m, step_m)):
        raise ValueError("trajectory parameters must be finite")
    steer = max(-geometry.max_steer_deg, min(geometry.max_steer_deg, steer_deg))
    curvature = math.tan(math.radians(steer)) / geometry.wheelbase_m
    if geometry.curvature_samples and abs(steer) > 1e-6:
        factors = [
            (
                angle,
                (
                    geometry.wheelbase_m * curve / math.tan(math.radians(angle))
                    if angle
                    else 1.0
                ),
            )
            for angle, curve in geometry.curvature_samples
        ]
        factor = factors[-1][1]
        for (a0, f0), (a1, f1) in zip(factors, factors[1:]):
            if abs(steer) <= a1:
                factor = f0 + (f1 - f0) * (abs(steer) - a0) / (a1 - a0)
                break
        curvature *= factor
    slip = 0.0
    if geometry.rear_slip_samples:
        slip = geometry.rear_slip_samples[-1][1]
        for (a0, b0), (a1, b1) in zip(
            geometry.rear_slip_samples, geometry.rear_slip_samples[1:]
        ):
            if abs(steer) <= a1:
                slip = b0 + (b1 - b0) * (abs(steer) - a0) / (a1 - a0)
                break
        slip *= 1 if steer >= 0 else -1
    xmin, xmax, ymin, ymax = geometry.bounds
    corners = [(xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin)]
    count = max(1, math.ceil(length_m / step_m))
    result = []
    for index in range(count + 1):
        arc = min(index * step_m, length_m) * (-1 if reverse else 1)
        heading = arc * curvature
        if abs(curvature) < 1e-9:
            rx, ry = geometry.rear_axle_x_m + arc, 0.0
        else:
            rx = (
                geometry.rear_axle_x_m
                + (math.sin(heading + slip) - math.sin(slip)) / curvature
            )
            ry = (math.cos(slip) - math.cos(heading + slip)) / curvature
        cosine, sine = math.cos(heading), math.sin(heading)
        result.append(
            [
                (
                    rx + cosine * (x - geometry.rear_axle_x_m) - sine * y,
                    ry + sine * (x - geometry.rear_axle_x_m) + cosine * y,
                )
                for x, y in corners
            ]
        )
    return result
