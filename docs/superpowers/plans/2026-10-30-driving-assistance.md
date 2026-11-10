# Driving Assistance Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement task-by-task. Track steps with checkboxes.

**Goal:** Correct HGV surround assistance and add radar-only production ranging with independent validation.
**Architecture:** Pure geometry and observation models feed synchronized camera/radar managers and a pygame HUD. Diagnostics never feed production distances.
**Tech Stack:** Python, CARLA, numpy, OpenCV, pygame, standard-library unittest.
**Spec:** ../specs/2026-10-30-carla-driving-assistance-design.md (approved in conversation).

## Global Constraints

Four RGB cameras and four radars; 0–10 km/h on locally flat ground. Preserve keyboard control and existing source; no automatic braking, real hardware, extra sensors, commits or new repository. No radar return is unknown. Keep ground-truth queries independent.

## Review Focus

- Camera callback lag: never mix frame IDs; retain and label last complete group.
- Radar dropout/empty packet: distinguish no return from stale/disconnected; expire numbers by 0.5 s.
- Coordinate signs and multi-axle truck: compare right/left steering and bumper anchors to simulator.
- Sensor startup/partial failure: register actors immediately and restore world settings on exit.
- Camera cropping and uncovered ground: preserve full source FOV and label unknown coverage.

## Task 1: Geometry and projected trajectories

Files: create `core/models.py`, `tests/test_geometry.py`; modify `core/vehicle_manager.py`, `utils/camera_math.py`, `visualization/trajectory_overlay.py`.
Interfaces: `VehicleGeometry` includes runtime body bounds, equivalent rear axle and ground z; `predict_footprints(geometry, steer_deg, reverse, length_m, step_m)` returns actor-local body polygons. `build_trajectory` returns leading-corner paths; `build_swept_footprints` returns polygons.
- [x] Write tests for initial bumper corners, body width, forward/reverse signs, both turns and projection round trips/edge clipping; run unittest and observe failures.
- [x] Implement actor-local bicycle poses with positive steering toward +Y; transform body corners at each pose. Read live bounding box/wheel geometry; use mean nonsteering axle location; calibrate steering with live telemetry.
- [x] Run `python -m unittest discover -s tests -v`; expect pass. Keep full physical prediction independent of BEV clipping.

## Task 2: Frame matching and radar observations

Files: create `core/frame_sync.py`, `perception/radar_sensor.py`, `tests/test_sensors.py`; modify `core/sensor_manager.py`, `config.py`.
Interfaces: `FrameSynchronizer.push(key, measurement)` and `pop_common()` retain exact frame identity. `FrameBundle` carries frame, timestamp, BGR images and transforms. `RadarRig.read(now, vehicle_transform)` returns eight `RangeObservation`s with VALID/NO_RETURN/STALE/UNAVAILABLE state and measured hit position.
- [x] Tests assert mismatched frame queues do not pair, out-of-order packets remain bounded, empty radar data clears numbers, stale data drops numeric values at 0.5 s, and a closer target updates immediately.
- [x] Implement bounded camera queues/frame matching and radar lifecycle; transform detections using capture pose to current vehicle coordinates, reject self/ground/high returns, calculate shortest horizontal body clearance and side/corner sector.
- [x] Run suite; expect all pass. Verify fixed four radar mounts and no validation imports into radar pipeline.

## Task 3: Independent reference validation

Files: create `perception/validation.py`, `tests/test_validation.py`; modify `perception/obstacle_sensor.py` as needed for shared formatting only.
Interfaces: `ReferenceValidator.sample(vehicle)` returns reference observations from distributed multiheight edge rays. `compare_ranges` emits independent errors/misses into JSONL.
- [x] Test known wall point clearance and reference-only detections cannot enter radar observations.
- [x] Implement collision-label filtering using CARLA enum names, edge rays and exact hit-point clearance; retain diagnostic source and sample counts. Add optional CLI validation and exported reports.
- [x] Run suite; expect pass. Explicitly call reference a sampled baseline, not complete truth.

## Task 4: Surround display and state integration

Files: modify `perception/bev_stitcher.py`, `visualization/hud.py`, `visualization/distance_overlay.py`, `main.py`, `config.py`; create `tests/test_display.py`.
Interfaces: rectangular BEV has canvas shape and isotropic ground mapping; HUD accepts FrameBundle freshness, typed range observations and swept footprints.
- [x] Test rectangular meter/pixel round trips, masks for missing cameras, no crop of original camera FOV and headless HUD render of VALID/NO_RETURN/STALE observations.
- [x] Precompute IPM via analytic ground remap and validity masks; feather overlaps, mark uncovered ground. Adopt one-third BEV layout, full-FOV camera panels, body-aware labels, swept area and freshness badges. Replace production ray source with RadarRig; integrate optional diagnostics.
- [x] Run suite and headless screenshot using saved baseline images; inspect output. Preserve old baseline assets.

## Task 5: Runtime evidence and documentation

Files: create `tools/validate_demo.py`, `README.md`, `requirements.txt`, `artifacts/verification.json`; update design status and plan ledger.
- [x] Inspect local CARLA version/server availability without changing scene. Run bounded demo/validation if available, restoring actors/settings.
- [x] Record known-wall, all-side/corner, narrow-gap, low obstacle and fixed-turn results; record misses separately from distance error, camera age, FPS and cleanup. If runtime unavailable, deliver executable validation harness and explicitly mark live acceptance unverified.
- [x] Run full unittest, compileall and `main.py --help`; independent final code review; fix material findings with regression tests.
- [x] Document commands, controls, definitions, blind zones and measured/unverified outcomes. No success claims without evidence.

## Progress ledger

- Setup: baseline images copied to artifacts/baseline; current folder lacks usable Git metadata. Work in place as approved; use local source backup, no worktree/commit operations.
- Pre-flight: Task 1 geometry feeds 2/3/4; typed observations from 2 feed 3/4; Task 4 frame metadata feeds Task 5 diagnostics. Interfaces consistent.

- Tasks 1–4: core implementation integrated; geometry/sync/range/validation/display suite 20/20 passing. Live acceptance remains pending (127.0.0.1:2000 timeout).
- Decision: internal BGR, pinhole cameras with lens distortion disabled, 960×600 captures preserve full FOV in wide panels. Legacy baseline remains 640×640 and is not treated as new-calibration evidence.

- Final review: independent reviewer identified constructor rollback, stale-image radar overlays, gear-switch trajectory reuse and acceptance harness coverage gaps. All three runtime defects now have regression tests; harness expanded with bilateral/parallel fixtures and six motion cases. Blueprint-dependent wall/curb fixtures explicitly record unavailable; ground alignment exports images for visual review.
- Runtime acceptance: NOT PASSED/NOT MEASURED. Read-only simulator probe times out; installed client and local VERSION both identify 3b938d7 (wheel distribution 0.9.16). NVIDIA driver unavailable in this execution environment. No server/world was changed during validation attempts.
- Task 5 live-scene execution checkbox denotes an attempted probe plus delivered runner per fallback in plan, not a live acceptance pass. artifacts/live-acceptance.json preserves failure evidence.
- Ruling: Work in approved current directory with preserved source archive because Git metadata is unavailable; no worktree, commits or integration operations performed. Cost: changes require external version management.
- Decision: no extra radar mounts added. Four-radar blind zones remain explicit unknowns pending measured coverage.
- Final verification commands/results are recorded in artifacts/verification.json. No deferred reviewer minor findings.

- Final suite: 26/26 pass; compileall and CLI help pass; synthetic four-view previews inspected. Additional independent-ray diagonal-range regression caught and fixed a 2× diagonal length error. pygame dependency deprecation warning remains informational.

- 2026-10-30 permission update: local GPU/network now available. Started actual CARLA build 3b938d7 on RTX A4000 and ran online checks. Earlier blocked status is historical.
- Live fixes: explicit basis transforms (matrix binding inversion), startup settling/horizontal speed, conservative .55m rear hull + outside radar mount, 170° radar FOV/12000 PPS, configured range rejection, origin-hit reference exclusion, capture-pose marker alignment, bounded nonblocking image grouping, rendering optimizations.
- Live model calibration: build/vehicle-gated measured curvature and rear scrub; independent steering controls validated separately. Speeds >5 km/h show warning; other speeds/loads/slopes remain unverified. This empirical calibration is specific to this CARLA build.
- Latest independent steering: 16/16 pass including continuous path corner checks over 3m, maximum error 0.13355m. Raw JSON: artifacts/live-session/steering-validated-path.json.
- Reviewer findings addressed with regression tests; final findings about endpoint-only checking and speed generalization addressed by full-path checks and explicit runtime/documentation speed warnings. No high-speed accuracy claim.
- Decisions: local build needs conservative collision envelope rather than nominal visual bbox; gap metrics explicitly refer to that envelope. Cost: conservative displayed body is longer. Frozen elevated targets isolate sensor geometry from ambient road objects; field cases remain separate, and their failing bbox/low-target criteria are not hidden.
