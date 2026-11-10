"""Four BGR cameras with bounded exact-frame grouping and capture metadata."""

from __future__ import annotations
import queue
import time
import carla
import numpy as np
import cv2
import config
from core.frame_sync import FrameSynchronizer, FrameBundle


class SensorManager:
    def __init__(self, carla_manager, vehicle, mounts: dict | None = None):
        self._manager = carla_manager
        self._streams = {}
        self._sync = FrameSynchronizer(config.CAMERA_KEYS, config.SENSOR_FRAME_BUFFER)
        self.last_bundle: FrameBundle | None = None
        blueprint = carla_manager.world.get_blueprint_library().find(
            "sensor.camera.rgb"
        )
        for key, value in {
            "image_size_x": config.CAMERA_WIDTH_PX,
            "image_size_y": config.CAMERA_HEIGHT_PX,
            "fov": config.CAMERA_FOV_DEG,
            "sensor_tick": 0.0,
        }.items():
            blueprint.set_attribute(key, str(value))
        if blueprint.has_attribute("enable_postprocess_effects"):
            blueprint.set_attribute("enable_postprocess_effects", "True")
        for attr in ("lens_k", "lens_kcube"):
            if blueprint.has_attribute(attr):
                blueprint.set_attribute(attr, "0.0")
        for key, (location, rotation) in (mounts or config.CAMERA_MOUNTS).items():
            actor = carla_manager.world.spawn_actor(
                blueprint,
                carla.Transform(
                    carla.Location(*location),
                    carla.Rotation(
                        pitch=rotation[0], yaw=rotation[1], roll=rotation[2]
                    ),
                ),
                attach_to=vehicle,
            )
            carla_manager.track_actor(actor)
            inbox = queue.Queue(maxsize=config.SENSOR_FRAME_BUFFER)
            self._streams[key] = (actor, inbox)
            actor.listen(
                lambda measurement, target=inbox: self._enqueue(target, measurement)
            )

    @staticmethod
    def _enqueue(inbox, measurement):
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

    def get_bundle(self, wait_s: float = config.SENSOR_WAIT_S) -> FrameBundle | None:
        deadline = time.monotonic() + wait_s
        while True:
            for key, (_, inbox) in self._streams.items():
                while True:
                    try:
                        self._sync.push(key, inbox.get_nowait())
                    except queue.Empty:
                        break
            group = self._sync.pop_common()
            if group:
                first = next(iter(group.values()))
                images = {
                    key: cv2.cvtColor(
                        np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                            image.height, image.width, 4
                        ),
                        cv2.COLOR_BGRA2BGR,
                    )
                    for key, image in group.items()
                }
                self.last_bundle = FrameBundle(
                    first.frame,
                    first.timestamp,
                    images,
                    {key: image.transform for key, image in group.items()},
                )
                return self.last_bundle
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.001)

    def get_frames(self) -> dict[str, np.ndarray] | None:
        bundle = self.get_bundle()
        return bundle.images if bundle else None

    def wait_first_frames(self, max_ticks: int = 60) -> dict[str, np.ndarray]:
        for _ in range(max_ticks):
            self._manager.tick()
            bundle = self.get_bundle()
            if bundle:
                return bundle.images
        raise RuntimeError("四路相机未能组成同帧图像，请检查 CARLA 渲染")

    def stop(self) -> None:
        for actor, _ in self._streams.values():
            try:
                if actor.is_alive:
                    actor.stop()
            except RuntimeError as exc:
                print(f"[SensorManager] stop: {exc}")
