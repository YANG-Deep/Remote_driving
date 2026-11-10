"""Only mock the simulator boundary; verify our rollback and destroy contracts."""

import unittest
from unittest.mock import patch
from types import SimpleNamespace
from core.carla_manager import CarlaManager
import carla


class LifecycleTests(unittest.TestCase):
    def test_weather_failure_restores_settings_when_constructor_never_returns(self):
        calls = []
        original = carla.WorldSettings(synchronous_mode=False)
        world = SimpleNamespace(
            get_map=lambda: SimpleNamespace(name="Town10HD"),
            get_settings=lambda: carla.WorldSettings(synchronous_mode=False),
            apply_settings=lambda settings: calls.append(settings.synchronous_mode),
            set_weather=lambda weather: (_ for _ in ()).throw(
                RuntimeError("weather failed")
            ),
        )
        client = SimpleNamespace(
            set_timeout=lambda value: None, get_world=lambda: world
        )
        with patch("core.carla_manager.carla.Client", return_value=client):
            with self.assertRaisesRegex(RuntimeError, "weather failed"):
                CarlaManager()
        self.assertEqual(calls, [True, False])

    def test_sensor_stop_failure_does_not_leak_sensor_or_parent(self):
        events = []
        manager = CarlaManager.__new__(CarlaManager)
        parent = SimpleNamespace(
            is_alive=True,
            type_id="vehicle.test",
            destroy=lambda: events.append("vehicle"),
        )
        sensor = SimpleNamespace(
            is_alive=True,
            type_id="sensor.other.radar",
            stop=lambda: (_ for _ in ()).throw(RuntimeError("already stopped")),
            destroy=lambda: events.append("sensor"),
        )
        manager._spawned_actors = [parent, sensor]
        manager._original_settings = None
        manager.destroy()
        self.assertEqual(events, ["sensor", "vehicle"])
