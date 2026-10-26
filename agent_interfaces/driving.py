"""Structural contract implemented by MDrive multi-ego Leaderboard adapters.

Inheritance is optional: existing AutonomousAgent subclasses already implement
this contract. Sensor collection and __call__ must use the installed MDrive
runtime so frame synchronization remains with its SensorInterface.
"""
from abc import ABC, abstractmethod


class DrivingAgent(ABC):
    """Initialized instances expose ego_vehicles_num and sensor_interface."""

    @abstractmethod
    def sensors(self):
        """Return MDrive sensor specifications with stable per-ego IDs."""

    @abstractmethod
    def set_global_plan(self, gps_routes, world_routes):
        """Accept one ordered route per ego."""

    @abstractmethod
    def run_step(self, input_data, timestamp):
        """Return ordered carla.VehicleControl objects for all egos."""

    @abstractmethod
    def __call__(self):
        """Read synchronized SensorInterface data, then dispatch run_step."""

    @abstractmethod
    def destroy(self):
        """Release model resources."""
