from typing import List
import numpy as np
from ..models.enums import PayloadMode
from ..models.data_structures import SensorData
from .parser import PayloadParser


class SensorDataCollector:
    """Collects and stores sensor data"""

    def __init__(self, payload_mode: PayloadMode, mac_address: str = None):
        self.parser = PayloadParser(payload_mode)
        self.data: List[SensorData] = []
        self.mac_address = mac_address

    def add_data(self, raw_data: bytes):
        """Parse and add new sensor data"""
        parsed_data = self.parser.parse(raw_data)
        self.data.append(parsed_data)

    def clear(self):
        """Clear collected data"""
        self.data.clear()

    def get_timestamps(self) -> np.ndarray:
        """Get array of timestamps"""
        return np.array([d.timestamp.microseconds for d in self.data])

    def get_quaternions(self) -> np.ndarray:
        """Get array of quaternions"""
        return np.array([d.quaternion.to_numpy() for d in self.data if d.quaternion])

    def get_euler_angles(self) -> np.ndarray:
        """Get array of euler angles"""
        return np.array([d.euler_angles.to_numpy() for d in self.data if d.euler_angles])

    def get_accelerations(self) -> np.ndarray:
        """Get array of accelerations"""
        return np.array([d.acceleration.to_numpy() for d in self.data if d.acceleration])

    def get_free_accelerations(self) -> np.ndarray:
        """Get array of free accelerations"""
        return np.array([d.free_acceleration.to_numpy() for d in self.data if d.free_acceleration])

    # --- 修改部分：添加 get_angular_velocities 方法 ---
    def get_angular_velocities(self) -> np.ndarray:
        """Get array of angular velocities"""
        return np.array([d.angular_velocity.to_numpy() for d in self.data if d.angular_velocity])

    # ----------------------------------------------

    def get_status_values(self) -> np.ndarray:
        """Get array of status values"""
        return np.array([d.status.value for d in self.data if d.status])

    def get_acc_clipping_counts(self) -> np.ndarray:
        """Get array of accelerometer clipping counts"""
        return np.array([d.clipping_acc for d in self.data if d.clipping_acc is not None])

    def get_gyr_clipping_counts(self) -> np.ndarray:
        """Get array of gyroscope clipping counts"""
        return np.array([d.clipping_gyr for d in self.data if d.clipping_gyr is not None])