from dataclasses import dataclass
from typing import Optional
import struct
import numpy as np
from .enums import FilterProfile, OutputRate, PayloadMode

@dataclass
class DeviceInfo:
    """Device information structure"""
    mac_address: str
    firmware_version: str
    serial_number: int
    product_code: str
    device_tag: str
    output_rate: int
    filter_profile: FilterProfile

@dataclass
class SensorConfiguration:
    """Sensor configuration parameters"""
    output_rate: OutputRate = OutputRate.RATE_60
    filter_profile: FilterProfile = FilterProfile.GENERAL
    payload_mode: PayloadMode = PayloadMode.COMPLETE_EULER

@dataclass
class Timestamp:
    """Timestamp in microseconds"""
    microseconds: int

    @classmethod
    def from_bytes(cls, data: bytes) -> 'Timestamp':
        return cls(struct.unpack('<I', data[:4])[0])

@dataclass
class Quaternion:
    """Quaternion orientation data"""
    w: float
    x: float
    y: float
    z: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'Quaternion':
        return cls(*struct.unpack('<4f', data[:16]))

    def to_numpy(self) -> np.ndarray:
        return np.array([self.w, self.x, self.y, self.z])

@dataclass
class EulerAngles:
    """Euler angles in degrees"""
    roll: float
    pitch: float
    yaw: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'EulerAngles':
        return cls(*struct.unpack('<3f', data[:12]))

    def to_numpy(self) -> np.ndarray:
        return np.array([self.roll, self.pitch, self.yaw])

@dataclass
class Vector3:
    """3D vector for acceleration, angular velocity, etc."""
    x: float
    y: float
    z: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'Vector3':
        return cls(*struct.unpack('<3f', data[:12]))

    def to_numpy(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

@dataclass
class MagneticField:
    """Magnetic field data"""
    x: float
    y: float
    z: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'MagneticField':
        TWO_POW_TWELVE = 2 ** 12
        if len(data) != 6:
            raise ValueError("Magnetic field data must be 6 bytes.")
        x, y, z = struct.unpack('<hhh', data)
        return cls(x / TWO_POW_TWELVE,
                  y / TWO_POW_TWELVE,
                  z / TWO_POW_TWELVE)

    def to_numpy(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

@dataclass
class Status:
    """Status information"""
    value: int

    def is_clipping_acc_x(self) -> bool:
        return bool(self.value & 0x0001)
    
    def is_clipping_acc_y(self) -> bool:
        return bool(self.value & 0x0002)
    
    def is_clipping_acc_z(self) -> bool:
        return bool(self.value & 0x0004)
    
    def is_clipping_gyr_x(self) -> bool:
        return bool(self.value & 0x0008)
    
    def is_clipping_gyr_y(self) -> bool:
        return bool(self.value & 0x0010)
    
    def is_clipping_gyr_z(self) -> bool:
        return bool(self.value & 0x0020)
    
    def is_mag_new(self) -> bool:
        return bool(self.value & 0x0200)

    @classmethod
    def from_bytes(cls, data: bytes) -> 'Status':
        return cls(struct.unpack('<H', data[:2])[0])

@dataclass
class SensorData:
    """Container for all possible sensor data types"""
    timestamp: Optional[Timestamp] = None
    quaternion: Optional[Quaternion] = None
    euler_angles: Optional[EulerAngles] = None
    free_acceleration: Optional[Vector3] = None
    acceleration: Optional[Vector3] = None
    angular_velocity: Optional[Vector3] = None
    magnetic_field: Optional[MagneticField] = None
    delta_q: Optional[Quaternion] = None
    delta_v: Optional[Vector3] = None
    status: Optional[Status] = None
    clipping_acc: Optional[int] = None
    clipping_gyr: Optional[int] = None