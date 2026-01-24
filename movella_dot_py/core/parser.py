from ..models.enums import PayloadMode
from ..models.data_structures import (SensorData, Timestamp, Quaternion,
                                      EulerAngles, Vector3, MagneticField, Status)


class PayloadParser:
    """Parser for different payload types"""

    def __init__(self, payload_mode: PayloadMode):
        self.payload_mode = payload_mode
        self.parse_map = {
            PayloadMode.EXTENDED_QUATERNION: self._parse_extended_quaternion,
            PayloadMode.COMPLETE_QUATERNION: self._parse_complete_quaternion,
            PayloadMode.ORIENTATION_EULER: self._parse_orientation_euler,
            PayloadMode.ORIENTATION_QUATERNION: self._parse_orientation_quaternion,
            PayloadMode.FREE_ACCELERATION: self._parse_free_acceleration,
            PayloadMode.EXTENDED_EULER: self._parse_extended_euler,
            PayloadMode.COMPLETE_EULER: self._parse_complete_euler,
            PayloadMode.DELTA_QUANTITIES: self._parse_delta_quantities,
            PayloadMode.DELTA_QUANTITIES_WITH_MAG: self._parse_delta_quantities_with_mag,
            PayloadMode.RATE_QUANTITIES: self._parse_rate_quantities,
            PayloadMode.RATE_QUANTITIES_WITH_MAG: self._parse_rate_quantities_with_mag,
            PayloadMode.CUSTOM_MODE_1: self._parse_custom_mode_1,
            PayloadMode.CUSTOM_MODE_2: self._parse_custom_mode_2,
            PayloadMode.CUSTOM_MODE_3: self._parse_custom_mode_3,
            PayloadMode.CUSTOM_MODE_5: self._parse_custom_mode_5,
        }

    def parse(self, data: bytes) -> SensorData:
        """Parse payload data according to current payload mode"""
        if self.payload_mode not in self.parse_map:
            raise ValueError(f"Unsupported payload mode: {self.payload_mode}")
        return self.parse_map[self.payload_mode](data)

    def _parse_extended_quaternion(self, data: bytes) -> SensorData:
        """Parse Extended Quaternion payload (36 bytes)
        - Timestamp (4)
        - Quaternion (16)
        - Free acceleration (12)
        - Status (2)
        - Clipping Count Accelerometer (1)
        - Clipping Count Gyroscope (1)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            quaternion=Quaternion.from_bytes(data[4:20]),
            free_acceleration=Vector3.from_bytes(data[20:32]),
            status=Status.from_bytes(data[32:34]),
            clipping_acc=data[34],
            clipping_gyr=data[35]
        )

    def _parse_complete_quaternion(self, data: bytes) -> SensorData:
        """Parse Complete Quaternion payload (32 bytes)"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            quaternion=Quaternion.from_bytes(data[4:20]),
            free_acceleration=Vector3.from_bytes(data[20:32])
        )

    def _parse_orientation_euler(self, data: bytes) -> SensorData:
        """Parse Orientation Euler payload (16 bytes)"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            euler_angles=EulerAngles.from_bytes(data[4:16])
        )

    def _parse_orientation_quaternion(self, data: bytes) -> SensorData:
        """Parse Orientation Quaternion payload (20 bytes)"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            quaternion=Quaternion.from_bytes(data[4:20])
        )

    def _parse_free_acceleration(self, data: bytes) -> SensorData:
        """Parse Free Acceleration payload (16 bytes)"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            free_acceleration=Vector3.from_bytes(data[4:16])
        )

    def _parse_extended_euler(self, data: bytes) -> SensorData:
        """Parse Extended Euler payload (32 bytes)
        - Timestamp (4)
        - Euler angles (12)
        - Free acceleration (12)
        - Status (2)
        - Clipping Count Accelerometer (1)
        - Clipping Count Gyroscope (1)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            euler_angles=EulerAngles.from_bytes(data[4:16]),
            free_acceleration=Vector3.from_bytes(data[16:28]),
            status=Status.from_bytes(data[28:30]),
            clipping_acc=data[30],
            clipping_gyr=data[31]
        )

    def _parse_complete_euler(self, data: bytes) -> SensorData:
        """Parse Complete Euler payload (28 bytes)"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            euler_angles=EulerAngles.from_bytes(data[4:16]),
            free_acceleration=Vector3.from_bytes(data[16:28])
        )

    def _parse_delta_quantities(self, data: bytes) -> SensorData:
        """Parse Delta Quantities payload (32 bytes)"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            delta_q=Quaternion.from_bytes(data[4:20]),
            delta_v=Vector3.from_bytes(data[20:32])
        )

    def _parse_delta_quantities_with_mag(self, data: bytes) -> SensorData:
        """Parse Delta Quantities with Mag payload (38 bytes)"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            delta_q=Quaternion.from_bytes(data[4:20]),
            delta_v=Vector3.from_bytes(data[20:32]),
            magnetic_field=MagneticField.from_bytes(data[32:38])
        )

    def _parse_rate_quantities(self, data: bytes) -> SensorData:
        """Parse Rate Quantities payload (28 bytes)"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            acceleration=Vector3.from_bytes(data[4:16]),
            angular_velocity=Vector3.from_bytes(data[16:28])
        )

    def _parse_rate_quantities_with_mag(self, data: bytes) -> SensorData:
        """Parse Rate Quantities with Mag payload (34 bytes)
        - Timestamp (4)
        - Acceleration (12)
        - Angular velocity (12)
        - Magnetic field (6)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            acceleration=Vector3.from_bytes(data[4:16]),
            angular_velocity=Vector3.from_bytes(data[16:28]),
            magnetic_field=MagneticField.from_bytes(data[28:34])
        )

    def _parse_custom_mode_1(self, data: bytes) -> SensorData:
        """Parse Custom Mode 1 payload (40 bytes)
        - Timestamp (4)
        - Euler angles (12)
        - Free acceleration (12)
        - Angular velocity (12)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            euler_angles=EulerAngles.from_bytes(data[4:16]),
            free_acceleration=Vector3.from_bytes(data[16:28]),
            angular_velocity=Vector3.from_bytes(data[28:40])
        )

    def _parse_custom_mode_2(self, data: bytes) -> SensorData:
        """Parse Custom Mode 2 payload (34 bytes)
        - Timestamp (4)
        - Euler angles (12)
        - Free acceleration (12)
        - Magnetic field (6)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            euler_angles=EulerAngles.from_bytes(data[4:16]),
            free_acceleration=Vector3.from_bytes(data[16:28]),
            magnetic_field=MagneticField.from_bytes(data[28:34])
        )

    def _parse_custom_mode_3(self, data: bytes) -> SensorData:
        """Parse Custom Mode 3 payload (32 bytes)
        - Timestamp (4)
        - Quaternion (16)
        - Angular velocity (12)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            quaternion=Quaternion.from_bytes(data[4:20]),
            angular_velocity=Vector3.from_bytes(data[20:32])
        )

    def _parse_custom_mode_5(self, data: bytes) -> SensorData:
        """Parse Custom Mode 5 payload (44 bytes)
        Contains:
        - Timestamp (4)
        - Quaternion (16)
        - Acceleration (12)
        - Angular velocity (12)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            quaternion=Quaternion.from_bytes(data[4:20]),
            acceleration=Vector3.from_bytes(data[20:32]),
            angular_velocity=Vector3.from_bytes(data[32:44])
        )