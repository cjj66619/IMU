from typing import Optional
import struct
import asyncio
from bleak import BleakClient, BleakScanner
from ..models.characteristics import MovellaDOTCharacteristics
from ..models.data_structures import (SensorConfiguration, DeviceInfo,
                                      SensorData)
from ..models.enums import PayloadMode, FilterProfile
from .collector import SensorDataCollector
import time


class MovellaDOTSensor:
    def __init__(self, config: SensorConfiguration = None):
        self.client: Optional[BleakClient] = None
        self.chars = MovellaDOTCharacteristics()
        self.is_connected = False
        self.config = config or SensorConfiguration()
        self.config.payload_mode = self._validate_and_adjust_payload_mode(self.config.payload_mode)
        self.data_collector = None
        self._device_address = None
        self._device_name = None
        self._device_tag = None

    def _get_payload_characteristic(self, payload_mode: PayloadMode) -> str:
        """Return the appropriate payload characteristic based on payload mode"""
        # Long payload (>40 bytes)
        if payload_mode in [
            PayloadMode.CUSTOM_MODE_5
        ]:
            return self.chars.LONG_PAYLOAD

        # Short payload (≤20 bytes)
        elif payload_mode in [
            PayloadMode.ORIENTATION_EULER,
            PayloadMode.ORIENTATION_QUATERNION,
            PayloadMode.FREE_ACCELERATION
        ]:
            return self.chars.SHORT_PAYLOAD

        # Medium payload (21-40 bytes)
        else:
            return self.chars.MEDIUM_PAYLOAD

    async def scan_and_connect(self, timeout=5.0):
        """Scan for and connect to the first available Movella DOT sensor"""
        print("Scanning for Movella DOT sensors...")
        devices = await BleakScanner.discover(timeout=timeout)
        dot_devices = [d for d in devices if d.name and "Movella DOT" in d.name]

        if not dot_devices:
            raise Exception("No Movella DOT sensors found")

        device = dot_devices[0]
        self._device_address = device.address
        self._device_name = device.name
        print(f"Connecting to {device.name} ({device.address})...")

        self.client = BleakClient(device.address)
        await self.client.connect()
        self.is_connected = True
        print("Connected successfully")

    async def reconnect(self):
        """Reconnect to the previously connected sensor"""
        if not self._device_address:
            raise Exception("No device address stored")

        print("Reconnecting...")
        try:
            self.client = BleakClient(self._device_address)
            await self.client.connect()
            self.is_connected = True
            print("Reconnected successfully")
        except Exception as e:
            print(f"Reconnection failed: {e}")
            raise

    async def configure_sensor(self):
        """Configure sensor with the current configuration settings"""
        # Configure output rate
        rate_bytes = struct.pack('<H', self.config.output_rate)
        rate_config = bytearray([
            0x10,  # Visit Index with bit 4 set for output rate
            0,  # Identifying
            0,  # Power off options
            0,  # Power saving timeout X (minute)
            0,  # Power saving timeout X (second)
            0,  # Power saving timeout Y (minute)
            0,  # Power saving timeout Y (second)
            0,  # Device Tag length
            0, 0, 0, 0, 0, 0, 0, 0,  # Device Tag (16 bytes)
            0, 0, 0, 0, 0, 0, 0, 0,
            rate_bytes[0], rate_bytes[1],  # Output rate (2 bytes)
            0,  # Filter profile index
            0, 0, 0, 0, 0  # Reserved
        ])

        await self.client.write_gatt_char(self.chars.DEVICE_CONTROL, rate_config)
        print(f"Configured output rate: {self.config.output_rate}Hz")

        # Configure filter profile
        filter_config = bytearray([
            0x20,  # Visit Index with bit 5 set for filter profile
            0,  # Identifying
            0,  # Power off options
            0,  # Power saving timeout X (minute)
            0,  # Power saving timeout X (second)
            0,  # Power saving timeout Y (minute)
            0,  # Power saving timeout Y (second)
            0,  # Device Tag length
            0, 0, 0, 0, 0, 0, 0, 0,  # Device Tag (16 bytes)
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0,  # Output rate (2 bytes)
            self.config.filter_profile,  # Filter profile index
            0, 0, 0, 0, 0  # Reserved
        ])

        await self.client.write_gatt_char(self.chars.DEVICE_CONTROL, filter_config)
        print(f"Configured filter profile: {self.config.filter_profile.name}")

        # Configure payload mode
        payload_config = bytearray([1, 1, self.config.payload_mode])
        await self.client.write_gatt_char(self.chars.MEASUREMENT_CONTROL, payload_config)
        print(f"Configured payload mode: {self.config.payload_mode.name}")

        # Initialize data collector
        self.data_collector = SensorDataCollector(
            self.config.payload_mode,
            self._device_address
        )

    def _validate_and_adjust_payload_mode(self, requested_mode: PayloadMode) -> PayloadMode:
        """Validate and adjust payload mode if necessary"""
        unsupported_modes = [
            PayloadMode.HIGH_FIDELITY_WITH_MAG,
            PayloadMode.HIGH_FIDELITY,
            PayloadMode.CUSTOM_MODE_4
        ]

        if requested_mode in unsupported_modes:
            print(f"\nWARNING: Payload mode {requested_mode.name} is not supported by this code!")
            print("This mode can only be used with the official Movella SDK.")
            print("Falling back to COMPLETE_EULER mode, which provides:")
            print("- Timestamp")
            print("- Euler angles (roll, pitch, yaw)")
            print("- Free acceleration")
            return PayloadMode.COMPLETE_EULER

        return requested_mode

    def notification_handler(self, sender: int, data: bytearray):
        """Handle incoming sensor data notifications"""
        try:
            if self.data_collector:
                parsed_data = self.data_collector.parser.parse(data)
                self.data_collector.add_data(data)

                print(f"\n实时数据来自 {self._device_tag} ({self._device_address}):")

                if parsed_data.quaternion:
                    print(f"四元数 (w,x,y,z): {parsed_data.quaternion.w:.3f}, "
                          f"{parsed_data.quaternion.x:.3f}, {parsed_data.quaternion.y:.3f}, "
                          f"{parsed_data.quaternion.z:.3f}")

                if parsed_data.euler_angles:
                    print(f"欧拉角 (roll,pitch,yaw): {parsed_data.euler_angles.roll:.1f}°, "
                          f"{parsed_data.euler_angles.pitch:.1f}°, {parsed_data.euler_angles.yaw:.1f}°")

                if parsed_data.acceleration:
                    print(f"加速度 (x,y,z): {parsed_data.acceleration.x:.2f}, "
                          f"{parsed_data.acceleration.y:.2f}, {parsed_data.acceleration.z:.2f}")

                if parsed_data.free_acceleration:
                    print(f"自由加速度 (x,y,z): {parsed_data.free_acceleration.x:.2f}, "
                          f"{parsed_data.free_acceleration.y:.2f}, {parsed_data.free_acceleration.z:.2f}")

                if parsed_data.angular_velocity:
                    print(f"角速度 (x,y,z): {parsed_data.angular_velocity.x:.2f}, "
                          f"{parsed_data.angular_velocity.y:.2f}, {parsed_data.angular_velocity.z:.2f}")

                if parsed_data.magnetic_field:
                    print(f"磁场 (x,y,z): {parsed_data.magnetic_field.x:.2f}, "
                          f"{parsed_data.magnetic_field.y:.2f}, {parsed_data.magnetic_field.z:.2f}")

                if parsed_data.status:
                    print("\n状态信息:")
                    if parsed_data.status.is_clipping_acc_x(): print("- 加速计 X 轴削波")
                    if parsed_data.status.is_clipping_acc_y(): print("- 加速计 Y 轴削波")
                    if parsed_data.status.is_clipping_acc_z(): print("- 加速计 Z 轴削波")
                    if parsed_data.status.is_clipping_gyr_x(): print("- 陀螺仪 X 轴削波")
                    if parsed_data.status.is_clipping_gyr_y(): print("- 陀螺仪 Y 轴削波")
                    if parsed_data.status.is_clipping_gyr_z(): print("- 陀螺仪 Z 轴削波")
                    if parsed_data.status.is_mag_new(): print("- 新磁场数据")

        except Exception as e:
            print(f"处理通知时出错: {e}")

    async def start_measurement(self):
        """Start measurement with notification handling"""
        print("Starting measurement...")

        payload_char = self._get_payload_characteristic(self.config.payload_mode)

        await self.client.start_notify(
            payload_char,
            self.notification_handler
        )

        await self.client.write_gatt_char(
            self.chars.MEASUREMENT_CONTROL,
            bytearray([1, 1, self.config.payload_mode])
        )

    async def stop_measurement(self):
        """Stop measurement and notifications"""
        print("Stopping measurement...")

        await self.client.write_gatt_char(
            self.chars.MEASUREMENT_CONTROL,
            bytearray([1, 0, self.config.payload_mode])
        )

        payload_char = self._get_payload_characteristic(self.config.payload_mode)
        await self.client.stop_notify(payload_char)

    async def start_recording(self, duration_seconds: int = 10):
        """Start recording data on the sensor"""
        print(f"Starting recording for {duration_seconds} seconds...")
        current_time = int(time.time())
        message = bytearray([0x01, 0x07, 0x40]) + struct.pack("<I", current_time) + struct.pack("<H", duration_seconds)
        checksum = (256 - sum(message) % 256) % 256
        message.append(checksum)
        await self.client.write_gatt_char(self.chars.MESSAGE_CONTROL, message)

    async def stop_recording(self):
        """Stop recording data on the sensor"""
        print("Stopping recording...")
        message = bytearray([0x01, 0x01, 0x41, 0xBD])
        await self.client.write_gatt_char(self.chars.MESSAGE_CONTROL, message)

    async def disconnect(self):
        """Disconnect from the sensor"""
        if self.client and self.is_connected:
            await self.client.disconnect()
            self.is_connected = False
            print("Disconnected from sensor")

    def get_collected_data(self):
        """Get the collected data in various formats"""
        if not self.data_collector:
            return None

        # --- 修改部分：添加 'angular_velocities' ---
        return {
            'device_tag': self._device_tag,
            'mac_address': self._device_address,
            'timestamps': self.data_collector.get_timestamps(),
            'quaternions': self.data_collector.get_quaternions(),
            'euler_angles': self.data_collector.get_euler_angles(),
            'accelerations': self.data_collector.get_accelerations(),
            'angular_velocities': self.data_collector.get_angular_velocities()  # 添加了角速度
        }
        # ----------------------------------------

    async def get_device_info(self) -> DeviceInfo:
        """Get device information from the sensor"""
        try:
            info_data = await self.client.read_gatt_char(self.chars.BASE_UUID.format(0x1001))

            # Parse MAC Address
            mac_bytes = info_data[0:6][::-1]
            mac = ':'.join([f'{b:02X}' for b in mac_bytes])

            # Parse firmware version
            version_major = info_data[6]
            version_minor = info_data[7]
            version_revision = info_data[8]
            firmware_version = f"{version_major}.{version_minor}.{version_revision}"

            # Parse serial number
            serial_number_bytes = info_data[20:28]
            serial_number = ''.join(f'{byte:02X}' for byte in reversed(serial_number_bytes))

            # Parse product code
            try:
                product_code = bytes(info_data[28:34]).decode('ascii').rstrip('\x00')
            except:
                product_code = ' '.join([f'{b:02X}' for b in info_data[28:34]])

            # Read Device Control for current settings
            control_data = await self.client.read_gatt_char(self.chars.DEVICE_CONTROL)

            # Parse device tag
            tag_length = control_data[7]
            device_tag = bytes(control_data[8:8 + tag_length]).decode('ascii')

            # Parse output rate and filter profile
            output_rate = int.from_bytes(control_data[24:26], byteorder='little')
            filter_profile = FilterProfile(control_data[26])

            return DeviceInfo(
                mac_address=mac,
                firmware_version=firmware_version,
                serial_number=serial_number,
                product_code=product_code,
                device_tag=device_tag,
                output_rate=output_rate,
                filter_profile=filter_profile
            )
        except Exception as e:
            print(f"Error getting device info: {e}")
            raise

    async def identify_sensor(self):
        """Make the sensor LED blink for identification"""
        try:
            identify_config = bytearray([
                                            0x01,  # Visit Index with bit 0 set for identifying
                                            0x01,  # Identifying set to 0x01
                                            0] + [0] * 29)  # Rest of the configuration bytes

            await self.client.write_gatt_char(self.chars.DEVICE_CONTROL, identify_config)
            print("Sensor LED should blink 8 times in red")
        except Exception as e:
            print(f"Error identifying sensor: {e}")
            raise

    async def power_off_sensor(self):
        """Power off the sensor
        According to Table 7: Set bit 1 in Visit Index and set Power off bit to 1
        """
        try:
            power_off_config = bytearray([
                0x02,  # Visit Index with bit 1 set for power off
                0,  # Identifying
                0x01,  # Power off bit set to 1
                0,  # Power saving timeout X (minute)
                0,  # Power saving timeout X (second)
                0,  # Power saving timeout Y (minute)
                0,  # Power saving timeout Y (second)
                0,  # Device Tag length
                0, 0, 0, 0, 0, 0, 0, 0,  # Device Tag (16 bytes)
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0,  # Output rate (2 bytes)
                0,  # Filter profile index
                0, 0, 0, 0, 0  # Reserved
            ])

            await self.client.write_gatt_char(self.chars.DEVICE_CONTROL, power_off_config)
            print("Sensor powered off")
        except Exception as e:
            print(f"Error powering off sensor: {e}")
            raise