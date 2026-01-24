# Movella DOT Python SDK

A Python SDK for interfacing with Movella DOT sensors via Bluetooth Low Energy (BLE). This package provides a simple and intuitive way to connect to, configure, and collect data from Movella DOT sensors.

## Features

- BLE connection and device discovery
- Multiple sensor support
- Real-time data streaming
- Various payload modes for different data types
- Configurable output rates and filter profiles
- Device information and status monitoring
- Battery status monitoring
- Power management features

## Requirements

- Python 3.7 or higher
- Windows 10 or higher (for BLE support)
- Movella DOT sensor(s)

## Installation

1. Create and activate a virtual environment (recommended):
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install required packages:
```bash
pip install -r requirements.txt
```

3. Install the package:
```bash
pip install -e .
```

## Requirements.txt
```
bleak>=0.22.3
numpy>=1.24.4
```

## Usage

### Basic Example

```python
from movella_dot_py.core.sensor import MovellaDOTSensor
from movella_dot_py.models.data_structures import SensorConfiguration
from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode

# Create configuration
config = SensorConfiguration(
    output_rate=OutputRate.RATE_60,
    filter_profile=FilterProfile.GENERAL,
    payload_mode=PayloadMode.COMPLETE_EULER
)

# Create sensor instance
sensor = MovellaDOTSensor(config)
```

### Running the Demo

The package includes a comprehensive demo script that shows how to work with multiple sensors:

```bash
python examples/multi_dot_demo.py
```

## Configuration Options

### Output Rates (OutputRate)
- `RATE_1`: 1 Hz
- `RATE_4`: 4 Hz
- `RATE_10`: 10 Hz
- `RATE_12`: 12 Hz
- `RATE_15`: 15 Hz
- `RATE_20`: 20 Hz
- `RATE_30`: 30 Hz
- `RATE_60`: 60 Hz
- `RATE_120`: 120 Hz (Recording only)

### Filter Profiles (FilterProfile)
- `GENERAL`: Default profile for general human motion
- `DYNAMIC`: Optimized for fast and jerky human motion like sprinting

### Payload Modes (PayloadMode)
1. Basic Orientations:
   - `ORIENTATION_EULER`: Euler angles
   - `ORIENTATION_QUATERNION`: Quaternion orientation
   - `COMPLETE_EULER`: Euler angles with free acceleration
   - `COMPLETE_QUATERNION`: Quaternion with free acceleration

2. Extended Data:
   - `EXTENDED_EULER`: Euler angles, free acceleration, status, and clipping data
   - `EXTENDED_QUATERNION`: Quaternion, free acceleration, status, and clipping data

3. Motion Data:
   - `FREE_ACCELERATION`: Linear acceleration with gravity compensated
   - `RATE_QUANTITIES`: Acceleration and angular velocity
   - `RATE_QUANTITIES_WITH_MAG`: Acceleration, angular velocity, and magnetic field

4. Custom Modes:
   - `CUSTOM_MODE_1`: Euler angles, free acceleration, angular velocity
   - `CUSTOM_MODE_2`: Euler angles, free acceleration, magnetic field
   - `CUSTOM_MODE_3`: Quaternion and angular velocity
   - `CUSTOM_MODE_5`: Quaternion, acceleration, angular velocity

### Example Configuration

```python
# High-frequency dynamic motion configuration
config = SensorConfiguration(
    output_rate=OutputRate.RATE_60,
    filter_profile=FilterProfile.DYNAMIC,
    payload_mode=PayloadMode.CUSTOM_MODE_5  # Full motion data
)

# Basic orientation tracking configuration
config = SensorConfiguration(
    output_rate=OutputRate.RATE_30,
    filter_profile=FilterProfile.GENERAL,
    payload_mode=PayloadMode.ORIENTATION_EULER
)
```

## Data Collection

The SDK provides various methods to access collected data:

- Quaternions
- Euler angles
- Acceleration (with and without gravity)
- Angular velocity
- Magnetic field data
- Status information
- Clipping detection

## Additional Features

- Device identification (LED blinking)
- Battery status monitoring
- Power saving options
- Device tag configuration
- Time synchronization
- Factory reset capabilities

## Known Limitations

- Some payload modes (`HIGH_FIDELITY`, `HIGH_FIDELITY_WITH_MAG`, `CUSTOM_MODE_4`) are only available with the official Movella SDK
- Maximum of 5 simultaneous sensor connections recommended
- Windows 10 or higher required for BLE functionality


## Acknowledgments

- [Movella DOT BLE Service Specification](https://www.xsens.com/hubfs/Downloads/Manuals/Xsens%20DOT%20BLE%20Services%20Specifications.pdf)
- [Movella DOT User Manual](https://www.movella.com/hubfs/Movella%20DOT%20User%20Manual.pdf)
- [Bleak](https://github.com/hbldh/bleak) BLE library