from enum import IntEnum

class OutputRate(IntEnum):
    """Available output rates in Hz"""
    RATE_1 = 1
    RATE_4 = 4
    RATE_10 = 10
    RATE_12 = 12
    RATE_15 = 15
    RATE_20 = 20
    RATE_30 = 30
    RATE_60 = 60
    RATE_120 = 120  # Recording only

class FilterProfile(IntEnum):
    """Available filter profiles"""
    GENERAL = 0     # Default for general human motion
    DYNAMIC = 1     # For fast and jerky human motion like sprinting

class PayloadMode(IntEnum):
    """Available payload modes"""
    HIGH_FIDELITY_WITH_MAG = 1    # Movella SDK only
    EXTENDED_QUATERNION = 2
    COMPLETE_QUATERNION = 3
    ORIENTATION_EULER = 4
    ORIENTATION_QUATERNION = 5
    FREE_ACCELERATION = 6
    EXTENDED_EULER = 7
    COMPLETE_EULER = 16
    HIGH_FIDELITY = 17            # Movella SDK only
    DELTA_QUANTITIES_WITH_MAG = 18
    DELTA_QUANTITIES = 19
    RATE_QUANTITIES_WITH_MAG = 20
    RATE_QUANTITIES = 21
    CUSTOM_MODE_1 = 22
    CUSTOM_MODE_2 = 23
    CUSTOM_MODE_3 = 24
    CUSTOM_MODE_4 = 25            # Movella SDK only
    CUSTOM_MODE_5 = 26