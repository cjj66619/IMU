from dataclasses import dataclass

@dataclass
class MovellaDOTCharacteristics:
    """Movella DOT BLE characteristics"""
    BASE_UUID = "1517{:04x}-4947-11e9-8646-d663bd873d93"
    
    # Service UUIDs
    CONFIG_SERVICE = BASE_UUID.format(0x1000)
    MEASUREMENT_SERVICE = BASE_UUID.format(0x2000)
    MESSAGE_SERVICE = BASE_UUID.format(0x7000)
    
    # Characteristic UUIDs
    DEVICE_CONTROL = BASE_UUID.format(0x1002)
    MEASUREMENT_CONTROL = BASE_UUID.format(0x2001)
    LONG_PAYLOAD = BASE_UUID.format(0x2002)
    MEDIUM_PAYLOAD = BASE_UUID.format(0x2003)
    SHORT_PAYLOAD = BASE_UUID.format(0x2004)
    MESSAGE_CONTROL = BASE_UUID.format(0x7001)
    MESSAGE_ACK = BASE_UUID.format(0x7002)
    MESSAGE_NOTIFY = BASE_UUID.format(0x7003)