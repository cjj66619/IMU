import asyncio
from bleak import BleakScanner

async def scan():
    print("正在扫描周围的蓝牙设备 (5秒)...")
    devices = await BleakScanner.discover(timeout=5.0)
    
    print("\n发现以下 Movella/Xsens 设备:")
    print("=" * 40)
    found = False
    for d in devices:
        # Movella DOT 的名字通常包含 "Movella" 或 "Xsens"
        if d.name and ("Movella" in d.name or "Xsens" in d.name):
            print(f"名称: {d.name}")
            print(f"地址: {d.address}")
            print("-" * 40)
            found = True
    
    if not found:
        print("未发现相关设备，请确保传感器已开机（灯闪烁）且未连接手机 APP。")

if __name__ == "__main__":
    asyncio.run(scan())