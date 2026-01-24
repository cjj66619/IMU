import asyncio
from bleak import BleakScanner

async def main():
    print("正在测试 Windows 蓝牙驱动是否存活...")
    try:
        # 扫描 5 秒
        devices = await BleakScanner.discover(timeout=5.0)
        print(f"扫描成功! 发现了 {len(devices)} 个设备。")
        
        movella_count = 0
        for d in devices:
            # 这里的 MAC 地址前缀通常是 D4:22:CD
            if d.name and "Movella" in d.name:
                print(f"  -> 发现 Movella 设备: {d.name} [{d.address}]")
                movella_count += 1
            elif d.address.startswith("D4:22:CD"):
                print(f"  -> 发现疑似设备: {d.address}")
                movella_count += 1
                
        if movella_count > 0:
            print("\n✅ 结论: 蓝牙驱动工作正常，传感器可见。")
        else:
            print("\n⚠️ 结论: 蓝牙驱动活着，但没看见传感器。请检查传感器是否在闪烁。")
            
    except Exception as e:
        print(f"\n❌ 严重错误: 蓝牙驱动依然无法响应。\n错误信息: {e}")

if __name__ == "__main__":
    asyncio.run(main())