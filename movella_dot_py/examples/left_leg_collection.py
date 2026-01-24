import asyncio
import sys
import os
import csv
import time
from datetime import datetime
import numpy as np
from bleak import BleakScanner, BleakClient

# --- 路径设置 ---
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
# 向上寻找 SDK
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

try:
    from movella_dot_py.core.sensor import MovellaDOTSensor
    from movella_dot_py.models.data_structures import SensorConfiguration
    from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode
except ImportError:
    print("错误: 找不到 movella_dot_py 库。")
    sys.exit(1)

# ==========================================
# 🎯 目标传感器地址 (确保填对)
# ==========================================
TARGET_ADDRESSES = [
    "D4:22:CD:00:7F:39",
    "D4:22:CD:00:7F:3A",
    "D4:22:CD:00:7F:2D"
]
COLLECTION_TIME_SEC = 20  # 采集时长
# ==========================================

G_CONST = 9.80665
DEG_TO_RAD = np.pi / 180.0

def quaternion_to_euler_rad(q):
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

async def connect_safe(address, config):
    print(f"\n[正在处理] {address} ...")
    
    # 1. 即时扫描 (解决 Object Closed 问题)
    device = await BleakScanner.find_device_by_address(address, timeout=10.0)
    if not device:
        print(f"  ❌ 扫描未找到 {address}")
        return None
        
    sensor = MovellaDOTSensor(config)
    sensor.custom_mac = address
    sensor.client = BleakClient(device)
    
    # 2. 尝试连接 (带重试)
    for attempt in range(3):
        try:
            print(f"  -> 尝试连接 (第{attempt+1}次)...", end=" ")
            await sensor.client.connect(timeout=15.0)
            if sensor.client.is_connected:
                print("✅ 成功!")
                await asyncio.sleep(0.5) # 喘息
                await sensor.identify_sensor()
                await sensor.configure_sensor()
                return sensor
        except Exception as e:
            print(f"失败: {e}")
            await asyncio.sleep(2.0) # 失败后的冷却
            
    return None

async def save_sensor_data(sensor, timestamp_str):
    """单独封装的保存函数，确保能被调用"""
    try:
        data = sensor.get_collected_data()
        if not data or len(data['timestamps']) == 0:
            print(f"  ⚠️ {sensor.custom_mac[-5:]}: 内存无数据")
            return

        # 文件名: MAC后两位_时间.csv
        short_mac = sensor.custom_mac.replace(':', '')[-4:]
        filename = f"Data_{short_mac}_{timestamp_str}.csv"
        
        timestamps = data['timestamps']
        t0 = timestamps[0]
        ts_rel = (timestamps - t0) / 1000.0
        
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(["Time_ms", "AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ", "Roll", "Pitch", "Yaw"])
            
            for i in range(len(timestamps)):
                acc = data['accelerations'][i] / G_CONST
                gyr = data['angular_velocities'][i] * DEG_TO_RAD
                r, p, y = quaternion_to_euler_rad(data['quaternions'][i])
                writer.writerow([
                    f"{ts_rel[i]:.2f}",
                    f"{acc[0]:.4f}", f"{acc[1]:.4f}", f"{acc[2]:.4f}",
                    f"{gyr[0]:.4f}", f"{gyr[1]:.4f}", f"{gyr[2]:.4f}",
                    f"{r:.4f}", f"{p:.4f}", f"{y:.4f}"
                ])
        print(f"  💾 已抢救保存 -> {filename} ({len(timestamps)}帧)")
        
    except Exception as e:
        print(f"  ❌ 保存 {sensor.custom_mac} 失败: {e}")

async def main():
    print("=== IMU 采集 (数据抢救版) ===")
    
    config = SensorConfiguration(
        output_rate=OutputRate.RATE_30, # 保持 30Hz 比较稳
        filter_profile=FilterProfile.DYNAMIC,
        payload_mode=PayloadMode.CUSTOM_MODE_5
    )
    
    sensors = []
    
    # 1. 逐个连接
    for addr in TARGET_ADDRESSES:
        s = await connect_safe(addr, config)
        if s: 
            sensors.append(s)
            print("  (等待 2s 保护蓝牙带宽...)")
            await asyncio.sleep(2.0)
            
    if not sensors:
        print("无设备连接。")
        return

    # 2. 采集
    print(f"\n✅ {len(sensors)} 个设备就绪。3秒后开始...")
    await asyncio.sleep(3)
    
    try:
        # 启动
        await asyncio.gather(*(s.start_measurement() for s in sensors))
        print("🚀 开始采集! (20秒)")
        
        for i in range(COLLECTION_TIME_SEC, 0, -1):
            print(f"  倒计时: {i} ", end='\r')
            await asyncio.sleep(1)
        print("\n⏳ 时间到!")

    except Exception as e:
        print(f"\n❌ 采集过程出错: {e}")

    # 3. 停止并保存 (关键修改：无论上面是否报错，这里都会执行)
    print("\n💾 进入保存阶段 (串行处理以防崩溃)...")
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    for s in sensors:
        print(f"处理 {s.custom_mac[-5:]} ...")
        
        # A. 尝试停止 (即使失败也没关系，主要是为了发指令)
        try:
            if s.client and s.client.is_connected:
                await s.stop_measurement()
        except Exception as e:
            print(f"  (停止指令发送失败，忽略: {e})")
            
        # B. 必须保存 (从内存读取数据)
        await save_sensor_data(s, timestamp_str)
        
        # C. 断开连接
        try:
            if s.client: await s.client.disconnect()
        except:
            pass
            
        # D. 休息一下，防止下一个设备卡死
        await asyncio.sleep(1.0)

    print("\n程序结束。请检查当前目录下是否有 CSV 文件。")

if __name__ == "__main__":
    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    asyncio.run(main())