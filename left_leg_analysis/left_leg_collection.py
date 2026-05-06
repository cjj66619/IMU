import asyncio
import sys
import os
import csv
import time
from datetime import datetime
import numpy as np
from bleak import BleakScanner, BleakClient
import winsound

# ==========================================
# 🔧 智能路径配置 (自动寻找 SDK)
# ==========================================
current_dir = os.path.dirname(os.path.abspath(__file__))

sys.path.append(current_dir)
sys.path.append(os.path.dirname(current_dir)) 
sys.path.append(os.path.dirname(os.path.dirname(current_dir))) 

try:
    from movella_dot_py.core.sensor import MovellaDOTSensor
    from movella_dot_py.models.data_structures import SensorConfiguration
    from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode
except ImportError:
    print("❌ 严重错误: 无法找到 'movella_dot_py' SDK。")
    print(f"请确认 'movella_dot_py' 文件夹位于 {os.path.dirname(current_dir)} 或其附近。")
    sys.exit(1)

# ==========================================
# ⚙️ [配置区域] MAC 地址设置
# ==========================================
SENSOR_MAP = {
    "D4:22:CD:00:7F:3A": "Left_Shank",
    "D4:22:CD:00:7F:30": "Left_Foot"
}

TARGET_ADDRESSES = set(SENSOR_MAP.keys())
COLLECTION_TIME_SEC = 30  # 采集时长 (秒)
OUTPUT_RATE = OutputRate.RATE_30 # 采样率 60Hz

# ==========================================
# 物理常量与计算函数
# ==========================================
G_CONST = 9.7949
DEG_TO_RAD = np.pi / 180.0

def quaternion_to_euler_rad(q):
    """四元数 (w, x, y, z) 转欧拉角 (Roll, Pitch, Yaw)"""
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

async def connect_sensor_with_retry(address, role, config):
    """单独封装的连接函数，带有深度重试机制"""
    sensor = MovellaDOTSensor(config)
    sensor.custom_role = role
    sensor.custom_mac = address
    
    max_retries = 5
    
    for attempt in range(max_retries):
        try:
            print(f"  [{role}] 正在尝试连接 (第 {attempt+1}/{max_retries} 次)...", end=" ", flush=True)
            
            device = await BleakScanner.find_device_by_address(address, timeout=10.0)
            
            if not device:
                print("❌ 扫描未找到，稍后重试...")
                await asyncio.sleep(2.0)
                continue
                
            sensor.client = BleakClient(device)
            await sensor.client.connect(timeout=25.0)
            
            if sensor.client.is_connected:
                print("✅ 成功!")
                print(f"    -> 配置 {role}...", end=" ")
                await sensor.identify_sensor()
                await sensor.configure_sensor()
                print("完成。")
                return sensor
            else:
                print("⚠️ 连接状态未知。")
                
        except Exception as e:
            print(f"\n    ⚠️ 失败: {e}")
            try:
                if sensor.client: await sensor.client.disconnect()
            except: pass
            
            if attempt < max_retries - 1:
                wait_time = 4.0 + attempt
                print(f"    ⏳ 等待 {wait_time:.1f} 秒...")
                await asyncio.sleep(wait_time)
            else:
                print(f"    ❌ 连接 {role} 失败。")
                
    return None

async def main():
    print("=== 左小腿步态数据采集 (蓝牙实时防拥塞版) ===")
    
    save_dir = os.path.join(current_dir, "data_log")
    os.makedirs(save_dir, exist_ok=True)
    print(f"📂 数据存储路径: {save_dir}")
    print("-" * 50)

    # 1. 预扫描
    print("\n[1/5] 预扫描环境...")
    devices = await BleakScanner.discover(timeout=5.0)
    found = [d for d in devices if d.address in TARGET_ADDRESSES]
    print(f"  -> 发现 {len(found)} 个目标设备")

    if not found:
        print("❌ 未发现目标传感器，请检查开机。")
        return

    # 2. 配置
    config = SensorConfiguration(
        output_rate=OUTPUT_RATE,
        filter_profile=FilterProfile.GENERAL,
        payload_mode=PayloadMode.CUSTOM_MODE_5
    )

    sensors = []

    # 3. 连接
    print("\n[2/5] 建立连接...")
    for address, role in SENSOR_MAP.items():
        sensor = await connect_sensor_with_retry(address, role, config)
        if sensor:
            sensors.append(sensor)
            if len(sensors) < len(SENSOR_MAP):
                print("  (蓝牙冷却 3s...)")
                await asyncio.sleep(3.0)

    if not sensors:
        return

    # 4. 采集 (带声音倒计时)
    try:
        print(f"\n[3/5] 准备就绪。")
        print("=" * 40)
        
        PRE_START_COUNTDOWN = 5
        print(f"🔊 听觉指令模式：将在 {PRE_START_COUNTDOWN} 秒后发出开始信号...")
        print("   (听到长音 '哔——' 请立即行走)")

        for i in range(PRE_START_COUNTDOWN, 0, -1):
            print(f"⏰ 倒计时: {i} ... ", end='\r', flush=True)
            winsound.Beep(600, 200)
            await asyncio.sleep(0.8)
            
        print(f"🚀 GO! 开始行走! (记录 {COLLECTION_TIME_SEC} 秒)        ")
        print("=" * 40)
        
        loop = asyncio.get_running_loop()
        loop.run_in_executor(None, winsound.Beep, 1000, 800)
        
        start_ts = datetime.now()
        
        # 【修改 1】: 废弃 gather 并发，采用错位启动，降低蓝牙通讯带宽峰值
        for sensor in sensors:
            await sensor.start_measurement()
            await asyncio.sleep(0.1)  # 缓冲 100ms

        # 采集循环
        for i in range(COLLECTION_TIME_SEC, 0, -1):
            print(f"正在采集: 剩余 {i} 秒...", end='\r')
            await asyncio.sleep(1)
        print("\n时间到! 正在停止...")

        # 【修改 2】: 错位停止
        for sensor in sensors:
            await sensor.stop_measurement()
            await asyncio.sleep(0.1)

        await asyncio.sleep(1.0)

        # 5. 保存
        print("\n[4/5] 保存数据...")
        timestamp_str = start_ts.strftime("%Y%m%d_%H%M%S")

        for sensor in sensors:
            try:
                data = sensor.get_collected_data()
                if not data or len(data.get('timestamps', [])) == 0:
                    print(f"  ❌ {sensor.custom_role} 无数据")
                    continue

                filename = os.path.join(save_dir, f"{sensor.custom_role}_{timestamp_str}.csv")
                timestamps = data['timestamps']
                quats = data['quaternions']
                accs = data['accelerations']
                gyros = data['angular_velocities']

                with open(filename, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    # 【修改 3】: 表头改为保存原始时间戳 Raw_Timestamp
                    writer.writerow([
                        "Raw_Timestamp", 
                        "Acc_X(g)", "Acc_Y(g)", "Acc_Z(g)",
                        "Gyr_X(rad/s)", "Gyr_Y(rad/s)", "Gyr_Z(rad/s)",
                        "Roll(rad)", "Pitch(rad)", "Yaw(rad)",
                        "Quat_W", "Quat_X", "Quat_Y", "Quat_Z"
                    ])

                    for i in range(len(timestamps)):
                        acc_g = accs[i] / G_CONST
                        gyr_rad = gyros[i] * DEG_TO_RAD
                        r, p, y = quaternion_to_euler_rad(quats[i])
                        q = quats[i]

                        writer.writerow([
                            f"{timestamps[i]}",  # 【修改 4】: 直接写入原始时间戳数据，不进行归零计算
                            f"{acc_g[0]:.6f}", f"{acc_g[1]:.6f}", f"{acc_g[2]:.6f}",
                            f"{gyr_rad[0]:.6f}", f"{gyr_rad[1]:.6f}", f"{gyr_rad[2]:.6f}",
                            f"{r:.6f}", f"{p:.6f}", f"{y:.6f}",
                            f"{q[0]:.6f}", f"{q[1]:.6f}", f"{q[2]:.6f}", f"{q[3]:.6f}"
                        ])
                print(f"  ✅ 保存成功: {os.path.basename(filename)} (共 {len(timestamps)} 行)")
            
            except Exception as e:
                print(f"  ❌ 保存出错: {e}")

    finally:
        print("\n[5/5] 断开连接...")
        for s in sensors:
            if s.client: 
                try: await s.client.disconnect()
                except: pass

if __name__ == "__main__":
    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    asyncio.run(main())