import asyncio
import sys
import os
import csv
from datetime import datetime
import numpy as np
from bleak import BleakClient

# --- 路径兼容设置 ---
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

try:
    from movella_dot_py.core.sensor import MovellaDOTSensor
    from movella_dot_py.models.data_structures import SensorConfiguration
    from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode
except ImportError:
    print("错误: 找不到 movella_dot_py 库。")
    sys.exit(1)

# ==========================================
# 🎯 仅需修改此处地址
# ==========================================
SENSOR_MAP = {
    "D4:22:CD:00:7F:2D": "Left_Thigh",
    "D4:22:CD:00:7F:3A": "Left_Shank"
}
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

async def connect_direct(address, role, config):
    print(f"\n[{role}] 正在呼叫 {address} ...")
    print("   (采用盲连模式，请耐心等待 20秒，不要动鼠标...)")
    
    sensor = MovellaDOTSensor(config)
    sensor.custom_role = role
    
    # 关键修改：直接传入地址字符串，不传设备对象
    # timeout=25.0 给 Windows 足够的时间去内部发现设备
    sensor.client = BleakClient(address, timeout=25.0)
    
    try:
        await sensor.client.connect()
        if sensor.client.is_connected:
            print(f"   ✅ [{role}] 捕获成功!")
            await asyncio.sleep(0.5)
            print("   ⚙️ 配置传感器参数...", end=" ")
            await sensor.identify_sensor()
            await sensor.configure_sensor()
            print("完成。")
            return sensor
    except Exception as e:
        print(f"\n   ❌ [{role}] 连接失败: {e}")
        # 尝试清理
        try:
            await sensor.client.disconnect()
        except:
            pass
            
    return None

async def main():
    print("=== IMU 盲连模式 (No Scan) ===")
    print("提示：此模式不进行扫描，直接发起连接请求。")
    print("请确认：1.手机蓝牙已关 2.电脑设置中已删除设备 3.传感器闪烁中")
    
    config = SensorConfiguration(
        output_rate=OutputRate.RATE_60,
        filter_profile=FilterProfile.DYNAMIC,
        payload_mode=PayloadMode.CUSTOM_MODE_5
    )
    
    sensors = []
    
    # 1. 逐个连接
    for address, role in SENSOR_MAP.items():
        sensor = await connect_direct(address, role, config)
        if sensor:
            sensors.append(sensor)
            # 盲连模式下，连接成功后的系统冷却尤为重要
            print("   ❄️ 冷却 3 秒...")
            await asyncio.sleep(3.0)
        else:
            print("   ⚠️ 跳过此设备，尝试下一个...")

    if not sensors:
        print("\n❌ 全部失败。请检查是否被其他设备(如手机)抢占。")
        return

    # 2. 采集
    print(f"\n=== 开始采集 ({len(sensors)} 个设备) ===")
    try:
        await asyncio.gather(*(s.start_measurement() for s in sensors))
        
        for i in range(20, 0, -1):
            print(f"采集倒计时: {i} s ...", end='\r')
            await asyncio.sleep(1)
            
        print("\n停止...")
        await asyncio.gather(*(s.stop_measurement() for s in sensors))
        await asyncio.sleep(1.0)
        
        # 3. 保存
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs("data_log", exist_ok=True)
        
        for s in sensors:
            data = s.get_collected_data()
            if data and len(data['timestamps']) > 0:
                fname = f"data_log/{s.custom_role}_{ts_str}.csv"
                # ... (简化的保存逻辑，保证能跑通就行) ...
                # 这里为了代码短一点，我直接把之前的保存逻辑贴过来使用即可
                # 或者你只要看到能采集成功，保存代码都是小事
                print(f"   ✅ 数据已缓存 (帧数: {len(data['timestamps'])}) -> {fname}")
                
                # 简单写入，确保你有数据拿
                with open(fname, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(["Timestamp", "Quat_W", "Quat_X", "Quat_Y", "Quat_Z"])
                    for i in range(len(data['timestamps'])):
                         writer.writerow([data['timestamps'][i], *data['quaternions'][i]])

    except Exception as e:
        print(f"运行错误: {e}")
    finally:
        print("\n断开连接...")
        for s in sensors:
            if s.client: await s.client.disconnect()

if __name__ == "__main__":
    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    asyncio.run(main())