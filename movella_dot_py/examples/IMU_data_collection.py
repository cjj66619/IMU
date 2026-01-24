import asyncio
import time
from bleak import BleakScanner, BleakClient
import sys
import os
import time
import csv
from datetime import datetime
import numpy as np

# --- 确保能找到 movella_dot_py 库 ---
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

try:
    from movella_dot_py.core.sensor import MovellaDOTSensor
    from movella_dot_py.models.data_structures import SensorConfiguration
    from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode
except ImportError:
    print("错误: 无法导入 'movella_dot_py'。请确保库文件在正确路径下。")
    sys.exit(1)

# ==========================================
# 1. 配置 4 个 IMU 的蓝牙地址
# ==========================================
TARGET_ADDRESSES = {
    # --- 左腿 ---
    "D4:22:CD:00:7F:39",
    "D4:22:CD:00:7F:30",

    # --- 右腿 ---
    "D4:22:CD:00:7F:39",
    "D4:22:CD:00:7F:3A"
}

# 采集时长 (秒)
COLLECTION_TIME_SEC =20


# ==========================================

# --- 定义物理常量 ---
G_CONST = 9.80665  # m/s^2 -> g
DEG_TO_RAD = np.pi / 180.0  # deg -> rad


# --- 四元数转欧拉角函数 ---
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


async def main():
    print("=== 双腿 (4 IMU) 数据采集程序启动 ===")
    print(f"目标传感器列表: {TARGET_ADDRESSES}")

    # 1. 扫描传感器
    print("\n正在扫描环境中的 Movella DOT 传感器 (5秒)...")
    devices = await BleakScanner.discover(timeout=5.0)

    # 筛选目标设备
    dot_devices = [d for d in devices if d.address in TARGET_ADDRESSES]

    if not dot_devices:
        print("错误: 未找到任何指定的目标传感器。请检查传感器是否开机且电量充足。")
        return

    print(f"扫描结果: 找到了 {len(dot_devices)} / {len(TARGET_ADDRESSES)} 个目标传感器:")
    for d in dot_devices:
        print(f"  - {d.name} ({d.address})")

    if len(dot_devices) < len(TARGET_ADDRESSES):
        print(f"警告: 缺少 {len(TARGET_ADDRESSES) - len(dot_devices)} 个传感器，将只连接已找到的设备。")

    # 2. 配置参数 (60Hz, 模式5)
    config = SensorConfiguration(
        output_rate=OutputRate.RATE_60,
        filter_profile=FilterProfile.GENERAL,
        payload_mode=PayloadMode.CUSTOM_MODE_5
    )

    sensors = []

    # 3. 逐个连接传感器
    print("\n开始逐个建立连接...")
    for device in dot_devices:
        sensor = MovellaDOTSensor(config)
        try:
            sensor.client = BleakClient(device.address)
            print(f"正在连接 -> {device.address} ...")

            # 连接重试机制
            connected = False
            for attempt in range(5):
                try:
                    await sensor.client.connect(timeout=15.0)
                    connected = True
                    break
                except Exception as e:
                    print(f"  第 {attempt + 1} 次连接失败: {e}，重试中...")
                    await asyncio.sleep(1.0)

            if not connected:
                print(f"  错误: 无法连接到 {device.address}，跳过此设备。")
                continue

            sensor.is_connected = True
            sensor._device_address = device.address

            # 读取基本信息
            try:
                device_info = await sensor.get_device_info()
                sensor._device_tag = device_info.device_tag
                # --- [修改] 删除了 get_battery_level 调用 ---
                print(f"  连接成功! 标签: {device_info.device_tag}")
            except Exception as e:
                print(f"  连接成功，但读取设备信息失败: {e} (不影响数据采集)")
                sensor._device_tag = "Unknown"

            # 识别 (LED闪烁)
            await sensor.identify_sensor()

            # 配置
            await sensor.configure_sensor()
            sensors.append(sensor)

        except Exception as e:
            print(f"  初始化传感器 {device.address} 时发生异常: {e}")
            # 尝试断开以防万一
            if sensor.client and sensor.client.is_connected:
                await sensor.client.disconnect()

    if not sensors:
        print("没有传感器连接成功，程序结束。")
        return

    # 4. 开始同步测量
    try:
        print(f"\n所有 {len(sensors)} 个传感器准备就绪。")
        print("正在启动同步测量...")
        await asyncio.gather(*(sensor.start_measurement() for sensor in sensors))

        print("\n" + "=" * 60)
        print(f"!!! 开始采集: 请在 {COLLECTION_TIME_SEC} 秒内完成双腿动作 !!!")
        print("!!! 建议动作: 静止3秒 -> 抬左腿 -> 抬右腿 -> 行走 !!!")
        print("=" * 60 + "\n")

        # 倒计时显示
        for i in range(COLLECTION_TIME_SEC, 0, -1):
            print(f"剩余时间: {i} 秒...", end='\r')
            await asyncio.sleep(1)
        print("\n\n时间到! 正在停止测量...")

        # 5. 停止测量
        await asyncio.gather(*(sensor.stop_measurement() for sensor in sensors))
        await asyncio.sleep(0.5)  # 等待最后的数据包

        # 6. 保存数据
        print("\n--- 开始保存数据文件 ---")
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        for sensor in sensors:
            data = sensor.get_collected_data()
            if not data or len(data['timestamps']) == 0:
                print(f"传感器 {sensor._device_address} 无数据。")
                continue

            # 生成文件名: MAC地址_时间.csv
            mac_clean = sensor._device_address.replace(':', '-')
            filename = f"{mac_clean}_{timestamp_str}.csv"

            # 提取数据
            timestamps = data['timestamps']
            quats = data['quaternions']
            accs = data['accelerations']
            gyros = data['angular_velocities']

            # 时间戳归零处理
            t0 = timestamps[0]
            timestamps_rel_ms = (timestamps - t0) / 1000.0

            try:
                with open(filename, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    # 写入表头
                    writer.writerow([
                        "Timestamp_ms",
                        "Acc_X(g)", "Acc_Y(g)", "Acc_Z(g)",
                        "Gyr_X(rad/s)", "Gyr_Y(rad/s)", "Gyr_Z(rad/s)",
                        "Roll(rad)", "Pitch(rad)", "Yaw(rad)"
                    ])

                    count = 0
                    for i in range(len(timestamps)):
                        # 单位转换
                        acc_g = accs[i] / G_CONST
                        gyr_rad = gyros[i] * DEG_TO_RAD
                        r, p, y = quaternion_to_euler_rad(quats[i])

                        writer.writerow([
                            f"{timestamps_rel_ms[i]:.2f}",
                            f"{acc_g[0]:.6f}", f"{acc_g[1]:.6f}", f"{acc_g[2]:.6f}",
                            f"{gyr_rad[0]:.6f}", f"{gyr_rad[1]:.6f}", f"{gyr_rad[2]:.6f}",
                            f"{r:.6f}", f"{p:.6f}", f"{y:.6f}"
                        ])
                        count += 1

                print(f"  -> [{sensor._device_address}] 已保存 {count} 帧数据到: {filename}")

            except Exception as e:
                print(f"  保存 {filename} 失败: {e}")

    except Exception as e:
        print(f"\n运行时错误: {e}")

    finally:
        print("\n正在断开所有连接...")
        connected = [s for s in sensors if s.is_connected]
        if connected:
            await asyncio.gather(*(s.disconnect() for s in connected))
        print("全部断开，程序结束。")


if __name__ == "__main__":
    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    asyncio.run(main())