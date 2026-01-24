import numpy as np
import os
import math
from scipy.optimize import least_squares
import csv
import glob

# ==========================================
# 1. 配置参数
# ==========================================

# 3 IMU 配置
CALIBRATION_GROUPS = {
    "Left_Hip": {
        "Pelvis_MAC": "D4:22:CD:00:7F:39",  # 后背
        "Thigh_MAC": "D4:22:CD:00:7F:30",   # 左大腿
        "OUTPUT_FILE": "calibration_params_left_hip.npz"
    },
    "Right_Hip": {
        "Pelvis_MAC": "D4:22:CD:00:7F:39",  # 后背 (共用)
        "Thigh_MAC": "D4:22:CD:00:7F:3A",   # 右大腿
        "OUTPUT_FILE": "calibration_params_right_hip.npz"
    }
}

SENSOR_RATE = 30.0
GRAVITY = 9.80665
DEG_TO_RAD = np.pi / 180.0
RAD_TO_DEG = 180.0 / np.pi
DELTA_T = 1.0 / SENSOR_RATE


# ==========================================

def s(x): return np.sin(x)
def c(x): return np.cos(x)


def find_latest_csv_for_mac(mac_address):
    """
    根据 MAC 地址自动查找当前目录下最新的 CSV 文件
    文件名格式通常为: D4-22-CD-00-7F-2D_20231119_...csv
    """
    mac_str = mac_address.replace(":", "-")
    pattern = f"{mac_str}*.csv"

    files = glob.glob(pattern)
    if not files:
        return None

    latest_file = max(files, key=os.path.getmtime)
    return latest_file


def load_single_imu_data(filename):
    """加载单个 CSV 文件并提取 Acc 和 Gyr"""
    print(f"  -> 正在读取文件: {filename}")
    acc_list = []
    gyr_list = []
    timestamps = []

    try:
        with open(filename, 'r', newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            header = next(reader)

            try:
                ts_idx = header.index('Timestamp_ms')
                acc_x_idx = header.index('Acc_X(g)')
                gyr_x_idx = header.index('Gyr_X(rad/s)')
            except ValueError:
                print(f"    错误: 文件 {filename} 表头格式不正确。")
                return None

            for row in reader:
                try:
                    if len(row) < max(acc_x_idx, gyr_x_idx) + 3: continue

                    ts = float(row[ts_idx])
                    acc = [float(row[acc_x_idx]), float(row[acc_x_idx + 1]), float(row[acc_x_idx + 2])]
                    gyr_rad = [float(row[gyr_x_idx]), float(row[gyr_x_idx + 1]), float(row[gyr_x_idx + 2])]
                    gyr_deg = [g * RAD_TO_DEG for g in gyr_rad]

                    timestamps.append(ts)
                    acc_list.append(acc)
                    gyr_list.append(gyr_deg)
                except:
                    continue

        if not timestamps:
            print("    错误: 文件为空或无有效数据")
            return None

        if len(timestamps) > 10:
            avg_dt = (timestamps[-1] - timestamps[0]) / (len(timestamps) - 1) / 1000.0
            global DELTA_T
            if abs(avg_dt - (1 / SENSOR_RATE)) > 0.005:
                print(f"    提示: 检测到实际采样间隔约为 {avg_dt:.4f}s")

        acc_np = np.array(acc_list)
        gyr_np = np.array(gyr_list)

        gyr_dot_np = np.gradient(gyr_np, DELTA_T, axis=0)

        return np.hstack((acc_np, gyr_np, gyr_dot_np))

    except Exception as e:
        print(f"    读取失败: {e}")
        return None


# --- 代价函数 ---

def cost_func_pos(params, input_data):
    o1, o2 = params[0:3], params[3:6]

    a1 = input_data[:, 0:3] * GRAVITY
    g1 = input_data[:, 3:6] * DEG_TO_RAD
    gd1 = input_data[:, 6:9] * DEG_TO_RAD

    a2 = input_data[:, 9:12] * GRAVITY
    g2 = input_data[:, 12:15] * DEG_TO_RAD
    gd2 = input_data[:, 15:18] * DEG_TO_RAD

    term1 = np.cross(g1, np.cross(g1, o1)) + np.cross(gd1, o1)
    term2 = np.cross(g2, np.cross(g2, o2)) + np.cross(gd2, o2)

    pred_a_joint_1 = a1 - term1
    pred_a_joint_2 = a2 - term2

    return np.linalg.norm(pred_a_joint_1 - pred_a_joint_2, axis=1)


def cost_func_axis(params, input_data):
    theta1, theta2, phi1, phi2 = params
    g1 = input_data[:, 0:3]
    g2 = input_data[:, 3:6]

    j1 = np.array([c(phi1) * c(theta1), c(phi1) * s(theta1), s(phi1)])
    j2 = np.array([c(phi2) * c(theta2), c(phi2) * s(theta2), s(phi2)])

    w1_sq = np.sum(g1 ** 2, axis=1)
    w1_dot_j1 = np.dot(g1, j1)
    cross1_sq = w1_sq - w1_dot_j1 ** 2

    w2_sq = np.sum(g2 ** 2, axis=1)
    w2_dot_j2 = np.dot(g2, j2)
    cross2_sq = w2_sq - w2_dot_j2 ** 2

    cross1_sq = np.maximum(cross1_sq, 1e-9)
    cross2_sq = np.maximum(cross2_sq, 1e-9)

    return np.sqrt(cross1_sq) - np.sqrt(cross2_sq)


def run_optimization(func, x0, data, label):
    print(f"    > 开始{label}优化...")
    res = least_squares(func, x0, args=([data]), method='trf', ftol=1e-5)
    print(f"      {label}完成. Cost: {res.cost:.4f}")
    return res.x


# ==========================================
# 主逻辑
# ==========================================

def calibrate_group(group_name, config):
    print(f"\n=== 正在处理: {group_name} ===")
    mac_pelvis = config["Pelvis_MAC"]
    mac_thigh = config["Thigh_MAC"]
    out_file = config["OUTPUT_FILE"]

    # 1. 自动查找文件
    file_pelvis = find_latest_csv_for_mac(mac_pelvis)
    file_thigh = find_latest_csv_for_mac(mac_thigh)

    if not file_pelvis:
        print(f"错误: 找不到 MAC 为 {mac_pelvis} 的 CSV 文件。")
        return
    if not file_thigh:
        print(f"错误: 找不到 MAC 为 {mac_thigh} 的 CSV 文件。")
        return

    print(f"使用 Pelvis 文件: {file_pelvis}")
    print(f"使用 Thigh 文件:  {file_thigh}")

    # 2. 加载数据
    data_pelvis = load_single_imu_data(file_pelvis)
    data_thigh = load_single_imu_data(file_thigh)

    if data_pelvis is None or data_thigh is None:
        return

    # 对齐长度
    min_len = min(len(data_pelvis), len(data_thigh))
    data_pelvis = data_pelvis[:min_len]
    data_thigh = data_thigh[:min_len]
    print(f"有效数据帧数: {min_len}")

    # 3. 轴向标定 (使用角速度)
    # 传入 Gyr: data_pelvis 和 data_thigh 的 [3:6]
    axis_input = np.hstack((data_pelvis[:, 3:6], data_thigh[:, 3:6]))
    x0_axis = np.array([0.5, 0.5, 0.5, 0.5])
    params_axis = run_optimization(cost_func_axis, x0_axis, axis_input, "轴向")

    # 还原轴向量
    th1, th2, ph1, ph2 = params_axis
    j_pelvis = np.array([c(ph1) * c(th1), c(ph1) * s(th1), s(ph1)])
    j_thigh = np.array([c(ph2) * c(th2), c(ph2) * s(th2), s(ph2)])
    j_pelvis /= np.linalg.norm(j_pelvis)
    j_thigh /= np.linalg.norm(j_thigh)

    print(f"      J_Pelvis: {j_pelvis}")
    print(f"      J_Thigh:  {j_thigh}")

    # 4. 位置标定 (使用所有数据)
    pos_input = np.hstack((data_pelvis, data_thigh))
    x0_pos = np.array([0.1, 0.0, 0.0, -0.1, 0.0, 0.0])
    params_pos = run_optimization(cost_func_pos, x0_pos, pos_input, "位置")

    o_pelvis, o_thigh = params_pos[0:3], params_pos[3:6]
    print(f"      O_Pelvis (原始): {o_pelvis}")
    print(f"      O_Thigh  (原始): {o_thigh}")

    # 5. 投影修正
    dot_sum = np.dot(o_pelvis, j_pelvis) + np.dot(o_thigh, j_thigh)
    o_pelvis_final = o_pelvis - j_pelvis * dot_sum / 2.0
    o_thigh_final = o_thigh - j_thigh * dot_sum / 2.0

    print(f"      O_Pelvis (修正): {o_pelvis_final}")
    print(f"      O_Thigh  (修正): {o_thigh_final}")

    # 6. 保存
    # j_axis1 对应 Pelvis, j_axis2 对应 Thigh
    np.savez(out_file, 
             j_axis1=j_pelvis, j_axis2=j_thigh, 
             o1=o_pelvis_final, o2=o_thigh_final,
             mac1=mac_pelvis, mac2=mac_thigh)
    print(f"*** {group_name} 标定完成，已保存至 {out_file} ***")


if __name__ == "__main__":
    print("=== 3 IMU 髋关节离线标定程序 ===")
    
    # 打印当前配置
    for name, cfg in CALIBRATION_GROUPS.items():
        print(f"配置: {name}")
        print(f"  Pelvis: {cfg['Pelvis_MAC']}")
        print(f"  Thigh:  {cfg['Thigh_MAC']}")

    for group_name, config in CALIBRATION_GROUPS.items():
        try:
            calibrate_group(group_name, config)
        except Exception as e:
            print(f"处理 {group_name} 时出错: {e}")

    print("\n所有任务结束。")
