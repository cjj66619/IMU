import numpy as np
import os
import csv
import glob
import matplotlib.pyplot as plt

# ================= 配置区域 =================
LEGS_CONFIG = {
    "Left_Hip": {
        "Pelvis_MAC": "D4:22:CD:00:7F:2D",  # 骨盆 (2D)
        "Thigh_MAC":  "D4:22:CD:00:7F:30",  # 左大腿 (30)
        "CALIB_FILE": "calibration_params_left_hip.npz"
    },
    "Right_Hip": {
        "Pelvis_MAC": "D4:22:CD:00:7F:2D",  # 骨盆 (2D) - 共用
        "Thigh_MAC": "D4:22:CD:00:7F:3A", 
        "CALIB_FILE": "calibration_params_right_hip.npz"
    }
}

# 全局参数
SENSOR_RATE = 30
LAMBDA_CF = 0.02
STATIC_DURATION_SEC = 3.0
REVERSE_ANGLE_DIRECTION = False
GRAVITY = 9.80665
RAD_TO_DEG = 180.0 / np.pi
DT_ONLINE = 1.0 / SENSOR_RATE

# 自适应归零参数
ADAPTIVE_CENTERING_ALPHA = 0.01


# ================= 核心算法: 关节角度计算器 =================
class JointAngleCalculator:
    def __init__(self, calibration_file_path, sensor_rate_hz, lambda_cf):
        if not os.path.exists(calibration_file_path):
            raise FileNotFoundError(f"文件未找到: {calibration_file_path}")
        try:
            params = np.load(calibration_file_path)
            # j_axis1 对应 Pelvis, j_axis2 对应 Thigh
            self.j_axis1 = params['j_axis1']
            self.j_axis2 = params['j_axis2']
            self.o1 = params['o1']
            self.o2 = params['o2']
        except Exception as e:
            raise RuntimeError(f"加载标定参数失败: {e}")

        self.lambda_cf = lambda_cf
        self.dt = 1.0 / sensor_rate_hz
        self.gyr_angle_rad_prev = None
        self.angle_offset_rad = 0.0
        self.gyr1_bias = np.zeros(3)
        self.gyr2_bias = np.zeros(3)
        self.is_calibrated = False

        # 动态偏差追踪器
        self.dynamic_bias = 0.0
        self.centering_alpha = ADAPTIVE_CENTERING_ALPHA

    def perform_static_calibration(self, acc1, gyr1, acc2, gyr2):
        self.gyr1_bias = np.mean(gyr1, axis=0)
        self.gyr2_bias = np.mean(gyr2, axis=0)
        angles = [self._calculate_acc_angle(acc1[i], np.zeros(3), acc2[i], np.zeros(3)) for i in range(len(acc1))]

        self.angle_offset_rad = np.mean(angles)
        self.gyr_angle_rad_prev = self.angle_offset_rad

        self.dynamic_bias = 0.0
        self.is_calibrated = True

    def _calculate_acc_angle(self, acc1_g, gyr1_rads, acc2_g, gyr2_rads):
        acc1 = acc1_g * GRAVITY
        acc2 = acc2_g * GRAVITY
        r1 = acc1 - np.cross(gyr1_rads, np.cross(gyr1_rads, self.o1))
        r2 = acc2 - np.cross(gyr2_rads, np.cross(gyr2_rads, self.o2))
        p1 = r1 - np.dot(r1, self.j_axis1) * self.j_axis1
        p2 = r2 - np.dot(r2, self.j_axis2) * self.j_axis2
        n1 = np.linalg.norm(p1)
        n2 = np.linalg.norm(p2)
        if n1 < 1e-6 or n2 < 1e-6:
            return self.gyr_angle_rad_prev if self.gyr_angle_rad_prev else 0.0
        u1 = p1 / n1
        u2 = p2 / n2
        return np.arctan2(np.dot(np.cross(u1, u2), self.j_axis1), np.clip(np.dot(u1, u2), -1.0, 1.0))

    def calculate_step_rad(self, acc1_g, gyr1_rads, acc2_g, gyr2_rads):
        # acc1_g, gyr1_rads 来自 Pelvis
        # acc2_g, gyr2_rads 来自 Thigh
        if not self.is_calibrated: return 0.0
        g1 = gyr1_rads - self.gyr1_bias
        g2 = gyr2_rads - self.gyr2_bias
        acc_angle = self._calculate_acc_angle(acc1_g, g1, acc2_g, g2)
        w_rel = np.dot(g2, self.j_axis2) - np.dot(g1, self.j_axis1)
        gyr_pred = self.gyr_angle_rad_prev + w_rel * self.dt
        err = (acc_angle - gyr_pred + np.pi) % (2 * np.pi) - np.pi
        fused = gyr_pred + self.lambda_cf * err
        self.gyr_angle_rad_prev = fused

        # 1. 计算基于初始标定的相对角度
        raw_rel_rad = (fused - self.angle_offset_rad + np.pi) % (2 * np.pi) - np.pi

        # 2. 动态自适应归零
        self.dynamic_bias = (1.0 - self.centering_alpha) * self.dynamic_bias + self.centering_alpha * raw_rel_rad

        # 3. 修正后的输出
        centered_rad = raw_rel_rad - self.dynamic_bias

        return -centered_rad if REVERSE_ANGLE_DIRECTION else centered_rad


# ================= 数据加载工具 =================
def find_latest_csv(mac):
    files = glob.glob(f"{mac.replace(':', '-')}*.csv")
    return max(files, key=os.path.getmtime) if files else None


def load_data(filename):
    accs, gyrs = [], []
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            next(reader)  # 跳过表头
            for row in reader:
                try:
                    # Time, AccX, AccY, AccZ, GyrX, GyrY, GyrZ ...
                    # 假设我们只需要 Acc 和 Gyr
                    accs.append([float(x) for x in row[1:4]])
                    gyrs.append([float(x) for x in row[4:7]])
                except:
                    continue
    except:
        pass
    return np.array(accs), np.array(gyrs)


# ================= 主程序 =================
def main():
    legs_data = {}
    valid_groups = 0
    min_len = float('inf')

    print("=== 正在加载数据... ===")
    
    # 查找并加载公共的 Pelvis 数据 (Left Hip 和 Right Hip 都要用)
    # 为了避免重复加载，我们可以先加载所有涉及的 MAC 文件
    
    for name, cfg in LEGS_CONFIG.items():
        mac_pelvis = cfg["Pelvis_MAC"]
        mac_thigh = cfg["Thigh_MAC"]
        
        f_pelvis = find_latest_csv(mac_pelvis)
        f_thigh = find_latest_csv(mac_thigh)

        if f_pelvis and f_thigh:
            print(f"  [{name}] 找到文件:")
            print(f"    Pelvis: {f_pelvis}")
            print(f"    Thigh:  {f_thigh}")
            
            # 加载数据
            a_p, g_p = load_data(f_pelvis)
            a_t, g_t = load_data(f_thigh)
            
            n = min(len(a_p), len(a_t))

            if n == 0:
                print(f"  警告: {name} 数据为空")
                continue

            legs_data[name] = {
                "c": JointAngleCalculator(cfg["CALIB_FILE"], SENSOR_RATE, LAMBDA_CF),
                "a_pelvis": a_p[:n], "g_pelvis": g_p[:n],
                "a_thigh":  a_t[:n], "g_thigh":  g_t[:n]
            }
            min_len = min(min_len, n)
            valid_groups += 1
            print(f"  {name}: 数据加载成功 ({n} 帧)")
        else:
            print(f"  警告: 未找到 {name} 的完整数据文件 (MAC: {mac_pelvis}, {mac_thigh})")

    if valid_groups == 0:
        print("错误: 没有有效的数据组")
        return

    # 对齐数据长度
    n_static = int(STATIC_DURATION_SEC * SENSOR_RATE)

    results = {
        "time": [],
        "left_hip_angle": [],
        "right_hip_angle": []
    }

    # 1. 静态标定
    print("=== 正在进行静态标定 (前3秒) ... ===")
    for name in legs_data:
        d = legs_data[name]
        calib_len = min(n_static, len(d["a_pelvis"]))
        # 调用标定: 传入 Pelvis(imu1), Thigh(imu2)
        d["c"].perform_static_calibration(
            d["a_pelvis"][:calib_len], d["g_pelvis"][:calib_len],
            d["a_thigh"][:calib_len],  d["g_thigh"][:calib_len]
        )

    # 2. 逐帧计算角度
    print("=== 正在计算髋关节角度... ===")
    for i in range(min_len):
        t = i * DT_ONLINE

        # 左髋
        d_left = legs_data.get("Left_Hip")
        ang_L = 0.0
        if d_left:
            ang_L = d_left["c"].calculate_step_rad(
                d_left["a_pelvis"][i], d_left["g_pelvis"][i],
                d_left["a_thigh"][i],  d_left["g_thigh"][i]
            )

        # 右髋
        d_right = legs_data.get("Right_Hip")
        ang_R = 0.0
        if d_right:
            ang_R = d_right["c"].calculate_step_rad(
                d_right["a_pelvis"][i], d_right["g_pelvis"][i],
                d_right["a_thigh"][i],  d_right["g_thigh"][i]
            )

        results["time"].append(t)
        results["left_hip_angle"].append(ang_L * RAD_TO_DEG)
        results["right_hip_angle"].append(ang_R * RAD_TO_DEG)

    # 3. 保存 CSV
    csv_filename = "hip_angle_results.csv"
    print(f"=== 正在保存结果到 {csv_filename} ... ===")
    try:
        with open(csv_filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(["Time(s)", "Left_Hip_Angle(deg)", "Right_Hip_Angle(deg)"])
            for t, l, r in zip(results["time"], results["left_hip_angle"], results["right_hip_angle"]):
                writer.writerow([f"{t:.4f}", f"{l:.4f}", f"{r:.4f}"])
        print("保存成功!")
    except Exception as e:
        print(f"保存 CSV 失败: {e}")

    # 4. 绘图
    print("=== 正在绘图... ===")
    try:
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
        plt.rcParams['axes.unicode_minus'] = False
    except:
        pass

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    # 子图1: 左髋
    ax1.set_title("左髋关节角度 (Left Hip Angle)")
    ax1.plot(results["time"], results["left_hip_angle"], 'b-', label="Left Hip")
    ax1.axhline(0, color='gray', linestyle='--', linewidth=1)
    ax1.set_ylabel("角度 (deg)")
    ax1.legend(loc="upper right")
    ax1.grid(True)

    # 子图2: 右髋
    ax2.set_title("右髋关节角度 (Right Hip Angle)")
    ax2.plot(results["time"], results["right_hip_angle"], 'r-', label="Right Hip")
    ax2.axhline(0, color='gray', linestyle='--', linewidth=1)
    ax2.set_ylabel("角度 (deg)")
    ax2.set_xlabel("时间 (s)")
    ax2.legend(loc="upper right")
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
