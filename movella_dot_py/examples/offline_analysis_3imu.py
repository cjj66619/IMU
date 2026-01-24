import numpy as np
import os
import csv
import glob
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

# ================= 配置区域 =================
LEGS_CONFIG = {
    "Left_Hip": {
        "Pelvis_MAC": "D4:22:CD:00:7F:39",  # 骨盆 (2D)
        "Thigh_MAC":  "D4:22:CD:00:7F:30",  # 左大腿 (30)
        "CALIB_FILE": "calibration_params_left_hip.npz"
    },
    "Right_Hip": {
        "Pelvis_MAC": "D4:22:CD:00:7F:39",  # 骨盆 (2D) - 共用
        "Thigh_MAC":  "D4:22:CD:00:7F:3A",  # 右大腿 (39)
        "CALIB_FILE": "calibration_params_right_hip.npz"
    }
}

# 全局参数
SENSOR_RATE = 30
LAMBDA_CF = 0.02
STATIC_DURATION_SEC = 3.0
REVERSE_ANGLE_DIRECTION = False  # [修改] 使用3IMU配置时，方向可能需要根据实际情况调整，之前 verification 脚本里改成了 False
GRAVITY = 9.80665
RAD_TO_DEG = 180.0 / np.pi
DT_ONLINE = 1.0 / SENSOR_RATE

# 新增：自适应归零参数
ADAPTIVE_CENTERING_ALPHA = 0.01

# [修改] 静止检测阈值与滤波参数
# 提高一点阈值，并要求连续多帧满足才算静止
STILLNESS_THRESHOLD = 0.25  # rad/s, 约 14 deg/s
STATIC_FRAME_COUNT = 6  # 连续 6 帧 (约0.2秒) 低于阈值才确认为静止


# ================= 1. 核心算法: Tricomi 自适应振荡器 =================
class TricomiAdaptiveOscillator:
    def __init__(self, dt, num_harmonics=3, init_freq=1.0):
        self.dt = dt
        self.N = num_harmonics
        self.nu_phi = 20.0
        self.nu_omega = 5.0
        self.eta = 5.0

        # [修改] 保存初始频率，用于Reset
        self.init_omega = init_freq * 2 * np.pi
        self.omega = self.init_omega

        self.alpha = np.zeros(self.N + 1)
        self.alpha[1] = 0.1
        self.phi = np.zeros(self.N + 1)

        # 记录上一次的有效输出
        self.last_estimated = 0.0
        self.last_phase = 0.0
        self.last_omega = self.omega

    def reset_state(self):
        """[新增] 强制重置算法状态到初始值"""
        # 1. 振幅参数归位 (保留微小初值以便下次启动)
        self.alpha = np.zeros(self.N + 1)
        self.alpha[1] = 0.1

        # 2. 相位归零 (代表站立/中立位)
        self.phi = np.zeros(self.N + 1)

        # 3. 频率重置为标准初始频率 (消除之前的漂移)
        self.omega = self.init_omega

        # 4. 缓存清除
        self.last_estimated = 0.0
        self.last_phase = 0.0
        self.last_omega = self.init_omega

    def update(self, signal_val):
        harmonic_sum = 0.0
        for n in range(1, self.N + 1):
            harmonic_sum += self.alpha[n] * np.sin(self.phi[n])
        self.estimated_signal = self.alpha[0] + harmonic_sum

        F_t = signal_val - self.estimated_signal
        sum_alpha = np.sum(self.alpha[1:])

        # [关键保护] 防止分母过小导致系统发散
        if sum_alpha < 1e-2:
            sum_alpha = 1e-2
            F_t *= 0.1

        d_phi = np.zeros(self.N + 1)
        for n in range(1, self.N + 1):
            coupling = self.nu_phi * (F_t / sum_alpha) * np.cos(self.phi[n])
            d_phi[n] = n * self.omega + coupling

        d_omega = self.nu_omega * (F_t / sum_alpha) * np.cos(self.phi[1])

        d_alpha = np.zeros(self.N + 1)
        d_alpha[0] = self.eta * F_t
        for n in range(1, self.N + 1):
            d_alpha[n] = self.eta * F_t * np.sin(self.phi[n])

        self.phi += d_phi * self.dt
        self.omega += d_omega * self.dt
        self.alpha += d_alpha * self.dt

        # 频率限幅保护
        self.omega = np.clip(self.omega, 0.3 * 2 * np.pi, 2.0 * 2 * np.pi)

        # 相位对齐逻辑
        base_shift = -np.pi / 2
        if self.alpha[1] < 0:
            base_shift += np.pi

        self.gait_phase = (self.phi[1] + base_shift) % (2 * np.pi)

        # 更新缓存
        self.last_estimated = self.estimated_signal
        self.last_phase = self.gait_phase
        self.last_omega = self.omega

        return self.estimated_signal, self.gait_phase, self.omega / (2 * np.pi), self.omega


# ================= 2. 双AO 步态识别器 =================
class DualGaitRecognizer:
    def __init__(self, dt):
        self.dt = dt
        self.ao_left = TricomiAdaptiveOscillator(dt, init_freq=0.7)
        self.ao_right = TricomiAdaptiveOscillator(dt, init_freq=0.7)

    def process(self, angle_l_rad, angle_r_rad, is_static_l, is_static_r):
        # [修改逻辑] 静止时重置AO参数，运动时重新计算

        if is_static_l:
            # 1. 彻底重置左腿算法状态
            self.ao_left.reset_state()
            # 2. 输出强制归零
            hat_l = 0.0
            phi_l = 0.0
            omega_l = self.ao_left.init_omega
            freq_l_hz = omega_l / (2 * np.pi)
        else:
            hat_l, phi_l, freq_l_hz, omega_l = self.ao_left.update(angle_l_rad)

        if is_static_r:
            # 1. 彻底重置右腿算法状态
            self.ao_right.reset_state()
            # 2. 输出强制归零
            hat_r = 0.0
            phi_r = 0.0
            omega_r = self.ao_right.init_omega
            freq_r_hz = omega_r / (2 * np.pi)
        else:
            hat_r, phi_r, freq_r_hz, omega_r = self.ao_right.update(angle_r_rad)

        return {
            "hat_l": hat_l, "phi_l": phi_l, "freq_l": freq_l_hz, "omega_l": omega_l,
            "hat_r": hat_r, "phi_r": phi_r, "freq_r": freq_r_hz, "omega_r": omega_r
        }


# ================= 3. 高级轨迹生成器 =================
def generate_bell_curve(phase, center_phase, width=0.5, amplitude=1.0):
    diff = np.abs(phase - center_phase)
    diff = np.minimum(diff, 2 * np.pi - diff)
    val = amplitude * np.exp(- (diff ** 2) / (2 * width ** 2))
    if val < 0.05: val = 0.0
    return val


# ================= 4. 基础工具 =================
class JointAngleCalculator:
    def __init__(self, calibration_file_path, sensor_rate_hz, lambda_cf):
        if not os.path.exists(calibration_file_path): raise FileNotFoundError(f"文件未找到: {calibration_file_path}")
        try:
            params = np.load(calibration_file_path)
            # 3IMU 配置下:
            # j_axis1/o1 => Pelvis
            # j_axis2/o2 => Thigh
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

    # [新增] 重置状态函数
    def reset_state(self):
        """
        当检测到静止时调用。
        功能：
        1. 将 gyr_angle_rad_prev 设为 None，强制下一次计算直接使用加速度计的绝对角度，
           消除之前运动累积的积分漂移。
        2. 重置 dynamic_bias，防止静止时的姿态被错误地当成Bias学习，
           确保下次重新运动时从干净的状态开始。
        """
        self.gyr_angle_rad_prev = None
        self.dynamic_bias = 0.0

    def _calculate_acc_angle(self, acc1_g, gyr1_rads, acc2_g, gyr2_rads):
        acc1 = acc1_g * GRAVITY
        acc2 = acc2_g * GRAVITY
        r1 = acc1 - np.cross(gyr1_rads, np.cross(gyr1_rads, self.o1))
        r2 = acc2 - np.cross(gyr2_rads, np.cross(gyr2_rads, self.o2))
        p1 = r1 - np.dot(r1, self.j_axis1) * self.j_axis1
        p2 = r2 - np.dot(r2, self.j_axis2) * self.j_axis2
        n1 = np.linalg.norm(p1)
        n2 = np.linalg.norm(p2)
        if n1 < 1e-6 or n2 < 1e-6: return self.gyr_angle_rad_prev if self.gyr_angle_rad_prev else 0.0
        u1 = p1 / n1
        u2 = p2 / n2
        return np.arctan2(np.dot(np.cross(u1, u2), self.j_axis1), np.clip(np.dot(u1, u2), -1.0, 1.0))

    def calculate_step_rad(self, acc1_g, gyr1_rads, acc2_g, gyr2_rads):
        if not self.is_calibrated: return 0.0
        g1 = gyr1_rads - self.gyr1_bias
        g2 = gyr2_rads - self.gyr2_bias
        acc_angle = self._calculate_acc_angle(acc1_g, g1, acc2_g, g2)

        # 如果是Reset后的第一帧，直接使用加速度计角度初始化
        if self.gyr_angle_rad_prev is None:
            self.gyr_angle_rad_prev = acc_angle

        w_rel = np.dot(g2, self.j_axis2) - np.dot(g1, self.j_axis1)
        gyr_pred = self.gyr_angle_rad_prev + w_rel * self.dt
        err = (acc_angle - gyr_pred + np.pi) % (2 * np.pi) - np.pi
        fused = gyr_pred + self.lambda_cf * err
        self.gyr_angle_rad_prev = fused

        raw_rel_rad = (fused - self.angle_offset_rad + np.pi) % (2 * np.pi) - np.pi
        self.dynamic_bias = (1.0 - self.centering_alpha) * self.dynamic_bias + self.centering_alpha * raw_rel_rad
        centered_rad = raw_rel_rad - self.dynamic_bias
        return -centered_rad if REVERSE_ANGLE_DIRECTION else centered_rad


def find_latest_csv(mac):
    files = glob.glob(f"{mac.replace(':', '-')}*.csv")
    return max(files, key=os.path.getmtime) if files else None


def load_data(filename):
    accs, gyrs = [], []
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                try:
                    accs.append([float(x) for x in row[1:4]])
                    gyrs.append([float(x) for x in row[4:7]])
                except:
                    continue
    except:
        pass
    return np.array(accs), np.array(gyrs)


# [修改] 辅助函数：计算是否静止 (仅做单帧判断)
# 这里的 threshold 可以稍微放松，因为髋关节有时候小幅度晃动也算静止
def check_leg_static_instant(gyro1, gyro2, threshold=STILLNESS_THRESHOLD):
    mag1 = np.linalg.norm(gyro1)
    mag2 = np.linalg.norm(gyro2)
    return (mag1 < threshold) and (mag2 < threshold)


# ================= 6. 主程序 =================
def main():
    legs_data = {}
    valid_groups = 0
    min_len = float('inf')

    print("=== 正在加载数据... ===")
    
    # [修改] 加载逻辑适配 3IMU
    for name, cfg in LEGS_CONFIG.items():
        # find_latest_csv 是通用的，只要 MAC 对即可
        f_pelvis = find_latest_csv(cfg["Pelvis_MAC"])
        f_thigh = find_latest_csv(cfg["Thigh_MAC"])

        if f_pelvis and f_thigh:
            a_p, g_p = load_data(f_pelvis)
            a_t, g_t = load_data(f_thigh)
            n = min(len(a_p), len(a_t))

            # 用 a_pelvis/g_pelvis 对应算法里的 1 (Reference)
            # 用 a_thigh/g_thigh 对应算法里的 2 (Target)
            legs_data[name] = {
                "c": JointAngleCalculator(cfg["CALIB_FILE"], SENSOR_RATE, LAMBDA_CF),
                "a1": a_p[:n], "g1": g_p[:n], 
                "a2": a_t[:n], "g2": g_t[:n]
            }
            min_len = min(min_len, n)
            valid_groups += 1
            print(f"  {name}: OK ({n} frames)")
        else:
            print(f"  警告: 未找到 {name} 的完整数据文件 (MAC: {cfg['Pelvis_MAC']}, {cfg['Thigh_MAC']})")

    if valid_groups < 1: 
        return print("没有足够的数据进行分析!")

    n_static = int(STATIC_DURATION_SEC * SENSOR_RATE)
    recognizer = DualGaitRecognizer(DT_ONLINE)

    res = {
        "t": [],
        "L_raw": [], "L_fit": [],
        "R_raw": [], "R_fit": [],
        "phi_l": [], "phi_r": [],
        "motor_cmd_L": [],
        "motor_cmd_R": [],
        "is_static_L": [],
        "is_static_R": []
    }

    # 静态标定
    for name in legs_data:
        d = legs_data[name]
        d["c"].perform_static_calibration(d["a1"][:n_static], d["g1"][:n_static], d["a2"][:n_static], d["g2"][:n_static])

    # [新增] 连续静止计数器
    static_counter_L = 0
    static_counter_R = 0
    # 状态标志：只有计数器超过阈值，is_really_static 才会变成 True
    is_really_static_L = False
    is_really_static_R = False

    print("=== 正在进行双AO步态识别 (此版本为 3-IMU 髋关节分析) ... ===")
    
    for i in range(min_len):
        if i < n_static: continue

        # 安全获取数据，如果只有单腿数据也能跑
        dL = legs_data.get("Left_Hip")
        dR = legs_data.get("Right_Hip")

        # --- 静止检测逻辑 ---
        # 左腿
        if dL:
            if check_leg_static_instant(dL["g1"][i], dL["g2"][i]):
                static_counter_L += 1
                if static_counter_L >= STATIC_FRAME_COUNT:
                    is_really_static_L = True
            else:
                static_counter_L = 0
                is_really_static_L = False
        
        # 右腿
        if dR:
            if check_leg_static_instant(dR["g1"][i], dR["g2"][i]):
                static_counter_R += 1
                if static_counter_R >= STATIC_FRAME_COUNT:
                    is_really_static_R = True
            else:
                static_counter_R = 0
                is_really_static_R = False

        # --- 角度计算逻辑 ---

        # 左腿
        ang_L = 0.0
        if dL:
            if is_really_static_L:
                dL["c"].reset_state()
                ang_L = 0.0
            else:
                ang_L = dL["c"].calculate_step_rad(dL["a1"][i], dL["g1"][i], dL["a2"][i], dL["g2"][i])

        # 右腿
        ang_R = 0.0
        if dR:
            if is_really_static_R:
                dR["c"].reset_state()
                ang_R = 0.0
            else:
                ang_R = dR["c"].calculate_step_rad(dR["a1"][i], dR["g1"][i], dR["a2"][i], dR["g2"][i])

        # --- 识别器逻辑 ---
        out = recognizer.process(ang_L, ang_R, is_really_static_L, is_really_static_R)

        # 4. 生成控制指令
        cmd_l = generate_bell_curve(out["phi_l"], 0, 0.8, 1.0) if not is_really_static_L else 0.0
        cmd_r = generate_bell_curve(out["phi_r"], 0, 0.8, 1.0) if not is_really_static_R else 0.0

        res["t"].append(i * DT_ONLINE)

        res["L_raw"].append(ang_L * RAD_TO_DEG)
        res["L_fit"].append(out["hat_l"] * RAD_TO_DEG)
        res["R_raw"].append(ang_R * RAD_TO_DEG)
        res["R_fit"].append(out["hat_r"] * RAD_TO_DEG)

        res["phi_l"].append(np.degrees(out["phi_l"]))
        res["phi_r"].append(np.degrees(out["phi_r"]))
        res["motor_cmd_L"].append(cmd_l)
        res["motor_cmd_R"].append(cmd_r)
        res["is_static_L"].append(1 if is_really_static_L else 0)

    # ================= 绘图 =================
    try:
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
        plt.rcParams['axes.unicode_minus'] = False
    except:
        pass

    np_t = np.array(res["t"])

    fig, axes = plt.subplots(3, 1, figsize=(10, 14), sharex=True)
    ax1, ax2, ax3 = axes

    # 子图1: 角度 - [修改标题]
    ax1.set_title("髋关节角度跟踪 (Left/Right Hip Angle)")
    ax1.axhline(0, color='gray', linestyle=':', linewidth=1)
    ax1.plot(np_t, res["L_raw"], 'c-', alpha=0.5, label="左髋实测")
    ax1.plot(np_t, res["L_fit"], 'b--', linewidth=1.5, label="左髋AO")
    ax1.plot(np_t, res["R_raw"], 'm-', alpha=0.5, label="右髋实测")
    ax1.plot(np_t, res["R_fit"], 'r--', linewidth=1.5, label="右髋AO")

    # 标记静止区域
    ax1.fill_between(np_t, -10, 10, where=np.array(res["is_static_L"]) == 1,
                     color='gray', alpha=0.2, label="静止(左)")

    ax1.set_ylabel("角度 (deg)")
    ax1.legend(loc="upper right", ncol=3)
    ax1.grid(True)

    # 子图2 - [修改标题]
    ax2.set_title("独立双电机控制 (基于髋关节相位)")
    ax2.plot(np_t, res["motor_cmd_L"], 'b-', linewidth=2, label="左电机")
    ax2.plot(np_t, res["motor_cmd_R"], 'r-', linewidth=2, label="右电机")
    ax2.set_ylabel("指令")
    ax2.grid(True)

    # 子图3
    ax3.set_title("相位")
    ax3.plot(np_t, res["phi_l"], 'b-', linewidth=1, label="左髋")
    ax3.plot(np_t, res["phi_r"], 'r-', linewidth=1, label="右髋")
    ax3.set_ylabel("相位 (deg)")
    ax3.set_xlabel("时间 (s)")
    ax3.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
