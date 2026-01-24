import asyncio
import sys
import os
import time
import numpy as np
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from bleak import BleakScanner, BleakClient

# --- 路径设置 ---
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from movella_dot_py.core.sensor import MovellaDOTSensor
    from movella_dot_py.core.parser import PayloadParser
    from movella_dot_py.models.data_structures import SensorConfiguration
    from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode
except ImportError:
    pass

# ================= 1. 配置参数 =================
SENSOR_RATE = 30
DT_ONLINE = 1.0 / SENSOR_RATE
GRAVITY = 9.80665
DEG_TO_RAD = np.pi / 180.0
RAD_TO_DEG = 180.0 / np.pi
LAMBDA_CF = 0.02
STATIC_DURATION_SEC = 3.0
ADAPTIVE_CENTERING_ALPHA = 0.01

# 绘图窗口宽度
PLOT_WINDOW = 300

# --- IMU 配置 ---
PELVIS_MAC = "D4:22:CD:00:7F:2D"
LEFT_THIGH_MAC = "D4:22:CD:00:7F:30"
RIGHT_THIGH_MAC = "D4:22:CD:00:7F:3A"

LEFT_HIP_CONFIG = {
    "name": "Left_Hip",
    "Pelvis": PELVIS_MAC, "Thigh": LEFT_THIGH_MAC,
    "CALIB_FILE": "calibration_params_left_hip.npz", "REVERSE": False
}
RIGHT_HIP_CONFIG = {
    "name": "Right_Hip",
    "Pelvis": PELVIS_MAC, "Thigh": RIGHT_THIGH_MAC,
    "CALIB_FILE": "calibration_params_right_hip.npz", "REVERSE": False
}

TARGET_ADDRESSES = {PELVIS_MAC, LEFT_THIGH_MAC, RIGHT_THIGH_MAC}


# ================= 2. 共享数据容器 (完全解耦) =================
class LegBuffer:
    def __init__(self, max_len=PLOT_WINDOW):
        self.t = deque(maxlen=max_len)
        self.raw_ang = deque(maxlen=max_len)
        self.est_ang = deque(maxlen=max_len)
        self.phase = deque(maxlen=max_len)

class PlotData:
    def __init__(self):
        self.lock = threading.Lock()
        self.start_time = time.time()
        self.left = LegBuffer()
        self.right = LegBuffer()

    def add(self, leg_name, raw, est, phase):
        with self.lock:
            now = time.time() - self.start_time
            if leg_name == "Left_Hip":
                buf = self.left
            else:
                buf = self.right
            
            buf.t.append(now)
            buf.raw_ang.append(raw)
            buf.est_ang.append(est)
            buf.phase.append(phase)

PLOT_DATA = PlotData()


# ================= 3. 算法类 =================
class TricomiAdaptiveOscillator:
    def __init__(self, dt, num_harmonics=3, init_freq=1.0):
        self.dt = dt
        self.N = num_harmonics
        self.nu_phi = 20.0
        self.nu_omega = 5.0
        self.eta = 5.0
        self.omega = init_freq * 2 * np.pi
        self.alpha = np.zeros(self.N + 1)
        self.alpha[1] = 0.1
        self.phi = np.zeros(self.N + 1)
        self.estimated_signal = 0.0

    def update(self, signal_val):
        harmonic_sum = sum(self.alpha[n] * np.sin(self.phi[n]) for n in range(1, self.N + 1))
        self.estimated_signal = self.alpha[0] + harmonic_sum
        F_t = signal_val - self.estimated_signal
        sum_alpha = np.sum(self.alpha[1:])
        if sum_alpha < 1e-6: sum_alpha = 1e-6

        d_phi = np.zeros(self.N + 1)
        for n in range(1, self.N + 1):
            coupling = self.nu_phi * (F_t / sum_alpha) * np.cos(self.phi[n])
            d_phi[n] = n * self.omega + coupling
        d_omega = self.nu_omega * (F_t / sum_alpha) * np.cos(self.phi[1])
        d_alpha = np.zeros(self.N + 1)
        d_alpha[0] = self.eta * F_t
        for n in range(1, self.N + 1): d_alpha[n] = self.eta * F_t * np.sin(self.phi[n])

        self.phi += d_phi * self.dt
        self.omega += d_omega * self.dt
        self.alpha += d_alpha * self.dt

        # Base shift logic
        base_shift = -np.pi / 2
        if self.alpha[1] < 0: base_shift += np.pi
        gait_phase = (self.phi[1] + base_shift) % (2 * np.pi)
        
        return self.estimated_signal, gait_phase

class JointAngleCalculator:
    def __init__(self, calibration_file_path, sensor_rate_hz, lambda_cf, reverse_sign):
        self.reverse_sign = reverse_sign
        if not os.path.exists(calibration_file_path):
            self.j_axis1 = np.array([0, 0, 1.0])
            self.j_axis2 = np.array([0, 0, 1.0])
            self.o1 = np.zeros(3)
            self.o2 = np.zeros(3)
        else:
            try:
                p = np.load(calibration_file_path)
                self.j_axis1 = p['j_axis1']
                self.j_axis2 = p['j_axis2']
                self.o1 = p['o1']
                self.o2 = p['o2']
            except:
                pass # Fail silently or handle error

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
        angles = [self._calc_acc_angle(acc1[i], np.zeros(3), acc2[i], np.zeros(3)) for i in range(len(acc1))]
        self.angle_offset_rad = np.mean(angles)
        self.gyr_angle_rad_prev = self.angle_offset_rad
        self.dynamic_bias = 0.0
        self.is_calibrated = True

    def _calc_acc_angle(self, a1_g, g1, a2_g, g2):
        a1 = a1_g * GRAVITY
        a2 = a2_g * GRAVITY
        r1 = a1 - np.cross(g1, np.cross(g1, self.o1))
        r2 = a2 - np.cross(g2, np.cross(g2, self.o2))
        p1 = r1 - np.dot(r1, self.j_axis1) * self.j_axis1
        p2 = r2 - np.dot(r2, self.j_axis2) * self.j_axis2
        n1, n2 = np.linalg.norm(p1), np.linalg.norm(p2)
        if n1 < 1e-6 or n2 < 1e-6: return self.gyr_angle_rad_prev or 0.0
        u1, u2 = p1 / n1, p2 / n2
        return np.arctan2(np.dot(np.cross(u1, u2), self.j_axis1), np.clip(np.dot(u1, u2), -1.0, 1.0))

    def calculate_step(self, a1, g1, a2, g2):
        if not self.is_calibrated: return 0.0
        g1 -= self.gyr1_bias
        g2 -= self.gyr2_bias
        acc_angle = self._calc_acc_angle(a1, g1, a2, g2)
        w_rel = np.dot(g2, self.j_axis2) - np.dot(g1, self.j_axis1)
        gyr_pred = self.gyr_angle_rad_prev + w_rel * self.dt
        err = (acc_angle - gyr_pred + np.pi) % (2 * np.pi) - np.pi
        fused = gyr_pred + self.lambda_cf * err
        self.gyr_angle_rad_prev = fused
        
        raw_rel_rad = (fused - self.angle_offset_rad + np.pi) % (2 * np.pi) - np.pi
        self.dynamic_bias = (1.0 - self.centering_alpha) * self.dynamic_bias + self.centering_alpha * raw_rel_rad
        
        centered_rad = raw_rel_rad - self.dynamic_bias
        rel_deg = centered_rad * RAD_TO_DEG
        rel_deg = (rel_deg + 180) % 360 - 180
        
        if self.reverse_sign: rel_deg = -rel_deg
        
        return rel_deg


# ================= 4. BLE 线程逻辑 =================

DATA_BUFFER = {PELVIS_MAC: None, LEFT_THIGH_MAC: None, RIGHT_THIGH_MAC: None}

def handle_notification_queue(sender, data, sensor_address, parser, queues):
    try:
        parsed = parser.parse(data)
        if parsed.acceleration and parsed.angular_velocity:
            now = time.time()
            acc = parsed.acceleration.to_numpy()
            gyr = parsed.angular_velocity.to_numpy()
            DATA_BUFFER[sensor_address] = (acc, gyr, now)
            
            if sensor_address == PELVIS_MAC: return

            pelvis_data = DATA_BUFFER[PELVIS_MAC]
            if pelvis_data:
                p_acc, p_gyr, p_ts = pelvis_data
                # [修改] 放宽时间窗口到 0.15s，防止丢包太严重
                if (now - p_ts) < 0.15:
                    pair_data = ((p_acc, p_gyr), (acc, gyr))
                    target_queue = queues.get("Left_Hip") if sensor_address == LEFT_THIGH_MAC else queues.get("Right_Hip")
                    if target_queue:
                        if target_queue.full(): 
                            try: target_queue.get_nowait()
                            except: pass
                        target_queue.put_nowait(pair_data)
    except: 
        pass

async def process_limb_loop(limb_name: str, queue: asyncio.Queue, calculator: JointAngleCalculator):
    ao = TricomiAdaptiveOscillator(dt=DT_ONLINE, init_freq=0.8)
    
    # buffers for calibration
    c_acc1, c_gyr1, c_acc2, c_gyr2 = [], [], [], []
    calib_limit = int(STATIC_DURATION_SEC * SENSOR_RATE)
    
    print(f"[{limb_name}] 计算任务就绪")
    
    while True:
        try:
            (a1_raw, g1_raw), (a2_raw, g2_raw) = await queue.get()
            
            a1, a2 = a1_raw / GRAVITY, a2_raw / GRAVITY
            g1, g2 = g1_raw * DEG_TO_RAD, g2_raw * DEG_TO_RAD

            if not calculator.is_calibrated:
                c_acc1.append(a1); c_gyr1.append(g1)
                c_acc2.append(a2); c_gyr2.append(g2)
                
                if len(c_acc1) >= calib_limit:
                    calculator.perform_static_calibration(
                        np.array(c_acc1), np.array(c_gyr1), 
                        np.array(c_acc2), np.array(c_gyr2)
                    )
                    print(f"✅ [{limb_name}] 标定完成")
                    ao.phi[1] = 0.0 # 重置相位
                
                queue.task_done()
                continue

            # 实时计算
            angle_deg = calculator.calculate_step(a1, g1, a2, g2)
            est_rad, phase_rad = ao.update(angle_deg * DEG_TO_RAD)
            
            # 存入绘图数据
            PLOT_DATA.add(limb_name, angle_deg, est_rad * RAD_TO_DEG, phase_rad)
            queue.task_done()

        except asyncio.CancelledError:
            break
        except Exception:
            await asyncio.sleep(0.01)

async def ble_main():
    print("=== 正在连接传感器... ===")
    
    # 初始化计算器
    try:
        calc_l = JointAngleCalculator(LEFT_HIP_CONFIG["CALIB_FILE"], SENSOR_RATE, LAMBDA_CF, LEFT_HIP_CONFIG["REVERSE"])
        calc_r = JointAngleCalculator(RIGHT_HIP_CONFIG["CALIB_FILE"], SENSOR_RATE, LAMBDA_CF, RIGHT_HIP_CONFIG["REVERSE"])
    except Exception as e:
        print(f"计算器初始化失败: {e}")
        return

    # 扫描
    devices = await BleakScanner.discover(timeout=5.0)
    targets = [d for d in devices if d.address in TARGET_ADDRESSES]
    if len(targets) == 0:
        print("❌ 未找到传感器")
        # 这里不 return，允许仿真/调试，或者部分连接
    
    if len(targets) < 3:
        print(f"⚠️ 警告: 只连上了 {len(targets)}/3 个传感器")

    config = SensorConfiguration(
        output_rate=OutputRate(SENSOR_RATE), 
        filter_profile=FilterProfile.DYNAMIC, 
        payload_mode=PayloadMode.CUSTOM_MODE_5
    )
    
    queues = {
        "Left_Hip": asyncio.Queue(maxsize=1), 
        "Right_Hip": asyncio.Queue(maxsize=1)
    }
    
    parsers = {}
    sensors = []

    for dev in targets:
        try:
            s = MovellaDOTSensor(config)
            s.client = BleakClient(dev.address)
            await s.client.connect(timeout=10.0)
            await s.configure_sensor()
            parsers[dev.address] = PayloadParser(config.payload_mode)
            
            # 闭包绑定地址
            def make_handler(addr, parser_inst):
                 return lambda s, d: handle_notification_queue(s, d, addr, parser_inst, queues)
            
            s.notification_handler = make_handler(dev.address, parsers[dev.address])
            sensors.append(s)
            print(f"  -> {dev.address} Ready")
        except:
            print(f"  -> {dev.address} Failed")

    # 启动计算消费者
    task_l = asyncio.create_task(process_limb_loop("Left_Hip", queues["Left_Hip"], calc_l))
    task_r = asyncio.create_task(process_limb_loop("Right_Hip", queues["Right_Hip"], calc_r))

    print("\n=== 全部就绪，请保持静止 3 秒标定 ===")
    if sensors: await asyncio.gather(*(s.start_measurement() for s in sensors))

    try:
        await asyncio.Event().wait()
    except asyncio.CancelledError:
        print("BLE Loop Stopping...")
    finally:
        for s in sensors:
            if s.is_connected: 
                try: await s.stop_measurement(); await s.disconnect()
                except: pass

def run_ble_loop():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(ble_main())
    except KeyboardInterrupt:
        pass

# ================= 5. Matplotlib 主线程 =================

def main_plot():
    t = threading.Thread(target=run_ble_loop, daemon=True)
    t.start()
    
    plt.style.use('dark_background')
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    ax_l_ang, ax_l_ph = axes[0, 0], axes[0, 1]
    ax_r_ang, ax_r_ph = axes[1, 0], axes[1, 1]
    
    # Lines
    ln_l_raw, = ax_l_ang.plot([], [], 'c-', alpha=0.6, label='Raw')
    ln_l_est, = ax_l_ang.plot([], [], 'w--', lw=2, label='AO Est')
    ln_l_ph, = ax_l_ph.plot([], [], 'y-', lw=2, label='Phase')
    
    ln_r_raw, = ax_r_ang.plot([], [], 'm-', alpha=0.6, label='Raw')
    ln_r_est, = ax_r_ang.plot([], [], 'w--', lw=2, label='AO Est')
    ln_r_ph, = ax_r_ph.plot([], [], 'g-', lw=2, label='Phase')

    # Formatting
    for ax in [ax_l_ang, ax_r_ang]:
        ax.set_ylim(-60, 60)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
    
    for ax in [ax_l_ph, ax_r_ph]:
        ax.set_ylim(0, 7)
        ax.grid(True, alpha=0.3)
    
    ax_l_ang.set_title("Left Hip Angle (deg)")
    ax_l_ph.set_title("Left Hip Phase (rad)")
    ax_r_ang.set_title("Right Hip Angle (deg)")
    ax_r_ph.set_title("Right Hip Phase (rad)")

    def update(frame):
        with PLOT_DATA.lock:
            # 深拷贝一份，防止迭代时修改
            l_t = list(PLOT_DATA.left.t)
            l_raw = list(PLOT_DATA.left.raw_ang)
            l_est = list(PLOT_DATA.left.est_ang)
            l_ph = list(PLOT_DATA.left.phase)
            
            r_t = list(PLOT_DATA.right.t)
            r_raw = list(PLOT_DATA.right.raw_ang)
            r_est = list(PLOT_DATA.right.est_ang)
            r_ph = list(PLOT_DATA.right.phase)
        
        # Update Left
        if l_t:
            ln_l_raw.set_data(l_t, l_raw)
            ln_l_est.set_data(l_t, l_est)
            ln_l_ph.set_data(l_t, l_ph)
            
            # Scroll Left X
            xlim_min = max(0, l_t[-1] - 10.0)
            xlim_max = l_t[-1] + 1.0
            ax_l_ang.set_xlim(xlim_min, xlim_max)
            ax_l_ph.set_xlim(xlim_min, xlim_max)

        # Update Right
        if r_t:
            ln_r_raw.set_data(r_t, r_raw)
            ln_r_est.set_data(r_t, r_est)
            ln_r_ph.set_data(r_t, r_ph)
            
            # Scroll Right X (independent)
            xlim_min = max(0, r_t[-1] - 10.0)
            xlim_max = r_t[-1] + 1.0
            ax_r_ang.set_xlim(xlim_min, xlim_max)
            ax_r_ph.set_xlim(xlim_min, xlim_max)
                
        return [ln_l_raw, ln_l_est, ln_l_ph, ln_r_raw, ln_r_est, ln_r_ph]

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    
    print("窗口已打开，关闭窗口以退出程序...")
    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main_plot()
