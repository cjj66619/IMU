import asyncio
import sys
import os
import time
import numpy as np
import struct
import serial
import serial.tools.list_ports
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from bleak import BleakScanner, BleakClient

# --- 路径设置 ---
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

try:
    from movella_dot_py.core.sensor import MovellaDOTSensor
    from movella_dot_py.core.parser import PayloadParser
    from movella_dot_py.models.data_structures import SensorConfiguration
    from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode
except ImportError:
    print("错误：无法导入 'movella_dot_py' 库。")

# ================= 1. 配置参数 =================
# SERIAL_PORT = "COM11"
# 自动判断系统并选择默认串口
if sys.platform.startswith("win"):
    SERIAL_PORT = "COM10"
else:
    # Jetson NX / Linux 下常见串口名
    # 如果是 USB 转串口，通常是 /dev/ttyUSB0 或 /dev/ttyACM0
    # 如果是 GPIO 引脚 (Pin 8/10)，通常是 /dev/ttyTHS0
    SERIAL_PORT = "/dev/ttyUSB0" 
    # SERIAL_PORT = "/dev/ttyACM0"
    # SERIAL_PORT = "/dev/ttyTHS0"
BAUD_RATE = 115200
MAX_ASSIST_ANGLE =760.0
OLLOW_GAIN = 1.0
ASSIST_LEGS = ["Left_Hip", "Right_Hip"]
CONTROL_MODE = "ASSIST_CURVE"

SENSOR_RATE = 30
DT_ONLINE = 1.0 / SENSOR_RATE
LAMBDA_CF = 0.02
STATIC_DURATION_SEC = 3.0
GRAVITY = 9.80665
DEG_TO_RAD = np.pi / 180.0
RAD_TO_DEG = 180.0 / np.pi
ADAPTIVE_CENTERING_ALPHA = 0.01

# 静止检测参数 (与离线分析一致)
STILLNESS_THRESHOLD = 0.25 # rad/s
STATIC_FRAME_COUNT = 6     # 连续帧数

PLOT_WINDOW = 300

# --- IMU 配置 ---
PELVIS_MAC = "D4:22:CD:00:7F:39"
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

SYSTEM_STATUS = {
    "STM32_Connected": False,
    "Left_Hip_Active": False,
    "Right_Hip_Active": False
}

# ================= 2. 共享数据容器 =================
class LegBuffer:
    def __init__(self, max_len=PLOT_WINDOW):
        self.t = deque(maxlen=max_len)
        self.raw_ang = deque(maxlen=max_len)
        self.est_ang = deque(maxlen=max_len)
        self.cmd_ang = deque(maxlen=max_len)
        self.phase = deque(maxlen=max_len)

class PlotData:
    def __init__(self):
        self.lock = threading.Lock()
        self.start_time = time.time()
        self.left = LegBuffer()
        self.right = LegBuffer()

    def add(self, leg_name, raw, est, cmd, phase):
        with self.lock:
            now = time.time() - self.start_time
            buf = self.left if leg_name == "Left_Hip" else self.right
            buf.t.append(now)
            buf.raw_ang.append(raw)
            buf.est_ang.append(est)
            buf.cmd_ang.append(cmd)
            buf.phase.append(phase)

PLOT_DATA = PlotData()


# ================= 3. 串口类 =================
class SerialCommander:
    def __init__(self, port, baudrate):
        self.ser = None
        self.is_running = True
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01, write_timeout=0)
            print(f"✅ [串口] {port} 打开成功")
            SYSTEM_STATUS["STM32_Connected"] = True
            self.rx_thread = threading.Thread(target=self.rx_drain_worker, daemon=True)
            self.rx_thread.start()
        except Exception as e:
            print(f"❌ [串口] 打开失败: {e}")
            SYSTEM_STATUS["STM32_Connected"] = False

    def send_dual_control_packet(self, ang_l, vel_l, ang_r, vel_r):
        if self.ser and self.ser.is_open:
            try:
                # 协议: Header(0xA5) + L_Ang + L_Vel + R_Ang + R_Vel + Tail(0x5A)
                # 共 1 + 4*4 + 1 = 18 字节
                payload = struct.pack('<ffff', float(ang_l), float(vel_l), float(ang_r), float(vel_r))
                packet = bytes([0xA5]) + payload + bytes([0x5A])
                self.ser.write(packet)
            except Exception: pass

    def rx_drain_worker(self):
        while self.is_running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0: self.ser.read_all()
                time.sleep(0.1)
            except: time.sleep(0.5)

    def close(self):
        self.is_running = False
        if self.ser: self.ser.close()


class DualCommandState:
    def __init__(self):
        self.lock = threading.Lock()
        # [Angle, Velocity]
        self.cmd_left = [0.0, 0.0] 
        self.cmd_right = [0.0, 0.0]

    def update(self, leg_name, angle, velocity):
        with self.lock:
            if leg_name == "Left_Hip":
                self.cmd_left = [angle, velocity]
            elif leg_name == "Right_Hip":
                self.cmd_right = [angle, velocity]

    def get_packet_data(self):
        with self.lock:
            # 返回副本
            return (*self.cmd_left, *self.cmd_right)

GLOBAL_CMD_STATE = DualCommandState()

async def command_sender_loop(commander: SerialCommander, interval=0.01):
    print("🚀 [串口发送] 任务启动")
    while True:
        try:
            ang_l, vel_l, ang_r, vel_r = GLOBAL_CMD_STATE.get_packet_data()
            commander.send_dual_control_packet(ang_l, vel_l, ang_r, vel_r)
            await asyncio.sleep(interval)
        except asyncio.CancelledError:
            break


# ================= 4. 算法与逻辑 =================

def check_leg_static_instant(gyro1, gyro2, threshold=STILLNESS_THRESHOLD):
    mag1 = np.linalg.norm(gyro1)
    mag2 = np.linalg.norm(gyro2)
    return (mag1 < threshold) and (mag2 < threshold)

def generate_smooth_bell_curve(phase, center_phase=0.0, width=0.8, amplitude=1.0):
    diff = np.abs(phase - center_phase)
    diff = np.minimum(diff, 2 * np.pi - diff)
    val = amplitude * np.exp(- (diff ** 2) / (2 * width ** 2))
    return val

class SharedGaitState:
    def __init__(self):
        # 0: Left Phase (rad), 1: Right Phase (rad) (基频相位 phi[1])
        # 使用数组原子操作比锁更快，但在Python里还是建议简单锁
        self.phases = { "Left_Hip": 0.0, "Right_Hip": np.pi } 
        self.omegas = { "Left_Hip": 1.0, "Right_Hip": 1.0 }

SHARED_GAIT = SharedGaitState()

class TricomiAdaptiveOscillator:
    def __init__(self, dt, num_harmonics=3, init_freq=1.0):
        self.dt = dt
        self.N = num_harmonics
        self.nu_phi = 20.0; self.nu_omega = 5.0; self.eta = 5.0
        self.omega = init_freq * 2 * np.pi
        self.alpha = np.zeros(self.N + 1); self.alpha[1] = 0.1
        self.phi = np.zeros(self.N + 1)
        self.estimated_signal = 0.0

    # 重置状态，支持基于当前速度和角度初始化相位
    def reset_state(self, current_angle_rad=0.0, current_vel_rad_s=0.0):
        self.alpha = np.zeros(self.N + 1)
        self.alpha[1] = 0.1
        self.phi = np.zeros(self.N + 1)
        
        # 启发式相位初始化 V2 (基于速度)
        threshold = 0.5
        
        if current_vel_rad_s > threshold:
             self.phi[1] = 0.0   # 屈曲加速 -> Start of Cycle
        elif current_vel_rad_s < -threshold:
             self.phi[1] = np.pi # 伸展加速 -> Mid Cycle
        else:
             # 速度较小时回退到角度判断
             if current_angle_rad < -5 * DEG_TO_RAD: 
                 self.phi[1] = np.pi
             else:
                 self.phi[1] = 0.0
             
        self.omega = 0.8 * 2 * np.pi 

    def update(self, signal_val, coupling_force=0.0):
        # coupling_force 是外部注入的相位调整量 (通常由双腿耦合产生)
        harmonic_sum = sum(self.alpha[n] * np.sin(self.phi[n]) for n in range(1, self.N + 1))
        self.estimated_signal = self.alpha[0] + harmonic_sum
        F_t = signal_val - self.estimated_signal
        sum_alpha = np.sum(self.alpha[1:])
        if sum_alpha < 1e-6: sum_alpha = 1e-6

        d_phi = np.zeros(self.N + 1)
        for n in range(1, self.N + 1):
            coupling = self.nu_phi * (F_t / sum_alpha) * np.cos(self.phi[n])
            # 对于基频 (n=1), 加入外部耦合力 (比如来自另一条腿的牵引)
            extra = coupling_force if n == 1 else 0.0
            d_phi[n] = n * self.omega + coupling + extra
            
        d_omega = self.nu_omega * (F_t / sum_alpha) * np.cos(self.phi[1])
        d_alpha = np.zeros(self.N + 1); d_alpha[0] = self.eta * F_t
        for n in range(1, self.N + 1): d_alpha[n] = self.eta * F_t * np.sin(self.phi[n])

        self.phi += d_phi * self.dt
        self.omega += d_omega * self.dt
        self.alpha += d_alpha * self.dt

        base_shift = -np.pi / 2
        if self.alpha[1] < 0: base_shift += np.pi
        gait_phase = (self.phi[1] + base_shift) % (2 * np.pi)
        return self.estimated_signal, gait_phase, self.omega / (2 * np.pi)


class JointAngleCalculator:
    def __init__(self, calibration_file_path, sensor_rate_hz, lambda_cf, reverse_sign):
        self.reverse_sign = reverse_sign
        if not os.path.exists(calibration_file_path):
            self.j_axis1 = np.array([0, 0, 1.0]); self.j_axis2 = np.array([0, 0, 1.0])
            self.o1 = np.zeros(3); self.o2 = np.zeros(3)
        else:
            p = np.load(calibration_file_path)
            self.j_axis1 = p['j_axis1']; self.j_axis2 = p['j_axis2']
            self.o1 = p['o1']; self.o2 = p['o2']

        self.lambda_cf = lambda_cf
        self.dt = 1.0 / sensor_rate_hz
        self.gyr_angle_rad_prev = None; self.angle_offset_rad = 0.0
        self.gyr1_bias = np.zeros(3); self.gyr2_bias = np.zeros(3)
        self.is_calibrated = False
        self.dynamic_bias = 0.0; self.centering_alpha = ADAPTIVE_CENTERING_ALPHA

    def perform_static_calibration(self, acc1, gyr1, acc2, gyr2):
        self.gyr1_bias = np.mean(gyr1, axis=0); self.gyr2_bias = np.mean(gyr2, axis=0)
        angles = [self._calc_acc_angle(acc1[i], np.zeros(3), acc2[i], np.zeros(3)) for i in range(len(acc1))]
        self.angle_offset_rad = np.mean(angles)
        self.gyr_angle_rad_prev = self.angle_offset_rad
        self.dynamic_bias = 0.0
        self.is_calibrated = True

    def reset_state(self):
        self.gyr_angle_rad_prev = None
        self.dynamic_bias = 0.0

    def _calc_acc_angle(self, a1_g, g1, a2_g, g2):
        a1 = a1_g * GRAVITY; a2 = a2_g * GRAVITY
        r1 = a1 - np.cross(g1, np.cross(g1, self.o1)); r2 = a2 - np.cross(g2, np.cross(g2, self.o2))
        p1 = r1 - np.dot(r1, self.j_axis1) * self.j_axis1; p2 = r2 - np.dot(r2, self.j_axis2) * self.j_axis2
        n1, n2 = np.linalg.norm(p1), np.linalg.norm(p2)
        if n1 < 1e-6 or n2 < 1e-6: return self.gyr_angle_rad_prev or 0.0
        u1, u2 = p1 / n1, p2 / n2
        return np.arctan2(np.dot(np.cross(u1, u2), self.j_axis1), np.clip(np.dot(u1, u2), -1.0, 1.0))

    def calculate_step(self, a1, g1, a2, g2):
        if not self.is_calibrated: return 0.0, 0.0
        g1 -= self.gyr1_bias; g2 -= self.gyr2_bias
        acc_angle = self._calc_acc_angle(a1, g1, a2, g2)
        
        if self.gyr_angle_rad_prev is None:
            self.gyr_angle_rad_prev = acc_angle

        w_rel = np.dot(g2, self.j_axis2) - np.dot(g1, self.j_axis1)
        gyr_pred = self.gyr_angle_rad_prev + w_rel * self.dt
        err = (acc_angle - gyr_pred + np.pi) % (2 * np.pi) - np.pi
        fused = gyr_pred + self.lambda_cf * err
        self.gyr_angle_rad_prev = fused
        raw_rel_rad = (fused - self.angle_offset_rad + np.pi) % (2 * np.pi) - np.pi
        self.dynamic_bias = (1.0 - self.centering_alpha) * self.dynamic_bias + self.centering_alpha * raw_rel_rad
        rel_deg = (raw_rel_rad - self.dynamic_bias) * RAD_TO_DEG
        rel_deg = (rel_deg + 180) % 360 - 180
        if self.reverse_sign: rel_deg = -rel_deg
        
        # 返回 (角度, 角速度)
        return rel_deg, w_rel * RAD_TO_DEG

# ================= 5. 数据流与控制循环 =================

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
                if (now - p_ts) < 0.15:
                    pair_data = ((p_acc, p_gyr), (acc, gyr))
                    target_queue = queues.get("Left_Hip") if sensor_address == LEFT_THIGH_MAC else queues.get("Right_Hip")
                    if target_queue:
                        if target_queue.full(): 
                            try: target_queue.get_nowait()
                            except: pass
                        target_queue.put_nowait(pair_data)
    except: pass

async def process_limb_control_loop(limb_name: str, queue: asyncio.Queue, calculator: JointAngleCalculator, commander: SerialCommander):
    ao = TricomiAdaptiveOscillator(dt=DT_ONLINE, init_freq=0.8)
    c_acc1, c_gyr1, c_acc2, c_gyr2 = [], [], [], []
    calib_limit = int(STATIC_DURATION_SEC * SENSOR_RATE)
    
    smooth_pos, smooth_vel = None, 0.0
    prev_target_pos = 0.0
    alpha = 0.7
    
    # 静止检测变量
    static_counter = 0
    static_counter = 0
    is_really_static = False
    
    # [Start Logic]
    movement_start_phi = None
    STARTUP_CYCLES = 1.5  # 1.5个周期 ≈ 3步 (左右左)
    print(f"[{limb_name}] 任务就绪")
    
    while True:
        try:
            (a1_raw, g1_raw), (a2_raw, g2_raw) = await queue.get()
            a1, a2 = a1_raw / GRAVITY, a2_raw / GRAVITY
            g1, g2 = g1_raw * DEG_TO_RAD, g2_raw * DEG_TO_RAD

            if not calculator.is_calibrated:
                c_acc1.append(a1); c_gyr1.append(g1); c_acc2.append(a2); c_gyr2.append(g2)
                if len(c_acc1) >= calib_limit:
                    calculator.perform_static_calibration(np.array(c_acc1), np.array(c_gyr1), np.array(c_acc2), np.array(c_gyr2))
                    print(f"✅ [{limb_name}] 标定完成")
                    ao.phi[1] = 0.0
                queue.task_done()
                continue
            
            # --- 1. 静止检测 ---
            # [Fix] 使用去除零偏后的 Gyro 数据进行检测，更准确
            check_g1, check_g2 = g1, g2
            if calculator.is_calibrated:
                check_g1 = g1 - calculator.gyr1_bias
                check_g2 = g2 - calculator.gyr2_bias

            if check_leg_static_instant(check_g1, check_g2):
                static_counter += 1
                if static_counter >= STATIC_FRAME_COUNT:
                    is_really_static = True
            else:
                static_counter = 0
                is_really_static = False

            # --- 2. 算法计算 ---
            if is_really_static:
                # 静止 ==> 各种重置
                calculator.reset_state()
                # 此时不Reset AO，等恢复运动时再Reset
                angle_deg = 0.0
                est_rad, phase_rad = 0.0, 0.0
                target_pos = 0.0
                if smooth_pos is None: smooth_pos = 0.0
                movement_start_phi = None # 静止时重置
            else:
                # 运动


                # 如果 calculator 刚刚被 reset 过 (gyr_angle_rad_prev is None), 说明这是第一帧
                is_first_frame = (calculator.gyr_angle_rad_prev is None)
                
                # 获取角度和速度
                angle_deg, vel_deg_s = calculator.calculate_step(a1, g1, a2, g2)
                
                if is_first_frame:
                    # 使用当前角度和速度进行初始化
                    ao.reset_state(angle_deg * DEG_TO_RAD, vel_deg_s * DEG_TO_RAD)
                
                # [Start Logic] 记录起始相位 (如果在刚开始运动)
                if movement_start_phi is None:
                    movement_start_phi = ao.phi[1]
                
                # [Fix] 防止初始突变：如果是第一帧或从未初始化，将 smooth_pos 对齐到 0.0 (电机零点)
                # 这样可以避免电机上电时刻(物理0点)与算法初始计算角度之间的偏差导致跳变
                if smooth_pos is None or is_first_frame:
                    smooth_pos = 0.0
                    prev_target_pos = 0.0
                    smooth_vel = 0.0

                # 计算耦合力: K * sin(phi_other - phi_self - PI)
                # Left 找 Right, Right 找 Left
                other_limb = "Right_Hip" if limb_name == "Left_Hip" else "Left_Hip"
                
                # 读取对方的相位 (不加锁直接读 approximate 值即可，对控制鲁棒性影响不大)
                phi_self = ao.phi[1]
                phi_other = SHARED_GAIT.phases.get(other_limb, 0.0)
                
                # 耦合强度 K
                COUPLING_STRENGTH = 2.0 
                # 目标相位差: Left - Right = PI or Right - Left = -PI. 统一用 diff - PI = 0
                # sin(phi_other - phi_self - PI)
                coupling_val = COUPLING_STRENGTH * np.sin(phi_other - phi_self - np.pi)
                
                est_rad, phase_rad, omega_Hz = ao.update(angle_deg * DEG_TO_RAD, coupling_force=coupling_val)
                
                # 更新共享状态供对方使用
                SHARED_GAIT.phases[limb_name] = ao.phi[1]
                
            target_pos = 0.0
            if limb_name in ASSIST_LEGS:
                if CONTROL_MODE == "ASSIST_CURVE":
                    gain = generate_smooth_bell_curve(phase_rad, center_phase=0.0)
                    target_pos = gain * MAX_ASSIST_ANGLE
                else:
                    target_pos = angle_deg * FOLLOW_GAIN

            # 3. 发送控制
            if limb_name in ASSIST_LEGS:
                # [New] 预热判定: 累计相位变化超过 1.5 个周期 (约3步) 后才助力
                current_phi = ao.phi[1]
                threshold_phi = STARTUP_CYCLES * 2 * np.pi
                
                if movement_start_phi is not None and (abs(current_phi - movement_start_phi) > threshold_phi):
                     pass # 允许正常输出 target_pos
                else:
                     target_pos = 0.0 # 预热期保持 0
                
                smooth_pos = (1 - alpha) * smooth_pos + alpha * target_pos
                raw_vel = (smooth_pos - prev_target_pos) / DT_ONLINE
                smooth_vel = (1 - alpha) * smooth_vel + alpha * raw_vel
                prev_target_pos = smooth_pos
                if commander: 
                    # 更新全局指令状态，不直接发送
                    # [User Request] 右腿电机方向反转
                    send_pos, send_vel = smooth_pos, smooth_vel
                    if limb_name == "Right_Hip":
                        send_pos = -send_pos
                        send_vel = -send_vel
                    
                    GLOBAL_CMD_STATE.update(limb_name, send_pos, send_vel)

            # 4. 更新绘图数据
            PLOT_DATA.add(limb_name, angle_deg, est_rad * RAD_TO_DEG, target_pos, phase_rad)
            queue.task_done()

        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"Error in loop {limb_name}: {e}")
            import traceback
            traceback.print_exc()
            await asyncio.sleep(0.01)


# ================= 6. 主程序架构 =================

async def ble_main_loop():
    commander = SerialCommander(SERIAL_PORT, BAUD_RATE)
    try:
        calc_l = JointAngleCalculator(LEFT_HIP_CONFIG["CALIB_FILE"], SENSOR_RATE, LAMBDA_CF, LEFT_HIP_CONFIG["REVERSE"])
        calc_r = JointAngleCalculator(RIGHT_HIP_CONFIG["CALIB_FILE"], SENSOR_RATE, LAMBDA_CF, RIGHT_HIP_CONFIG["REVERSE"])
    except Exception as e:
        print(f"❌ 初始化算法失败(通常是缺少标定文件): {e}")
        return

    print("🔍 正在扫描蓝牙设备...")
    devices = await BleakScanner.discover(timeout=5.0)
    print(f"扫描结束，发现 {len(devices)} 个设备")

    targets = [d for d in devices if d.address in TARGET_ADDRESSES]
    if len(targets) == 0: print("❌ 未找到传感器"); return

    config = SensorConfiguration(output_rate=OutputRate(SENSOR_RATE), filter_profile=FilterProfile.DYNAMIC, payload_mode=PayloadMode.CUSTOM_MODE_5)
    queues = {"Left_Hip": asyncio.Queue(maxsize=1), "Right_Hip": asyncio.Queue(maxsize=1)}
    
    parsers = {}
    sensors = []

    for dev in targets:
        print(f"尝试连接: {dev.address}")
        try:
            s = MovellaDOTSensor(config)
            s.client = BleakClient(dev.address)
            await s.client.connect(timeout=10.0)
            await s.configure_sensor()
            parsers[dev.address] = PayloadParser(config.payload_mode)
            s.notification_handler = lambda s, d, a=dev.address, p=parsers[dev.address]: handle_notification_queue(s, d, a, p, queues)
            sensors.append(s)
            print(f"  -> {dev.address} Ready")
        except: pass

    asyncio.create_task(process_limb_control_loop("Left_Hip", queues["Left_Hip"], calc_l, commander))
    asyncio.create_task(process_limb_control_loop("Right_Hip", queues["Right_Hip"], calc_r, commander))
    
    # 启动双腿发送任务 (100Hz)
    sender_task = asyncio.create_task(command_sender_loop(commander, interval=0.01))

    print("\n=== 请保持静止 3 秒标定 ===")
    if sensors: await asyncio.gather(*(s.start_measurement() for s in sensors))

    try:
        await asyncio.Event().wait()
    except asyncio.CancelledError:
        pass
    finally:
        sender_task.cancel()
        commander.close()
        for s in sensors:
            if s.is_connected: await s.stop_measurement(); await s.disconnect()

def run_background_loop():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(ble_main_loop())
    except Exception as e:
        print(e)

def main_ui():
    t = threading.Thread(target=run_background_loop, daemon=True)
    t.start()
    
    plt.style.use('dark_background')
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    ax_l_ang, ax_l_ph = axes[0,0], axes[0,1]
    ax_r_ang, ax_r_ph = axes[1,0], axes[1,1]
    
    l_raw, = ax_l_ang.plot([], [], 'c-', alpha=0.5, label='Raw')
    l_est, = ax_l_ang.plot([], [], 'w--', lw=1.5, label='AO Est')
    l_cmd, = ax_l_ang.plot([], [], 'r-', lw=2, label='Command')
    l_ph, = ax_l_ph.plot([], [], 'y-', label='Phase')
    
    r_raw, = ax_r_ang.plot([], [], 'm-', alpha=0.5, label='Raw')
    r_est, = ax_r_ang.plot([], [], 'w--', lw=1.5, label='AO Est')
    r_cmd, = ax_r_ang.plot([], [], 'r-', lw=2, label='Command')
    r_ph, = ax_r_ph.plot([], [], 'g-', label='Phase')

    for ax in [ax_l_ang, ax_r_ang]:
        ax.set_ylim(-60, 100)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
    for ax in [ax_l_ph, ax_r_ph]:
        ax.set_ylim(0, 7)
        ax.grid(True, alpha=0.3)

    ax_l_ang.set_title("Left Hip (Assist) [Smart Static V2]"); ax_l_ph.set_title("Left Phase")
    ax_r_ang.set_title("Right Hip (Passive)"); ax_r_ph.set_title("Right Phase")

    def update(_):
        with PLOT_DATA.lock:
            l_t = list(PLOT_DATA.left.t); l_r = list(PLOT_DATA.left.raw_ang)
            l_e = list(PLOT_DATA.left.est_ang); l_c = list(PLOT_DATA.left.cmd_ang)
            l_p = list(PLOT_DATA.left.phase)
            
            r_t = list(PLOT_DATA.right.t); r_r = list(PLOT_DATA.right.raw_ang)
            r_e = list(PLOT_DATA.right.est_ang); r_c = list(PLOT_DATA.right.cmd_ang)
            r_p = list(PLOT_DATA.right.phase)

        if l_t:
            l_raw.set_data(l_t, l_r); l_est.set_data(l_t, l_e); l_cmd.set_data(l_t, l_c)
            l_ph.set_data(l_t, l_p)
            ax_l_ang.set_xlim(max(0, l_t[-1]-10), l_t[-1]+1)
            ax_l_ph.set_xlim(max(0, l_t[-1]-10), l_t[-1]+1)

        if r_t:
            r_raw.set_data(r_t, r_r); r_est.set_data(r_t, r_e); r_cmd.set_data(r_t, r_c)
            r_ph.set_data(r_t, r_p)
            ax_r_ang.set_xlim(max(0, r_t[-1]-10), r_t[-1]+1)
            ax_r_ph.set_xlim(max(0, r_t[-1]-10), r_t[-1]+1)

        return [l_raw, l_est, l_cmd, l_ph, r_raw, r_est, r_cmd, r_ph]

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if sys.platform == "win32": asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    main_ui()
