import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, filtfilt

# ==========================================
# 🔧 自动路径配置
# ==========================================
# 自动定位到脚本所在目录下的 data_log 文件夹
CURRENT_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(CURRENT_SCRIPT_DIR, "data_log")

# 截止频率 (Hz)
CUTOFF_FREQ = 6.0  
SAMPLING_RATE = 60.0

# ==========================================
# 核心算法类
# ==========================================

class GaitAnalyst:
    def __init__(self, data_dir):
        self.data_dir = data_dir
        if not os.path.exists(self.data_dir):
            print(f"⚠️ 警告: 数据文件夹不存在 -> {self.data_dir}")
            print("请先运行采集脚本 (left_leg_collection.py) 生成数据！")

    def find_latest_pair(self):
        """自动寻找文件夹中最新的一对数据"""
        files = glob.glob(os.path.join(self.data_dir, "*.csv"))
        if not files:
            raise FileNotFoundError(f"在 {self.data_dir} 没找到CSV文件")
        
        files.sort(key=os.path.getmtime, reverse=True)
        
        thigh_file = None
        shank_file = None
        
        for f in files:
            if "Left_Thigh" in f and thigh_file is None:
                thigh_file = f
            elif "Left_Shank" in f and shank_file is None:
                shank_file = f
            
            if thigh_file and shank_file:
                break
                
        if not thigh_file or not shank_file:
            raise ValueError("未找到成对的 Left_Thigh 和 Left_Shank 数据！")
            
        print(f"📂 正在分析:\n  1. {os.path.basename(thigh_file)}\n  2. {os.path.basename(shank_file)}")
        return thigh_file, shank_file

    def load_and_sync(self, thigh_path, shank_path):
        df_t = pd.read_csv(thigh_path)
        df_s = pd.read_csv(shank_path)
        
        df_t = df_t.add_prefix('T_')
        df_s = df_s.add_prefix('S_')
        
        df_t['Timestamp_ms'] = df_t['T_Timestamp_ms']
        df_s['Timestamp_ms'] = df_s['S_Timestamp_ms']
        
        merged = pd.merge_asof(
            df_t.sort_values('Timestamp_ms'), 
            df_s.sort_values('Timestamp_ms'), 
            on='Timestamp_ms', 
            direction='nearest',
            tolerance=20
        )
        
        merged.dropna(inplace=True)
        return merged

    def low_pass_filter(self, data):
        nyq = 0.5 * SAMPLING_RATE
        normal_cutoff = CUTOFF_FREQ / nyq
        b, a = butter(2, normal_cutoff, btype='low', analog=False)
        return filtfilt(b, a, data)

    def calculate_knee_angle(self, df):
        q_thigh = df[['T_Quat_X', 'T_Quat_Y', 'T_Quat_Z', 'T_Quat_W']].values
        q_shank = df[['S_Quat_X', 'S_Quat_Y', 'S_Quat_Z', 'S_Quat_W']].values
        
        r_t = R.from_quat(q_thigh)
        r_s = R.from_quat(q_shank)
        
        r_rel = r_t.inv() * r_s
        euler_rel = r_rel.as_euler('xyz', degrees=True)
        
        variances = np.var(euler_rel, axis=0)
        flexion_axis_idx = np.argmax(variances)
        knee_angle = euler_rel[:, flexion_axis_idx]
        
        return self.low_pass_filter(knee_angle)

    def compute_phase(self, angle):
        dt = 1.0 / SAMPLING_RATE
        velocity = np.gradient(angle, dt)
        
        angle_centered = angle - np.mean(angle)
        angle_norm = angle_centered / (np.std(angle_centered) + 1e-6)
        
        velocity_filtered = self.low_pass_filter(velocity)
        velocity_norm = velocity_filtered / (np.std(velocity_filtered) + 1e-6)
        
        phase_rad = np.arctan2(-velocity_norm, angle_norm)
        phase_percent = (phase_rad + np.pi) / (2 * np.pi) * 100.0
        
        return angle_filtered, velocity_filtered, phase_percent

    def visualize(self, timestamp, angle, velocity, phase):
        plt.style.use('dark_background')
        fig = plt.figure(figsize=(14, 10))
        fig.suptitle(f"Gait Phase Analysis (Auto-Detected Data)", fontsize=16, color='cyan')

        ax1 = fig.add_subplot(2, 2, 1)
        time_s = (timestamp - timestamp[0]) / 1000.0
        ax1.plot(time_s, angle, 'yellow', linewidth=1.5)
        ax1.set_title("Knee Flexion Angle (deg)", color='white')
        ax1.set_ylabel("Angle (°)")
        ax1.grid(True, alpha=0.3)

        ax2 = fig.add_subplot(2, 2, 3)
        ax2.plot(time_s, phase, 'cyan', linewidth=1.5)
        ax2.set_title("Continuous Gait Phase (%)", color='white')
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Phase (0-100%)")
        ax2.grid(True, alpha=0.3)
        ax2.axhline(0, color='r', linestyle='--', alpha=0.5, label='HS (0%)')
        ax2.axhline(60, color='g', linestyle='--', alpha=0.5, label='TO (60%)')
        ax2.legend()

        ax3 = fig.add_subplot(1, 2, 2)
        sc = ax3.scatter(angle, velocity, c=phase, cmap='hsv', s=5, alpha=0.6)
        ax3.set_title("Phase Portrait", color='white')
        ax3.set_xlabel("Angle")
        ax3.set_ylabel("Velocity")
        ax3.grid(True, alpha=0.3)
        plt.colorbar(sc, ax=ax3).set_label('Phase %')

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    try:
        print(f"🤖 初始化分析引擎... (数据目录: {DATA_DIR})")
        analyst = GaitAnalyst(DATA_DIR)
        
        t_file, s_file = analyst.find_latest_pair()
        print("🔄 对齐时间轴...")
        merged_df = analyst.load_and_sync(t_file, s_file)
        
        if len(merged_df) < 100:
            print("⚠️ 数据太短！")
        else:
            print("📐 计算运动学...")
            angle_filtered = analyst.calculate_knee_angle(merged_df)
            print("🌊 计算相位...")
            ang, vel, phase = analyst.compute_phase(angle_filtered)
            print("📊 绘图...")
            analyst.visualize(merged_df['Timestamp_ms'].values, ang, vel, phase)
            print("✅ 完成。")
            
    except Exception as e:
        print(f"\n❌ 错误: {e}")
