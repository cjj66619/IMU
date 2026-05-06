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
CURRENT_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(CURRENT_SCRIPT_DIR, "data_log")

# 截止频率 (Hz) - 单IMU数据通常噪声稍大，保持6Hz或适当调低
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
            print("请先运行采集脚本生成数据！")

    def find_latest_shank(self):
        """只寻找最新的 Left_Shank 数据"""
        # 搜索模式改为只找 Left_Shank
        search_pattern = os.path.join(self.data_dir, "*Left_Shank*.csv")
        files = glob.glob(search_pattern)
        
        if not files:
            raise FileNotFoundError(f"在 {self.data_dir} 没找到包含 'Left_Shank' 的CSV文件")
        
        # 按时间倒序排列，取第一个
        files.sort(key=os.path.getmtime, reverse=True)
        latest_file = files[0]
            
        print(f"📂 正在分析单通道数据:\n  -> {os.path.basename(latest_file)}")
        return latest_file

    def load_data(self, file_path):
        """加载单文件数据，不再需要 merge_asof 对齐"""
        df = pd.read_csv(file_path)
        
        # 为了保持逻辑一致性，加上前缀（防止原始CSV没有前缀）
        df = df.add_prefix('S_')
        
        # 确保时间戳列名正确（假设原始数据里有一列是 Timestamp_ms 或类似）
        # 如果原始列名变成了 S_Timestamp_ms，我们需要把它复制一份作为主索引
        if 'S_Timestamp_ms' in df.columns:
            df['Timestamp_ms'] = df['S_Timestamp_ms']
        else:
            # 容错处理：如果没有时间戳，按采样率生成一个
            print("⚠️ 未检测到时间戳，根据采样率自动生成...")
            df['Timestamp_ms'] = np.arange(len(df)) * (1000.0 / SAMPLING_RATE)

        df.sort_values('Timestamp_ms', inplace=True)
        return df

    def low_pass_filter(self, data):
        nyq = 0.5 * SAMPLING_RATE
        normal_cutoff = CUTOFF_FREQ / nyq
        b, a = butter(2, normal_cutoff, btype='low', analog=False)
        return filtfilt(b, a, data)

    def calculate_shank_angle(self, df):
        """
        计算小腿的绝对摆动角 (Segment Angle)。
        原理：将四元数转为欧拉角，寻找方差最大的轴作为矢状面摆动轴。
        """
        # 提取四元数 (注意：这里只取小腿数据)
        q_vals = df[['S_Quat_X', 'S_Quat_Y', 'S_Quat_Z', 'S_Quat_W']].values
        
        # 转换为旋转对象
        r = R.from_quat(q_vals)
        
        # 转为欧拉角 (XYZ顺序，单位度)
        euler_angles = r.as_euler('xyz', degrees=True)
        
        # 自动检测主运动轴：
        # 走路时，小腿主要在一个平面（矢状面）摆动，那个轴的方差最大
        variances = np.var(euler_angles, axis=0)
        flexion_axis_idx = np.argmax(variances)
        
        raw_angle = euler_angles[:, flexion_axis_idx]
        
        # 简单处理：减去均值，让震荡中心在0度附近（方便看相图）
        # 注意：这只是为了相位分析，不是解剖学上的绝对垂直角度
        centered_angle = raw_angle - np.mean(raw_angle)
        
        return self.low_pass_filter(centered_angle)

    def compute_phase(self, angle):
        """计算相位图数据 (Angle vs Velocity)"""
        dt = 1.0 / SAMPLING_RATE
        velocity = np.gradient(angle, dt)
        
        # 归一化处理（为了画圆形相图）
        angle_centered = angle - np.mean(angle)
        angle_norm = angle_centered / (np.std(angle_centered) + 1e-6)
        
        velocity_filtered = self.low_pass_filter(velocity)
        velocity_norm = velocity_filtered / (np.std(velocity_filtered) + 1e-6)
        
        # 计算连续相位 (0-100%)
        # arctan2(y, x) -> 这里通常用 (-velocity, angle) 来顺时针映射步态周期
        phase_rad = np.arctan2(-velocity_norm, angle_norm)
        phase_percent = (phase_rad + np.pi) / (2 * np.pi) * 100.0
        
        # 返回用于绘图的原始/滤波数据，而非归一化数据
        return angle, velocity_filtered, phase_percent

    def visualize(self, timestamp, angle, velocity, phase):
        plt.style.use('dark_background')
        fig = plt.figure(figsize=(14, 10))
        fig.suptitle(f"Single-Shank Gait Analysis", fontsize=16, color='cyan')

        # 1. 角度图
        ax1 = fig.add_subplot(2, 2, 1)
        time_s = (timestamp - timestamp[0]) / 1000.0
        ax1.plot(time_s, angle, 'yellow', linewidth=1.5)
        ax1.set_title("Shank Swing Angle (deg)", color='white') # 标题改为小腿摆动角
        ax1.set_ylabel("Angle (°)")
        ax1.grid(True, alpha=0.3)

        # 2. 相位百分比图
        ax2 = fig.add_subplot(2, 2, 3)
        ax2.plot(time_s, phase, 'cyan', linewidth=1.5)
        ax2.set_title("Gait Phase Estimate (%)", color='white')
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Phase (0-100%)")
        ax2.grid(True, alpha=0.3)
        # 移除 HS/TO 线，因为单IMU估算这两个事件的百分比可能不准，仅保留参考线
        ax2.axhline(0, color='r', linestyle='--', alpha=0.3)
        ax2.axhline(100, color='r', linestyle='--', alpha=0.3)

        # 3. 相图 (Phase Portrait)
        ax3 = fig.add_subplot(1, 2, 2)
        # 使用散点图映射相位颜色
        sc = ax3.scatter(angle, velocity, c=phase, cmap='hsv', s=5, alpha=0.6)
        ax3.set_title("Shank Phase Portrait", color='white')
        ax3.set_xlabel("Shank Angle (°)")
        ax3.set_ylabel("Angular Velocity (°/s)")
        ax3.grid(True, alpha=0.3)
        plt.colorbar(sc, ax=ax3).set_label('Phase %')

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    try:
        print(f"🤖 初始化单通道分析引擎... (数据目录: {DATA_DIR})")
        analyst = GaitAnalyst(DATA_DIR)
        
        # 1. 找文件 (只找 Shank)
        shank_file = analyst.find_latest_shank()
        
        # 2. 加载数据
        print("📥 加载数据...")
        df = analyst.load_data(shank_file)
        
        if len(df) < 100:
            print("⚠️ 数据太短，无法分析！")
        else:
            # 3. 计算运动学
            print("📐 计算小腿摆动角...")
            angle_filtered = analyst.calculate_shank_angle(df)
            
            # 4. 计算相位
            print("🌊 计算相平面...")
            ang, vel, phase = analyst.compute_phase(angle_filtered)
            
            # 5. 绘图
            print("📊 绘图...")
            analyst.visualize(df['Timestamp_ms'].values, ang, vel, phase)
            print("✅ 分析完成。")
            
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"\n❌ 错误: {e}")