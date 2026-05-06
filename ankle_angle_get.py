import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

shank_file = 'IMU/left_leg_analysis/data_log/Left_Shank_20260407_114604.csv'
foot_file = 'IMU/left_leg_analysis/data_log/Left_Foot_20260407_114604.csv'

shank_df = pd.read_csv(shank_file)
foot_df = pd.read_csv(foot_file)

min_len = min(len(shank_df), len(foot_df))
shank_df = shank_df.iloc[:min_len].copy()
foot_df = foot_df.iloc[:min_len].copy()
shank_df['Time_sec'] = shank_df['Timestamp_s'] / 1000.0

q_shank_raw = shank_df[['Quat_X', 'Quat_Y', 'Quat_Z', 'Quat_W']].to_numpy()
q_foot_raw = foot_df[['Quat_X', 'Quat_Y', 'Quat_Z', 'Quat_W']].to_numpy()

rot_shank_raw = R.from_quat(q_shank_raw)
rot_foot_raw = R.from_quat(q_foot_raw)

R_shank_to_AF_mat = np.array([
    [-1,  0,  0], 
    [ 0,  0,  1], 
    [ 0,  1,  0]  
])
rot_align_shank = R.from_matrix(R_shank_to_AF_mat)

R_foot_to_AF_mat = np.array([
    [ 1,  0,  0], 
    [ 0,  1,  0], 
    [ 0,  0,  1]  
])
rot_align_foot = R.from_matrix(R_foot_to_AF_mat)

rot_shank_af = rot_shank_raw * rot_align_shank.inv()
rot_foot_af = rot_foot_raw * rot_align_foot.inv()

rot_ankle = rot_shank_af.inv() * rot_foot_af

euler_ankle = rot_ankle.as_euler('xyz', degrees=True)

shank_df['Ankle_Dorsi_Plantar_raw'] = euler_ankle[:, 0] 
shank_df['Ankle_Inver_Ever_raw'] = euler_ankle[:, 1]    
shank_df['Ankle_Inter_Exter_raw'] = euler_ankle[:, 2]   

calib_samples = 50 
offset_dp = shank_df['Ankle_Dorsi_Plantar_raw'].iloc[:calib_samples].mean()
offset_ie = shank_df['Ankle_Inver_Ever_raw'].iloc[:calib_samples].mean()
offset_rot = shank_df['Ankle_Inter_Exter_raw'].iloc[:calib_samples].mean()

shank_df['Ankle_Dorsi_Plantar'] = shank_df['Ankle_Dorsi_Plantar_raw'] - offset_dp
shank_df['Ankle_Inver_Ever'] = shank_df['Ankle_Inver_Ever_raw'] - offset_ie
shank_df['Ankle_Inter_Exter'] = shank_df['Ankle_Inter_Exter_raw'] - offset_rot

plt.rcParams['font.sans-serif'] = ['SimHei'] 
plt.rcParams['axes.unicode_minus'] = False

plt.figure(figsize=(14, 7), dpi=150)
plt.plot(shank_df['Time_sec'], shank_df['Ankle_Dorsi_Plantar'], label='背屈(+) / 跖屈(-)', color='red', linewidth=2.5)
plt.plot(shank_df['Time_sec'], shank_df['Ankle_Inver_Ever'], label='内翻 / 外翻', color='blue', alpha=0.7)
plt.plot(shank_df['Time_sec'], shank_df['Ankle_Inter_Exter'], label='内旋 / 外旋', color='green', alpha=0.7)

plt.title('左侧踝关节运动学角度', fontsize=16)
plt.xlabel('时间 (秒)', fontsize=12)
plt.ylabel('关节角度 (°)', fontsize=12)
plt.axhline(y=0, color='black', linestyle='--', linewidth=1)
plt.legend(loc='upper right', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)
plt.tight_layout()
plt.show()

output_columns = ['Timestamp_s', 'Time_sec', 'Ankle_Dorsi_Plantar', 'Ankle_Inver_Ever', 'Ankle_Inter_Exter']
shank_df[output_columns].to_csv('IMU/left_leg_analysis/data_log/Final_Ankle_Angles.csv', index=False)