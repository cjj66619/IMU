# IMU 采集（Movella DOT）

双击 **`启动IMU采集.bat`** → 浏览器自动打开采集页面。关闭黑色命令行窗口即退出。

整个文件夹可以直接拷到别的 Windows 电脑（64 位 Win10/11）使用：Python 和 Movella DOT SDK 都已经打包在 `runtime/` 里，**不需要安装任何东西、不需要联网**。

## 使用步骤

1. **连接传感器**：传感器开机（不要放在充电盒里、不要被手机 App 连着）→「扫描传感器」→ 勾选 →「连接」。
   - 「名字」一栏可改成 `Left_Shank` 之类，用作文件名，会记住（按 MAC 地址）。
   - 不知道哪个是哪个？点「闪灯」，对应的传感器会快速闪烁。
2. **采集设置**（改完立即写入传感器，表格里的「采样率」是从传感器读回来的实际值）
   - **实时传到电脑**：边采边存 CSV，有实时曲线；最高 60 Hz；≥2 个传感器时可勾「硬件同步」（开始前约 15 秒）。
   - **存在传感器芯片里**：最高 120 Hz，不怕蓝牙掉包；采完要用 USB 导出（见下）。
   - 采集时长填 `0` = 手动点「停止」。
3. **保存位置**：「选择文件夹…」弹出系统对话框。默认是本文件夹下的 `data/`（相对路径，拷到别的电脑仍然有效）。
4. 点 **开始采集**。

### 芯片里的数据（USB 导出 / 清空）

把传感器放回充电盒、盒子插电脑（或数据线直连）→「查找 USB 传感器」→「全部导出到保存位置」。
导出的 CSV 会按录制开始时间**自动放回**当时网页建的那个采集文件夹；找不到对应的会放进 `IMU_USB导出_时间/`。
确认导出成功后再「清空所选芯片」（芯片满了会出现「开始录制失败」）。

## 输出文件

```
<保存位置>/IMU_20261019_132604[_备注]/
├── Left_Shank_20261019_132604.csv     # 每个传感器一个
├── Right_Shank_20261019_132604.csv
└── session_info.json                  # 模式、采样率、滤波、同步、MAC→名字、开始时间、每个传感器行数/实际 Hz
```

CSV 列（随「数据内容」变化，只写传感器实际给出的量）：

| 列 | 单位 / 说明 |
|---|---|
| `Time(s)` | 硬件同步时以所有传感器最早一帧为 0（可直接跨传感器对齐）；否则以本传感器首帧为 0 |
| `Quat_W..Z` | 姿态四元数 |
| `Roll/Pitch/Yaw(rad)` | 欧拉角，弧度 |
| `Acc_X..Z(g)` | 加速度，按 g = 9.7949 m/s² 换算（与实验室旧脚本一致） |
| `Gyr_X..Z(rad/s)` | 角速度 |
| `FreeAcc_X..Z(m/s2)` / `Mag_X..Z(a.u.)` | 自由加速度 / 磁场（选了对应数据内容才有） |
| `SampleTimeFine(0.1ms)` / `SampleTimeFine(us)` | 传感器时钟原始值，已处理 32 位回绕。单位随来源不同：实时采集为 0.1 ms，USB 导出的芯片录制为 µs（列名里写明） |
| `PC_Time(epoch_s)` | 电脑收到这一帧的时刻，可用于和其他设备（FSR、视频）对齐 |

## 常见问题

| 现象 | 处理 |
|---|---|
| 扫描不到 | 传感器开机、离电脑近一些；关掉手机上的 Movella DOT App；不要放在充电盒里 |
| 点 bat 闪退 / 提示缺 DLL | 安装微软 VC++ 运行库 <https://aka.ms/vs/17/release/vc_redist.x64.exe> |
| 「开始录制失败」 | 芯片满了：USB 导出后清空 |
| 浏览器没自动打开 | 手动打开 <http://127.0.0.1:8765/> |
| 双击了两次 bat | 没关系，第二次只会打开浏览器 |

## 文件夹结构

```
启动IMU采集.bat     一键启动
imu_app/
  server.py         网页后端（只用 Python 标准库）
  dot.py            Movella DOT SDK 封装：扫描/连接/设置/实时采集/芯片录制/USB 导出/清空
  web/              网页（index.html, app.js, style.css）
  settings.json     自动生成：上次的设置和传感器名字
runtime/python/     便携 Python 3.10 + movelladot_pc_sdk 2023.6 + numpy 1.26 + pyserial
data/               默认数据保存位置（首次采集时自动创建）
```

`runtime/` 的来源：python.org 的 `python-3.10.11-embed-amd64.zip`，解压后把
`movelladot_pc_sdk-2023.6.0-cp310-none-win_amd64.whl`（Movella DOT PC SDK 安装目录 `SDK Files\Python\x64\`）、
`numpy-1.26.4-cp310`、`pyserial-3.5` 三个 wheel 解压进 `runtime/python/Lib/site-packages/`，
并在 `python310._pth` 里加一行 `Lib\site-packages`。
