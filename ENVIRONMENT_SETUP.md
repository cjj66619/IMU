# 项目环境配置指南

本指南将帮助你配置 Python 开发环境，以便顺利运行 Movella DOT 传感器的数据采集和控制脚本。

## 1. 系统要求

*   **操作系统**: Windows 10 或 Windows 11 (需要支持蓝牙 4.0+ / BLE)
*   **Python 版本**: Python 3.7 或更高版本

## 2. 环境搭建步骤

建议在项目根目录下使用虚拟环境，以避免产生依赖冲突。

### 第一步：创建虚拟环境

在项目根目录 (`D:\Desktop\POF\IMU`) 打开终端 (PowerShell 或 CMD)，运行以下命令：

```bash
python -m venv venv
```

### 第二步：激活虚拟环境

*   **Windows (PowerShell)**:
    ```powershell
    .\venv\Scripts\Activate.ps1
    ```
    *(如果遇到权限错误，可先运行 `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`)*

*   **Windows (CMD)**:
    ```cmd
    venv\Scripts\activate.bat
    ```

激活成功后，命令行行首会出现 `(venv)` 标识。

### 第三步：安装依赖库

安装 `requirements.txt` 中列出的通用 Python 库（如 `bleak`, `numpy` 等）：

```bash
pip install -r requirements.txt
```

### 第四步：安装本项目 SDK

为了让脚本能够正确引用 `movella_dot_py` 包，需要以编辑模式安装当前目录的包：

```bash
pip install -e .
```
*注意：命令最后有一个点 `.`，代表当前目录。这一步至关重要，否则运行脚本时可能会报错 `ImportError`。*

## 3. 运行代码

### 数据采集脚本
主要的 IMU 数据采集脚本位于 `movella_dot_py/examples/IMU_data_collection.py`。

1.  **修改目标地址**：
    打开脚本，找到 `TARGET_ADDRESSES` 变量，确保里面的蓝牙 MAC 地址（例如 `"D4:22:CD:00:7F:2D"`）是你当前要连接的传感器地址。

2.  **运行脚本**：
    在终端中运行：
    ```bash
    python movella_dot_py/examples/IMU_data_collection.py
    ```

## 4. 常见问题排查

*   **找不到模块 `movella_dot_py`**:
    *   请确保你已经运行了 `pip install -e .`。
    *   或者确保你在项目根目录下运行脚本，并且环境变量 `PYTHONPATH` 包含了当前目录。

*   **蓝牙扫描不到设备**:
    *   检查传感器是否已开机（LED 闪烁）。
    *   检查电脑蓝牙是否已开启。
    *   如果之前连接过手机 APP，请确保已在手机上断开连接（蓝牙设备通常不支持同时连接两个主设备）。

*   **连接超时或失败**:
    *   蓝牙信号可能受干扰，尝试靠近电脑蓝牙接收器。
    *   传感器电量可能不足。

*   **WIN11系统下蓝牙连接异常**:
    *   WIN11下的蓝牙堆栈比较严格，对于未连接的设备比较苛刻
    *   打开设置 -> 蓝牙和其他设备 -> 添加设备，连接所有用到的Movella DOT


## 5. 目录结构说明

*   `movella_dot_py/`: SDK 核心源码
    *   `core/`: 传感器核心逻辑 (连接, 订阅特征值等)
    *   `examples/`: 示例脚本 (如数据采集)
*   `setup.py`: 包安装配置文件
*   `requirements.txt`: 依赖列表
