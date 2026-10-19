"""Movella DOT 控制核心（官方 movelladot_pc_sdk）。

线程模型：所有 SDK 调用只在 server 的 worker 线程里执行；SDK 回调线程只往
带锁的缓冲区里塞数据。网页通过 snapshot() 读状态。
"""
import csv
import glob
import json
import math
import os
import re
import threading
import time
from collections import deque
from datetime import datetime

import movelladot_pc_sdk as sdk

try:
    import winsound
except ImportError:  # 非 Windows：倒计时静音
    winsound = None

G_CONST = 9.7949          # 与实验室既有脚本一致的重力加速度 (m/s^2)
DEG2RAD = math.pi / 180.0
# sampleTimeFine 的单位随数据来源不同（均为实测确认）：
LIVE_TICKS_PER_S = 10000.0      # 蓝牙实时数据：0.1 ms（30 Hz 时每帧 +333）
EXPORT_TICKS_PER_S = 1000000.0  # USB 导出的芯片录制：1 µs（60 Hz 时每帧 +16667）
WRAP = 2 ** 32                  # uint32 回绕（实时约 4.97 天，导出约 71.6 分钟），已自动展开
_TICK_UNIT = {LIVE_TICKS_PER_S: "0.1ms", EXPORT_TICKS_PER_S: "us"}

LIVE_RATES = [1, 4, 10, 12, 15, 20, 30, 60]
RECORD_RATES = LIVE_RATES + [120]   # 120 Hz 仅板载录制可用
FILTERS = ["General", "Dynamic"]
PAYLOAD_MODES = [  # (SDK 名称后缀, 网页显示)
    ("CustomMode5", "四元数 + 加速度 + 角速度（推荐）"),
    ("CustomMode1", "欧拉角 + 自由加速度 + 角速度"),
    ("CustomMode2", "欧拉角 + 自由加速度 + 磁场"),
    ("CustomMode3", "四元数 + 角速度"),
    ("CustomMode4", "四元数 + 加速度 + 角速度 + 磁场（全量）"),
    ("RateQuantities", "加速度 + 角速度"),
    ("RateQuantitieswMag", "加速度 + 角速度 + 磁场"),
    ("CompleteQuaternion", "四元数 + 自由加速度"),
    ("CompleteEuler", "欧拉角 + 自由加速度"),
    ("OrientationQuaternion", "仅四元数"),
    ("OrientationEuler", "仅欧拉角"),
    ("FreeAcceleration", "仅自由加速度"),
]
PAYLOAD_KEYS = [k for k, _ in PAYLOAD_MODES]

# 导出板载录制时请求的字段
EXPORT_FIELDS = ["Timestamp", "Quaternion", "Acceleration", "AngularVelocity", "MagneticField"]

# (key, CSV 列, packet 判断函数, 取值函数)
_FIELDS = [
    ("quat", ["Quat_W", "Quat_X", "Quat_Y", "Quat_Z"], "containsOrientation",
     lambda p: _quat(p.orientationQuaternion())),
    ("euler", ["Roll(rad)", "Pitch(rad)", "Yaw(rad)"], "containsOrientation",
     lambda p: _euler_rad(p.orientationEuler())),
    ("acc", ["Acc_X(g)", "Acc_Y(g)", "Acc_Z(g)"], "containsCalibratedAcceleration",
     lambda p: tuple(x / G_CONST for x in _vec3(p.calibratedAcceleration()))),
    ("gyr", ["Gyr_X(rad/s)", "Gyr_Y(rad/s)", "Gyr_Z(rad/s)"], "containsCalibratedGyroscopeData",
     lambda p: tuple(x * DEG2RAD for x in _vec3(p.calibratedGyroscopeData()))),
    ("freeacc", ["FreeAcc_X(m/s2)", "FreeAcc_Y(m/s2)", "FreeAcc_Z(m/s2)"], "containsFreeAcceleration",
     lambda p: _vec3(p.freeAcceleration())),
    ("mag", ["Mag_X(a.u.)", "Mag_Y(a.u.)", "Mag_Z(a.u.)"], "containsCalibratedMagneticField",
     lambda p: _vec3(p.calibratedMagneticField())),
]
_PREVIEW = [  # 网页实时曲线显示哪一个量（按优先级）
    ("acc", "|加速度| (g)", lambda v: math.sqrt(sum(x * x for x in v))),
    ("freeacc", "|自由加速度| (m/s²)", lambda v: math.sqrt(sum(x * x for x in v))),
    ("gyr", "|角速度| (rad/s)", lambda v: math.sqrt(sum(x * x for x in v))),
    ("euler", "Roll (°)", lambda v: v[0] / DEG2RAD),
    ("quat", "Quat_W", lambda v: v[0]),
]


def _vec3(v):
    return float(v[0]), float(v[1]), float(v[2])


def _quat(q):
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])   # SDK 返回 numpy 数组 [w, x, y, z]


def _nonneg(n):
    return n if isinstance(n, int) and n >= 0 else None


def _euler_rad(e):
    return float(e.roll()) * DEG2RAD, float(e.pitch()) * DEG2RAD, float(e.yaw()) * DEG2RAD


def extract(packet):
    """XsDataPacket -> (sampleTimeFine, {field: tuple})。packet 会被 SDK 复用，必须当场取值。"""
    vals = {}
    for key, _cols, has, get in _FIELDS:
        try:
            if getattr(packet, has)():
                vals[key] = get(packet)
        except Exception:
            pass
    try:
        stf = int(packet.sampleTimeFine())
    except Exception:
        stf = None
    return stf, vals


def safe_name(s):
    s = re.sub(r'[\\/:*?"<>|\s]+', "_", str(s)).strip("._")
    return s or "IMU"


def _mac_of(dev):
    try:
        return str(dev.bluetoothAddress()).upper()
    except Exception:
        return ""


def _did_of(dev):
    try:
        return str(dev.deviceId().toXsString())
    except Exception:
        return str(id(dev))


def _try(fn, default=None):
    try:
        return fn()
    except Exception:
        return default


# --------------------------------------------------------------------------- CSV
class TrackWriter(object):
    """一个传感器一个 CSV。列在首个样本到达时按其实际包含的字段确定。"""

    def __init__(self, path, ticks_per_s=LIVE_TICKS_PER_S):
        self.path = path
        self.ticks = ticks_per_s
        self.f = None
        self.w = None
        self.keys = None
        self.n = 0
        self.first_stf = None
        self._last = None
        self._wrap = 0
        self.last_stf = None   # 已展开回绕

    def write(self, samples, t0):
        """samples: [(pc_time, stf_raw, vals)]；t0: 作为 0 秒的 sampleTimeFine（None=本传感器首帧）。"""
        for pc_t, stf, vals in samples:
            if stf is None:
                continue
            if self._last is not None and stf < self._last - WRAP // 2:
                self._wrap += WRAP
            self._last = stf
            stf_u = stf + self._wrap
            self.last_stf = stf_u
            if self.first_stf is None:
                self.first_stf = stf
            if self.w is None:
                self.keys = [k for k, _c, _h, _g in _FIELDS if k in vals]
                cols = ["Time(s)"]
                for k, c, _h, _g in _FIELDS:
                    if k in self.keys:
                        cols += c
                cols += ["SampleTimeFine(%s)" % _TICK_UNIT[self.ticks], "PC_Time(epoch_s)"]
                self.f = open(self.path, "w", newline="", encoding="utf-8")
                self.w = csv.writer(self.f)
                self.w.writerow(cols)
            base = self.first_stf if t0 is None else t0
            row = ["%.6f" % ((stf_u - base) / self.ticks)]
            for k in self.keys:
                v = vals.get(k)
                row += ["%.6f" % x for x in v] if v else [""] * (4 if k == "quat" else 3)
            row += [str(stf_u), "%.4f" % pc_t]
            self.w.writerow(row)
            self.n += 1

    def hz(self):
        """按传感器时钟算的实际采样率。"""
        if self.n < 2 or self.last_stf is None or self.last_stf <= self.first_stf:
            return 0
        return round((self.n - 1) / ((self.last_stf - self.first_stf) / self.ticks), 1)

    def close(self):
        if self.f:
            self.f.close()
            self.f = None


# --------------------------------------------------------------------------- callback
class _Callback(sdk.XsDotCallback):
    def __init__(self, ctl):
        sdk.XsDotCallback.__init__(self)
        self.ctl = ctl

    def onAdvertisementFound(self, port_info):
        self.ctl._on_adv(port_info)

    def onLiveDataAvailable(self, device, packet):
        self.ctl._on_sample(_mac_of(device), packet)

    def onRecordedDataAvailable(self, device, packet):
        self.ctl._on_export_sample(_did_of(device), packet)

    def onRecordedDataDone(self, device):
        self.ctl._on_export_done(_did_of(device))

    def onRecordingStopped(self, device):
        with self.ctl.lock:
            self.ctl._rec_stopped.add(_mac_of(device))

    def onBatteryUpdated(self, device, level, charging):
        with self.ctl.lock:
            d = self.ctl.devices.get(_mac_of(device))
            if d:
                d["battery"], d["charging"] = int(level), bool(charging)

    def onDeviceStateChanged(self, device, new_state, old_state):
        if new_state == sdk.XDS_Destructing:
            mac = _mac_of(device)
            with self.ctl.lock:
                d = self.ctl.devices.get(mac)
                if d:
                    d["lost"] = True
            self.ctl.log("⚠️ %s 断开连接（关机 / 超出范围 / 放回了充电盒）" % self.ctl.role_of(mac))

    def onError(self, result, error):
        self.ctl.log("[SDK 错误] %s: %s" % (_try(lambda: sdk.XsResultValueToString(result), result), error))


# --------------------------------------------------------------------------- controller
class DotController(object):
    def __init__(self, settings, log):
        self.settings = settings     # Settings 对象（roles / 输出目录等）
        self.log = log
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.phase = "idle"          # idle|scanning|connecting|ready|syncing|countdown|streaming|recording|saving|usb
        self.status = "未连接传感器"
        self.error = ""
        self.scanned = {}            # mac -> {"mac", "port"}
        self.devices = {}            # mac -> 信息 dict（含 "_dev" SDK 对象）
        self.usb = {}                # did -> 信息 dict（含 "_dev"、"_port"）
        self.capture = None          # 当前/上一次采集信息
        self.usb_job = None
        self._pending = {}           # mac -> [samples]（实时采集缓冲）
        self._preview = {}           # mac -> deque
        self._times = {}             # mac -> deque(pc_time)
        self._counts = {}
        self._rec_stopped = set()
        self._exp_pending = {}       # did -> [samples]
        self._exp_done = set()
        self._live_on = False
        self.cb = _Callback(self)
        self.manager = self._new_manager()

    # ------------------------------------------------------------- helpers
    def _new_manager(self):
        m = sdk.XsDotConnectionManager()
        if m is None:
            raise RuntimeError("无法创建 XsDotConnectionManager（SDK 初始化失败）")
        m.addXsDotCallbackHandler(self.cb)
        return m

    def role_of(self, mac):
        return self.settings.data["roles"].get(mac) or mac.replace(":", "")[-4:]

    def _set(self, phase=None, status=None, error=None):
        with self.lock:
            if phase is not None:
                self.phase = phase
            if status is not None:
                self.status = status
            if error is not None:
                self.error = error

    def _live_devs(self):
        return [d for d in self.devices.values() if not d.get("lost")]

    # ------------------------------------------------------------- SDK callbacks
    def _on_adv(self, port_info):
        mac = str(port_info.bluetoothAddress()).upper()
        with self.lock:
            if mac not in self.scanned:
                self.scanned[mac] = {"mac": mac, "port": port_info}

    def _on_sample(self, mac, packet):
        if not self._live_on:
            return
        stf, vals = extract(packet)
        now = time.time()
        with self.lock:
            self._pending.setdefault(mac, []).append((now, stf, vals))
            n = self._counts[mac] = self._counts.get(mac, 0) + 1
            self._times.setdefault(mac, deque(maxlen=400)).append(now)
            if stf is not None:
                for key, _label, fn in _PREVIEW:
                    if key in vals:
                        # (序号, 传感器时间 s, 值)：网页按传感器时间平滑回放
                        self._preview.setdefault(mac, deque(maxlen=900)).append(
                            (n, round(stf / LIVE_TICKS_PER_S, 4), round(fn(vals[key]), 4)))
                        break

    def _on_export_sample(self, did, packet):
        stf, vals = extract(packet)
        with self.lock:
            self._exp_pending.setdefault(did, []).append((time.time(), stf, vals))

    def _on_export_done(self, did):
        with self.lock:
            self._exp_done.add(did)

    # ------------------------------------------------------------- snapshot
    def snapshot(self):
        with self.lock:
            devs = []
            now = time.time()
            for mac, d in self.devices.items():
                info = {k: v for k, v in d.items() if not k.startswith("_")}
                info["role"] = self.role_of(mac)
                ts = self._times.get(mac)
                if ts and len(ts) > 1 and now - ts[-1] < 2.0:
                    recent = [t for t in ts if now - t <= 3.0]
                    info["hz"] = round((len(recent) - 1) / (recent[-1] - recent[0]), 1) if len(recent) > 1 and recent[-1] > recent[0] else None
                else:
                    info["hz"] = None
                info["count"] = self._counts.get(mac, 0)
                devs.append(info)
            devs.sort(key=lambda x: x["role"])
            scanned = sorted(({"mac": m, "role": self.role_of(m), "connected": m in self.devices}
                              for m in self.scanned), key=lambda x: x["mac"])
            usb = sorted(({k: v for k, v in u.items() if not k.startswith("_")} | {"role": self.role_of(u["mac"])}
                          for u in self.usb.values()), key=lambda x: x["role"])
            return {
                "phase": self.phase, "status": self.status, "error": self.error,
                "scanned": scanned, "devices": devs, "usb": usb,
                "capture": dict(self.capture) if self.capture else None,
                "usb_job": dict(self.usb_job) if self.usb_job else None,
                "preview_label": next((lbl for k, lbl, _f in _PREVIEW
                                       if self.capture and k in self.capture.get("fields", [])), None),
            }

    def live_since(self, since):
        """实时曲线增量：since = {mac: 已收到的最大序号}，只返回之后的新点。"""
        out = {}
        with self.lock:
            for mac, dq in self._preview.items():
                last = int(since.get(mac, 0) or 0)
                pts = [p for p in dq if p[0] > last] if dq and dq[-1][0] > last else []
                out[mac] = {"role": self.role_of(mac), "count": self._counts.get(mac, 0), "pts": pts}
            label = next((lbl for k, lbl, _f in _PREVIEW
                          if self.capture and k in self.capture.get("fields", [])), None)
            session = self.capture.get("start") if self.capture else None
        return {"label": label, "session": session, "devices": out}

    # ------------------------------------------------------------- scan / connect
    def scan(self, seconds=6.0):
        self._set("scanning", "正在扫描蓝牙… 请确认传感器已开机（LED 闪烁）", "")
        with self.lock:
            self.scanned = {m: v for m, v in self.scanned.items() if m in self.devices}
        self.manager.enableDeviceDetection()
        t0 = time.time()
        try:
            while time.time() - t0 < seconds:
                time.sleep(0.2)
        finally:
            self.manager.disableDeviceDetection()
        n = len([m for m in self.scanned if m not in self.devices])
        self.log("扫描完成：发现 %d 个未连接的传感器" % n)
        self._set("ready" if self.devices else "idle",
                  ("发现 %d 个传感器，勾选后点「连接」" % n) if n else "没扫到传感器：确认已开机、不在充电盒里、没被手机 App 连着")

    def connect(self, macs):
        self._set("connecting", "正在连接…", "")
        ok = 0
        for mac in macs:
            mac = mac.upper()
            if mac in self.devices and not self.devices[mac].get("lost"):
                continue
            entry = self.scanned.get(mac)
            if not entry:
                self.log("❌ %s 不在扫描结果里，请重新扫描" % mac)
                continue
            self._set(status="正在连接 %s …" % self.role_of(mac))
            port = entry["port"]
            if not (self.manager.openPort(port) or self.manager.openPort(port)):
                self.log("❌ 连接 %s 失败：%s" % (self.role_of(mac), self.manager.lastResultText()))
                continue
            dev = self.manager.device(port.deviceId())
            if dev is None:
                self.log("❌ %s：拿不到设备句柄" % self.role_of(mac))
                continue
            with self.lock:
                self.devices[mac] = {"_dev": dev, "mac": mac, "lost": False, "charging": None}
            self.refresh_device(mac)
            ok += 1
            self.log("✅ 已连接 %s (%s)" % (self.role_of(mac), mac))
        if ok:
            self.apply_settings()
        self._set("ready" if self.devices else "idle",
                  "已连接 %d 个传感器" % len(self._live_devs()) if self.devices else "未连接传感器")

    def refresh_device(self, mac):
        d = self.devices.get(mac)
        if not d:
            return
        dev = d["_dev"]
        info = {
            "battery": _try(dev.batteryLevel),
            "firmware": _try(lambda: str(dev.firmwareVersion().toXsString())),
            "product": _try(lambda: str(dev.productCode())),
            "tag": _try(lambda: str(dev.deviceTagName())),
            "rate": _try(dev.outputRate),
            "filter": _try(lambda: str(dev.onboardFilterProfile().label())),
            "recordings": _nonneg(_try(dev.recordingCount)),   # 蓝牙连接时 SDK 常返回 -1（未知）
        }
        with self.lock:
            d.update(info)

    def disconnect(self):
        self._set("connecting", "正在断开…")
        try:
            self.manager.close()
        except Exception:
            pass
        with self.lock:
            self.devices, self.scanned, self.usb = {}, {}, {}
            self._preview, self._times, self._counts = {}, {}, {}
        self.manager = self._new_manager()
        self._set("idle", "已断开全部传感器", "")

    def identify(self, mac):
        d = self.devices.get(mac.upper())
        if d and d["_dev"].identify():
            self.log("💡 %s 的灯正在快速闪烁" % self.role_of(mac.upper()))

    def apply_settings(self):
        """把采样率 / 滤波模式写入所有已连接传感器，并读回确认。"""
        s = self.settings.data
        rate = int(s["rate"])
        if s["mode"] == "live" and rate not in LIVE_RATES:
            rate = 60
        bad = []
        for d in self._live_devs():
            dev = d["_dev"]
            ok1 = dev.setOnboardFilterProfile(s["filter"])
            ok2 = dev.setOutputRate(rate)
            self.refresh_device(d["mac"])
            if not (ok1 and ok2):
                bad.append("%s(%s)" % (self.role_of(d["mac"]), dev.lastResultText()))
        if bad:
            self.log("⚠️ 设置未全部生效：" + "，".join(bad))
        elif self._live_devs():
            self.log("⚙️ 已设置：%d Hz，滤波 %s" % (rate, s["filter"]))

    # ------------------------------------------------------------- capture
    def _countdown(self, n):
        for i in range(int(n), 0, -1):
            if self.stop_event.is_set():
                return False
            self._set("countdown", "倒计时 %d …" % i)
            if winsound:
                _try(lambda: winsound.Beep(700, 150))
            time.sleep(0.85)
        if winsound:
            _try(lambda: winsound.Beep(1200, 400))
        return not self.stop_event.is_set()

    def _session_dir(self, out_dir, stamp, tag):
        name = "IMU_%s%s" % (stamp, ("_" + safe_name(tag)) if tag else "")
        path = os.path.join(out_dir, name)
        os.makedirs(path, exist_ok=True)
        return path

    def start_capture(self, out_dir):
        s = dict(self.settings.data)
        devs = self._live_devs()
        if not devs:
            raise RuntimeError("没有已连接的传感器")
        self.stop_event.clear()
        with self.lock:
            self.capture = None
        self.apply_settings()   # 保证用的是网页上当前的采样率 / 滤波
        try:
            if s["mode"] == "live":
                self._run_live(s, devs, out_dir)
            else:
                self._run_onboard(s, devs, out_dir)
        finally:
            done = bool(self.capture and self.capture.get("done"))
            self._set("ready" if self.devices else "idle",
                      "采集完成，可以开始下一次" if done else "已连接 %d 个传感器" % len(self._live_devs()))

    def _run_live(self, s, devs, out_dir):
        mode_attr = "XsPayloadMode_" + s["payload"]
        payload = getattr(sdk, mode_attr)
        duration = int(s["duration"])
        synced = False
        root = devs[-1]["mac"]
        if s.get("sync") and len(devs) >= 2:
            self._set("syncing", "多传感器硬件同步中（约 15 秒），请保持传感器静止、靠近电脑…")
            if not self.manager.startSync(root):
                self.log("首次同步失败：%s，重试…" % self.manager.lastResultText())
                self.manager.stopSync()
                if not self.manager.startSync(root):
                    raise RuntimeError("硬件同步失败：%s（可取消勾选「硬件同步」后再试）" % self.manager.lastResultText())
            synced = True
            self.log("🔗 硬件同步完成（root = %s）" % self.role_of(root))
        try:
            if not self._countdown(s["countdown"]):
                self.log("已取消")
                return
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            sdir = self._session_dir(out_dir, stamp, s.get("tag"))
            writers = {d["mac"]: TrackWriter(os.path.join(sdir, "%s_%s.csv" % (safe_name(self.role_of(d["mac"])), stamp)))
                       for d in devs}
            with self.lock:
                self._pending = {}
                self._preview, self._times, self._counts = {}, {}, {}
                self.capture = {"mode": "live", "dir": sdir, "start": time.time(), "duration": duration,
                                "elapsed": 0, "fields": [], "done": False}
            self._live_on = True
            order = [d for d in devs if d["mac"] != root] + [d for d in devs if d["mac"] == root]
            started = []
            for d in order:
                if d["_dev"].startMeasurement(payload):
                    started.append(d)
                else:
                    self.log("❌ %s 启动测量失败：%s" % (self.role_of(d["mac"]), d["_dev"].lastResultText()))
            if not started:
                raise RuntimeError("所有传感器都没能开始测量")
            self._set("streaming", "采集中…")
            self.log("🚀 开始实时采集（%s, %s Hz）→ %s" % (s["payload"], s["rate"], sdir))
            t_start = time.time()
            t0_global, t0_deadline = None, t_start + 3.0
            try:
                while True:
                    time.sleep(0.2)
                    el = time.time() - t_start
                    with self.lock:
                        self.capture["elapsed"] = round(el, 1)
                        batches = self._pending
                        self._pending = {}
                    if synced and t0_global is None:
                        firsts = [b[0][1] for b in batches.values() if b and b[0][1] is not None]
                        firsts += [w.first_stf for w in writers.values() if w.first_stf is not None]
                        have = set(m for m, b in batches.items() if b) | set(m for m, w in writers.items() if w.n)
                        if firsts and (len(have) >= len(started) or time.time() > t0_deadline):
                            t0_global = min(firsts)
                        else:  # 还没决定 0 点：放回去等
                            with self.lock:
                                for m, b in batches.items():
                                    self._pending[m] = b + self._pending.get(m, [])
                            batches = {}
                    for mac, b in batches.items():
                        if mac in writers:
                            writers[mac].write(b, t0_global)
                    if not self.capture["fields"]:
                        ks = [w.keys for w in writers.values() if w.keys]
                        if ks:
                            with self.lock:
                                self.capture["fields"] = ks[0]
                    if self.stop_event.is_set() or (duration > 0 and el >= duration):
                        break
            finally:
                self._set("saving", "正在停止并保存…")
                for d in started:
                    if not d["_dev"].stopMeasurement():
                        self.log("⚠️ %s 停止测量失败：%s" % (self.role_of(d["mac"]), d["_dev"].lastResultText()))
                time.sleep(0.3)
                self._live_on = False
                with self.lock:
                    batches, self._pending = self._pending, {}
                if synced and t0_global is None:
                    firsts = [b[0][1] for b in batches.values() if b and b[0][1] is not None]
                    t0_global = min(firsts) if firsts else None
                for mac, b in batches.items():
                    if mac in writers:
                        writers[mac].write(b, t0_global)
                for w in writers.values():
                    w.close()
            el = time.time() - t_start
            summary = []
            for d in devs:
                w = writers[d["mac"]]
                summary.append({"role": self.role_of(d["mac"]), "mac": d["mac"], "rows": w.n,
                                "hz": w.hz(),
                                "file": os.path.basename(w.path) if w.n else None})
                if not w.n:
                    self.log("❌ %s 没收到数据" % self.role_of(d["mac"]))
            self._write_info(sdir, s, devs, synced, t_start, el, summary)
            with self.lock:
                self.capture.update(done=True, elapsed=round(el, 1), summary=summary)
            self.log("✅ 已保存 %s" % sdir)
        finally:
            self._live_on = False
            if synced:
                if not self.manager.stopSync():
                    self.log("⚠️ 停止同步失败：%s" % self.manager.lastResultText())

    def _run_onboard(self, s, devs, out_dir):
        duration = int(s["duration"])
        if not self._countdown(s["countdown"]):
            self.log("已取消")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        sdir = self._session_dir(out_dir, stamp, s.get("tag"))
        with self.lock:
            self._rec_stopped = set()
            self.capture = {"mode": "onboard", "dir": sdir, "start": time.time(), "duration": duration,
                            "elapsed": 0, "fields": [], "done": False}
        started = []
        t_start = time.time()
        for d in devs:
            dev = d["_dev"]
            ok = dev.startTimedRecording(duration) if duration > 0 else dev.startRecording()
            if ok:
                started.append(d)
            else:
                self.log("❌ %s 开始录制失败：%s（芯片可能已满，请 USB 导出后清空）"
                         % (self.role_of(d["mac"]), dev.lastResultText()))
        if not started:
            raise RuntimeError("所有传感器都没能开始板载录制")
        self._set("recording", "板载录制中（数据存在传感器芯片里）…")
        self.log("⏺️ 开始板载录制（%s Hz）" % s["rate"])
        try:
            while True:
                time.sleep(0.25)
                el = time.time() - t_start
                with self.lock:
                    self.capture["elapsed"] = round(el, 1)
                    stopped = set(self._rec_stopped)
                if all(d["mac"] in stopped for d in started):
                    break
                if self.stop_event.is_set():
                    break
                if duration > 0 and el >= duration + 8:
                    break
        finally:
            self._set("saving", "正在结束录制…")
            for d in started:
                if d["mac"] not in self._rec_stopped:
                    d["_dev"].stopRecording()
            t1 = time.time()
            while time.time() - t1 < 5 and not all(d["mac"] in self._rec_stopped for d in started):
                time.sleep(0.2)
        el = time.time() - t_start
        summary = []
        for d in started:
            self.refresh_device(d["mac"])
            summary.append({"role": self.role_of(d["mac"]), "mac": d["mac"],
                            "recording_index": self.devices[d["mac"]].get("recordings")})
        self._write_info(sdir, s, started, False, t_start, el, summary)
        with self.lock:
            self.capture.update(done=True, elapsed=round(el, 1), summary=summary)
        self.log("✅ 板载录制结束。把传感器放回充电盒接 USB，在「芯片数据」里导出即可（会自动放进 %s）" % sdir)

    def _write_info(self, sdir, s, devs, synced, t_start, el, summary):
        info = {
            "mode": s["mode"], "output_rate_hz": int(s["rate"]), "filter": s["filter"],
            "payload": s["payload"] if s["mode"] == "live" else None, "hardware_sync": synced,
            "start_epoch": round(t_start, 3), "start_local": datetime.fromtimestamp(t_start).isoformat(timespec="seconds"),
            "duration_s": round(el, 2), "tag": s.get("tag", ""),
            "sensors": {d["mac"]: self.role_of(d["mac"]) for d in devs},
            "summary": summary, "g_const": G_CONST,
            "time_note": "Time(s) 以%s为 0；SampleTimeFine 为传感器原始时钟（实时数据单位 0.1 ms，USB 导出数据单位 µs）%s；PC_Time 为电脑收到数据的时刻"
                         % ("所有传感器最早一帧" if synced else "各传感器自己的首帧",
                            "（已硬件同步，可直接跨传感器对齐）" if synced else ""),
        }
        with open(os.path.join(sdir, "session_info.json"), "w", encoding="utf-8") as f:
            json.dump(info, f, ensure_ascii=False, indent=2)

    # ------------------------------------------------------------- USB（充电盒）
    _USB_BAUDS = (921600, 2000000, 460800, 230400, 115200)

    def _usb_ports(self):
        """充电盒/数据线上的 DOT = FTDI 0403:6015 虚拟 COM 口。

        不用 SDK 的 detectUsbDevices()：它会探测所有 COM 口，在有蓝牙串口（COM3/COM4
        “蓝牙链接上的标准串行”）的电脑上实测 50 s 以上不返回，「查找 USB」因此卡死。
        """
        import serial.tools.list_ports as lp
        return sorted(p.device for p in lp.comports()
                      if "0403:6015" in (p.hwid or "").upper() or "VID_0403&PID_6015" in (p.hwid or "").upper())

    def usb_scan(self):
        self._set("usb", "正在查找 USB / 充电盒里的传感器…", "")
        try:
            for u in self.usb.values():
                _try(lambda: self.manager.closePort(u["_port"]))
            with self.lock:
                self.usb = {}
            for com in self._usb_ports():
                pi = next((p for p in (sdk.XsPortInfo(com, b) for b in self._USB_BAUDS)
                           if self.manager.openPort(p)), None)
                if pi is None:
                    self.log("❌ USB %s 打开失败：%s" % (com, self.manager.lastResultText()))
                    continue
                dev = self.manager.usbDevice(pi.deviceId())
                if dev is None:
                    _try(lambda: self.manager.closePort(pi))
                    continue
                did = _did_of(dev)
                if did in self.usb:
                    _try(lambda: self.manager.closePort(pi))
                    continue
                with self.lock:
                    self.usb[did] = {"_dev": dev, "_port": pi, "did": did, "mac": _mac_of(dev)}
                self._usb_refresh(did)
            n = len(self.usb)
            self.log("USB：找到 %d 个传感器" % n)
            self._set(status=("USB 找到 %d 个传感器" % n) if n else
                      "USB 没找到传感器：把传感器放进充电盒、盒子插电脑，或用数据线直连")
        finally:
            self._set("ready" if self.devices else "idle")

    def _usb_refresh(self, did):
        u = self.usb[did]
        dev = u["_dev"]
        n = _nonneg(_try(dev.recordingCount, 0)) or 0
        recs = []
        for i in range(1, n + 1):
            ri = _try(lambda: dev.getRecordingInfo(i))
            if ri is None or _try(ri.empty, True):
                continue
            utc = _try(ri.startUTC, 0) or 0
            recs.append({"index": i, "start": datetime.fromtimestamp(utc).strftime("%m-%d %H:%M:%S") if utc else "?",
                         "start_utc": utc, "seconds": _try(ri.totalRecordingTime), "bytes": _try(ri.storageSize)})
        with self.lock:
            u.update(recordings=n, recs=recs, used=_try(dev.usedFlashSizeTotal),
                     firmware=_try(lambda: str(dev.firmwareVersion().toXsString())))

    def _match_session(self, out_dir, mac, start_utc):
        """把板载录制按开始时间放回网页里录制时建的会话文件夹。"""
        best = None
        for p in glob.glob(os.path.join(out_dir, "IMU_*", "session_info.json")):
            try:
                info = json.load(open(p, encoding="utf-8"))
            except Exception:
                continue
            if info.get("mode") != "onboard" or mac not in info.get("sensors", {}):
                continue
            dt = abs(info.get("start_epoch", 0) - start_utc)
            if dt <= 15 and (best is None or dt < best[0]):
                best = (dt, os.path.dirname(p))
        return best[1] if best else None

    def usb_export(self, out_dir, dids=None):
        targets = [d for d in self.usb if dids is None or d in dids]
        if not targets:
            raise RuntimeError("没有可导出的 USB 传感器，先点「查找 USB 传感器」")
        self.stop_event.clear()
        fields = sdk.XsIntArray()
        for name in EXPORT_FIELDS:
            fields.push_back(getattr(sdk, "RecordingData_" + name))
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fallback_dir = os.path.join(out_dir, "IMU_USB导出_%s" % stamp)
        files, total = [], sum(len(self.usb[d].get("recs", [])) for d in targets)
        with self.lock:
            self.usb_job = {"kind": "export", "done": 0, "total": total, "current": "", "packets": 0,
                            "files": files, "finished": False}
        self._set("usb", "正在导出芯片数据…", "")
        try:
            for did in targets:
                u = self.usb[did]
                dev, role = u["_dev"], self.role_of(u["mac"])
                for rec in u.get("recs", []):
                    if self.stop_event.is_set():
                        raise RuntimeError("导出已被中止")
                    idx = rec["index"]
                    sdir = self._match_session(out_dir, u["mac"], rec["start_utc"]) or fallback_dir
                    os.makedirs(sdir, exist_ok=True)
                    tstr = datetime.fromtimestamp(rec["start_utc"]).strftime("%Y%m%d_%H%M%S") if rec["start_utc"] else stamp
                    path = os.path.join(sdir, "%s_%s_rec%02d.csv" % (safe_name(role), tstr, idx))
                    with self.lock:
                        self.usb_job.update(current="%s 第 %d 段" % (role, idx), packets=0)
                        self._exp_pending[did] = []
                        self._exp_done.discard(did)
                    if not dev.selectExportData(fields):
                        self.log("❌ %s 选择导出字段失败：%s" % (role, dev.lastResultText()))
                        continue
                    if not dev.startExportRecording(idx):
                        self.log("❌ %s 第 %d 段导出失败：%s" % (role, idx, dev.lastResultText()))
                        continue
                    w = TrackWriter(path, EXPORT_TICKS_PER_S)
                    last_progress = time.time()
                    try:
                        while True:
                            time.sleep(0.3)
                            with self.lock:
                                batch = self._exp_pending.get(did, [])
                                self._exp_pending[did] = []
                                finished = did in self._exp_done
                            if batch:
                                w.write(batch, None)
                                last_progress = time.time()
                                with self.lock:
                                    self.usb_job["packets"] = w.n
                            if finished or self.stop_event.is_set():
                                break
                            if time.time() - last_progress > 20:
                                self.log("⚠️ %s 第 %d 段 20 秒没有新数据，停止该段" % (role, idx))
                                break
                    finally:
                        _try(dev.stopExportRecording)
                        with self.lock:
                            rest = self._exp_pending.pop(did, [])
                        w.write(rest, None)
                        w.close()
                    files.append({"role": role, "index": idx, "rows": w.n, "path": path})
                    self.log("💾 %s 第 %d 段：%d 行 → %s" % (role, idx, w.n, path))
                    with self.lock:
                        self.usb_job["done"] += 1
            self._set(status="导出完成：%d 个文件" % len(files))
        finally:
            with self.lock:
                self.usb_job["finished"] = True
            self._set("ready" if self.devices else "idle")

    def usb_erase(self, dids):
        targets = [d for d in dids if d in self.usb]
        if not targets:
            raise RuntimeError("没有选中要清空的传感器")
        self._set("usb", "正在清空芯片（每个约 10~60 秒，请勿拔出）…", "")
        try:
            for did in targets:
                u = self.usb[did]
                role = self.role_of(u["mac"])
                t0 = time.time()
                if u["_dev"].eraseFlash():
                    self.log("🧹 %s 已清空（%.1f 秒）" % (role, time.time() - t0))
                else:
                    self.log("❌ %s 清空失败：%s" % (role, u["_dev"].lastResultText()))
                self._usb_refresh(did)
            self._set(status="清空完成")
        finally:
            self._set("ready" if self.devices else "idle")

    def close(self):
        _try(self.manager.close)
