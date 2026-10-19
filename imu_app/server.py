"""IMU 采集网页后端：只用 Python 标准库 + movelladot_pc_sdk。

双击 启动IMU采集.bat 即可；浏览器会自动打开 http://127.0.0.1:8765
"""
import json
import os
import queue
import subprocess
import sys
import threading
import time
import traceback
import urllib.request
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

APP_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(APP_DIR)
WEB_DIR = os.path.join(APP_DIR, "web")
SETTINGS_PATH = os.path.join(APP_DIR, "settings.json")
HOST, BASE_PORT = "127.0.0.1", 8765
APP_ID = "imu-collector"

if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)

# ----------------------------------------------------------------- log
LOG = deque(maxlen=300)
_log_lock = threading.Lock()


def log(msg):
    line = "%s  %s" % (time.strftime("%H:%M:%S"), msg)
    with _log_lock:
        LOG.append(line)
    try:
        print(line, flush=True)
    except Exception:
        pass


try:
    import dot
except Exception as e:  # SDK 缺失 / DLL 加载失败
    print("\n无法加载 Movella DOT SDK：%s\n" % e)
    print("请确认 runtime 文件夹完整；若提示缺少 DLL，安装微软 VC++ 运行库："
          "https://aka.ms/vs/17/release/vc_redist.x64.exe\n")
    sys.exit(1)

# ----------------------------------------------------------------- settings
DEFAULTS = {
    "mode": "live",            # live = 蓝牙实时传输到电脑；onboard = 存在传感器芯片里
    "rate": 60,
    "filter": "General",
    "payload": "CustomMode5",
    "duration": 30,            # 秒；0 = 手动点停止
    "countdown": 3,
    "sync": True,
    "tag": "",
    "output_dir": "data",      # 相对路径以工作区根目录为基准，方便整个文件夹拷到别的电脑
    "roles": {},               # MAC -> 名字（如 Left_Shank）
}


class Settings(object):
    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()
        self.data = dict(DEFAULTS)
        try:
            with open(path, encoding="utf-8") as f:
                saved = json.load(f)
            self.update(saved)
        except FileNotFoundError:
            pass
        except Exception as e:
            log("⚠️ settings.json 读取失败，使用默认设置：%s" % e)

    def update(self, patch):
        """校验并合并；返回发生变化的键。"""
        d = dict(self.data)
        if patch.get("mode") in ("live", "onboard"):
            d["mode"] = patch["mode"]
        if "rate" in patch:
            try:
                r = int(patch["rate"])
                if r in dot.RECORD_RATES:
                    d["rate"] = r
            except (TypeError, ValueError):
                pass
        if d["mode"] == "live" and d["rate"] not in dot.LIVE_RATES:
            d["rate"] = 60
        if patch.get("filter") in dot.FILTERS:
            d["filter"] = patch["filter"]
        if patch.get("payload") in dot.PAYLOAD_KEYS:
            d["payload"] = patch["payload"]
        for key, lo, hi in (("duration", 0, 36000), ("countdown", 0, 10)):
            if key in patch:
                try:
                    d[key] = max(lo, min(hi, int(patch[key])))
                except (TypeError, ValueError):
                    pass
        if "sync" in patch:
            d["sync"] = bool(patch["sync"])
        if "tag" in patch:
            d["tag"] = str(patch["tag"] or "")[:40]
        if patch.get("output_dir"):
            d["output_dir"] = str(patch["output_dir"]).strip()
        if isinstance(patch.get("roles"), dict):
            roles = dict(d["roles"])
            for mac, name in patch["roles"].items():
                name = dot.safe_name(name) if str(name or "").strip() else ""
                if name:
                    roles[str(mac).upper()] = name
                else:
                    roles.pop(str(mac).upper(), None)
            d["roles"] = roles
        with self.lock:
            changed = [k for k in d if d[k] != self.data.get(k)]
            self.data = d
            if changed:
                tmp = self.path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(d, f, ensure_ascii=False, indent=2)
                os.replace(tmp, self.path)
        return changed

    def out_dir(self):
        p = os.path.expandvars(os.path.expanduser(self.data["output_dir"]))
        return os.path.normpath(p if os.path.isabs(p) else os.path.join(ROOT, p))


SETTINGS = Settings(SETTINGS_PATH)
CTL = dot.DotController(SETTINGS, log)

# ----------------------------------------------------------------- worker
JOBS = queue.Queue()
_busy = threading.Event()


def submit(name, fn, *args):
    if _busy.is_set():
        return False
    _busy.set()
    JOBS.put((name, fn, args))
    return True


def _worker():
    while True:
        name, fn, args = JOBS.get()
        try:
            fn(*args)
        except Exception as e:
            traceback.print_exc()
            log("❌ %s 出错：%s" % (name, e))
            CTL._set("ready" if CTL.devices else "idle", "出错了，见下方日志", str(e))
        finally:
            _busy.clear()


def _apply_settings():
    if CTL.devices and CTL.phase == "ready":
        CTL._set("connecting", "正在写入设置…")
        try:
            CTL.apply_settings()
        finally:
            CTL._set("ready")


# ----------------------------------------------------------------- folder dialogs
_PICK_PS = r"""
Add-Type -AssemblyName System.Windows.Forms
[Console]::OutputEncoding = [Text.Encoding]::UTF8
$owner = New-Object System.Windows.Forms.Form -Property @{TopMost = $true; ShowInTaskbar = $false}
$d = New-Object System.Windows.Forms.FolderBrowserDialog
$d.Description = '选择 IMU 数据保存位置'
$d.ShowNewFolderButton = $true
$d.SelectedPath = $env:IMU_START_DIR
if ($d.ShowDialog($owner) -eq 'OK') { Write-Output $d.SelectedPath }
"""


def pick_folder(start):
    env = dict(os.environ, IMU_START_DIR=start)
    r = subprocess.run(["powershell", "-NoProfile", "-STA", "-ExecutionPolicy", "Bypass", "-Command", _PICK_PS],
                       capture_output=True, env=env, timeout=600,
                       creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    out = r.stdout.decode("utf-8", "replace").strip().splitlines()
    return out[-1].strip() if out else None


def relativize(path):
    """工作区内部的路径存成相对路径，整个文件夹拷走后依然有效。"""
    try:
        rel = os.path.relpath(path, ROOT)
        if not rel.startswith(".."):
            return rel.replace("\\", "/")
    except ValueError:  # 不同盘符
        pass
    return path


def _win32():
    import ctypes
    from ctypes import wintypes as w
    u, k = ctypes.windll.user32, ctypes.windll.kernel32
    u.GetForegroundWindow.restype = w.HWND
    u.GetWindowThreadProcessId.argtypes = [w.HWND, ctypes.c_void_p]
    u.GetWindowThreadProcessId.restype = w.DWORD
    u.AttachThreadInput.argtypes = [w.DWORD, w.DWORD, w.BOOL]
    for fn in (u.SetForegroundWindow, u.BringWindowToTop, u.IsIconic):
        fn.argtypes = [w.HWND]
    u.ShowWindow.argtypes = [w.HWND, ctypes.c_int]
    u.SetWindowPos.argtypes = [w.HWND, w.HWND, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, w.UINT]
    u.GetClassNameW.argtypes = u.GetWindowTextW.argtypes = [w.HWND, w.LPWSTR, ctypes.c_int]
    return ctypes, w, u, k


def _explorer_windows():
    ctypes, w, u, _k = _win32()
    found, cls, title = [], ctypes.create_unicode_buffer(64), ctypes.create_unicode_buffer(1024)

    @ctypes.WINFUNCTYPE(w.BOOL, w.HWND, w.LPARAM)
    def cb(hwnd, _lp):
        u.GetClassNameW(hwnd, cls, 64)
        if cls.value == "CabinetWClass":
            u.GetWindowTextW(hwnd, title, 1024)
            found.append((hwnd, title.value))
        return True

    u.EnumWindows(cb, 0)
    return found


def _bring_to_front(hwnd):
    """后台进程默认抢不到前台（Windows 前台锁）：借用当前前台线程的输入队列再激活。"""
    _c, _w, u, k = _win32()
    if u.IsIconic(hwnd):
        u.ShowWindow(hwnd, 9)  # SW_RESTORE
    fg_tid = u.GetWindowThreadProcessId(u.GetForegroundWindow(), None)
    me = k.GetCurrentThreadId()
    attached = bool(fg_tid and fg_tid != me and u.AttachThreadInput(me, fg_tid, True))
    try:
        u.BringWindowToTop(hwnd)
        u.SetForegroundWindow(hwnd)
    finally:
        if attached:
            u.AttachThreadInput(me, fg_tid, False)
    if u.GetForegroundWindow() != hwnd:  # 兜底：至少把窗口放到最上层
        flags = 0x0001 | 0x0002 | 0x0040  # SWP_NOSIZE | SWP_NOMOVE | SWP_SHOWWINDOW
        u.SetWindowPos(hwnd, -1, 0, 0, 0, 0, flags)   # HWND_TOPMOST
        u.SetWindowPos(hwnd, -2, 0, 0, 0, 0, flags)   # HWND_NOTOPMOST


def open_folder(path):
    """在资源管理器里打开文件夹，并让窗口直接弹到最前面。"""
    path = os.path.normpath(path)
    if os.name != "nt":
        os.startfile(path)
        return
    before = {h for h, _t in _explorer_windows()}
    subprocess.Popen(["explorer.exe", path])
    name = os.path.basename(path).lower()
    titles = (name, path.lower())   # 标题形如 "data" 或 "data - 文件资源管理器" / "data - File Explorer"
    t0 = time.time()
    while time.time() - t0 < 4:
        time.sleep(0.15)
        wins = _explorer_windows()
        new = [h for h, _t in wins if h not in before]
        if new:
            _bring_to_front(new[0])
            return
        if time.time() - t0 > 1.0:  # 同一文件夹已开着时资源管理器复用旧窗口：按标题找
            same = [h for h, t in wins if t.lower() in titles or t.lower().startswith(name + " - ")]
            if same:
                _bring_to_front(same[0])
                return



# ----------------------------------------------------------------- HTTP
def state():
    snap = CTL.snapshot()
    with _log_lock:
        snap["log"] = list(LOG)[-80:]
    snap["busy"] = _busy.is_set()
    snap["settings"] = dict(SETTINGS.data)
    snap["out_dir"] = SETTINGS.out_dir()
    snap["options"] = {
        "live_rates": dot.LIVE_RATES, "record_rates": dot.RECORD_RATES,
        "filters": dot.FILTERS, "payloads": dot.PAYLOAD_MODES,
    }
    return snap


_STATIC = {"/": ("index.html", "text/html"), "/app.js": ("app.js", "application/javascript"),
           "/style.css": ("style.css", "text/css")}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, obj=None, body=None, ctype="application/json"):
        if body is None:
            body = json.dumps(obj if obj is not None else {}, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path in _STATIC:
            name, ctype = _STATIC[path]
            with open(os.path.join(WEB_DIR, name), "rb") as f:
                self._send(200, body=f.read(), ctype=ctype)
        elif path == "/api/state":
            self._send(200, state())
        elif path == "/api/ping":
            self._send(200, {"app": APP_ID})
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        path = urlparse(self.path).path
        n = int(self.headers.get("Content-Length") or 0)
        try:
            data = json.loads(self.rfile.read(n).decode("utf-8")) if n else {}
        except ValueError:
            data = {}
        try:
            code, obj = self.route(path, data)
        except Exception as e:
            traceback.print_exc()
            code, obj = 500, {"error": str(e)}
        self._send(code, obj)

    def route(self, path, data):
        busy = (409, {"error": "正在忙，请等当前操作完成"})
        phase = CTL.phase

        if path == "/api/live":
            since = data.get("since") if isinstance(data.get("since"), dict) else {}
            return 200, CTL.live_since(since)

        if path == "/api/stop":
            CTL.stop_event.set()
            return 200, {"ok": True}

        if path == "/api/settings":
            capturing = phase in ("syncing", "countdown", "streaming", "recording", "saving")
            if capturing and any(k in data for k in ("mode", "rate", "filter", "payload")):
                return 409, {"error": "采集中不能修改采样率 / 模式"}
            changed = SETTINGS.update(data)
            if set(changed) & {"rate", "filter", "mode"}:
                submit("写入设置", _apply_settings)
            return 200, {"ok": True, "changed": changed}

        if path == "/api/role":
            SETTINGS.update({"roles": {data.get("mac", ""): data.get("role", "")}})
            return 200, {"ok": True}

        if path == "/api/pick_dir":
            start = SETTINGS.out_dir()
            os.makedirs(start, exist_ok=True)
            chosen = pick_folder(start)
            if chosen:
                SETTINGS.update({"output_dir": relativize(chosen)})
            return 200, {"ok": True, "path": chosen}

        if path == "/api/open_dir":
            p = data.get("path") or SETTINGS.out_dir()
            if not os.path.isdir(p):
                os.makedirs(p, exist_ok=True)
            open_folder(p)
            return 200, {"ok": True}

        jobs = {
            "/api/scan": ("扫描", CTL.scan, ()),
            "/api/disconnect": ("断开", CTL.disconnect, ()),
            "/api/usb/scan": ("查找 USB", CTL.usb_scan, ()),
        }
        if path in jobs:
            name, fn, args = jobs[path]
            return (200, {"ok": True}) if submit(name, fn, *args) else busy

        if path == "/api/connect":
            macs = [m for m in data.get("macs", []) if m]
            if not macs:
                return 400, {"error": "请先勾选要连接的传感器"}
            return (200, {"ok": True}) if submit("连接", CTL.connect, macs) else busy

        if path == "/api/identify":
            return (200, {"ok": True}) if submit("闪灯", CTL.identify, data.get("mac", "")) else busy

        if path == "/api/start":
            if phase != "ready" or not CTL._live_devs():
                return 409, {"error": "请先连接至少一个传感器"}
            out = SETTINGS.out_dir()
            try:
                os.makedirs(out, exist_ok=True)
            except OSError as e:
                return 400, {"error": "保存路径不可用：%s" % e}
            return (200, {"ok": True}) if submit("采集", CTL.start_capture, out) else busy

        if path == "/api/usb/export":
            out = SETTINGS.out_dir()
            os.makedirs(out, exist_ok=True)
            return (200, {"ok": True}) if submit("导出", CTL.usb_export, out, data.get("dids")) else busy

        if path == "/api/usb/erase":
            dids = data.get("dids") or []
            if not dids:
                return 400, {"error": "请先勾选要清空的传感器"}
            return (200, {"ok": True}) if submit("清空芯片", CTL.usb_erase, dids) else busy

        return 404, {"error": "unknown api"}


def _already_running(port):
    try:
        with urllib.request.urlopen("http://%s:%d/api/ping" % (HOST, port), timeout=1) as r:
            return json.load(r).get("app") == APP_ID
    except Exception:
        return False


def main():
    httpd = None
    for port in range(BASE_PORT, BASE_PORT + 10):
        if _already_running(port):
            print("采集界面已经在运行，直接打开浏览器。")
            webbrowser.open("http://%s:%d/" % (HOST, port))
            return
        try:
            httpd = ThreadingHTTPServer((HOST, port), Handler)
            break
        except OSError:
            continue
    if httpd is None:
        print("端口 %d~%d 都被占用，无法启动。" % (BASE_PORT, BASE_PORT + 9))
        sys.exit(1)
    threading.Thread(target=_worker, daemon=True).start()
    url = "http://%s:%d/" % (HOST, httpd.server_address[1])
    log("IMU 采集界面已启动：%s （关闭这个黑色窗口即退出）" % url)
    log("数据保存到：%s" % SETTINGS.out_dir())
    if not os.environ.get("IMU_NO_BROWSER"):
        webbrowser.open(url)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        CTL.close()


if __name__ == "__main__":
    main()
