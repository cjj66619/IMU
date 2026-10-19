"use strict";
const $ = (s) => document.querySelector(s);
const CAPTURING = ["syncing", "countdown", "streaming", "recording", "saving"];
const PHASE_TEXT = {
  idle: "未连接", scanning: "扫描中", connecting: "处理中", ready: "就绪", syncing: "同步中",
  countdown: "倒计时", streaming: "实时采集中", recording: "板载录制中", saving: "保存中", usb: "USB 操作中",
};
let S = null;                 // 最近一次 /api/state
const unchecked = new Set();  // 扫描列表里取消勾选的 MAC
const usbUnchecked = new Set();

async function api(path, body) {
  const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) alert(j.error || ("请求失败 " + r.status));
  poll();
  return j;
}
const setting = (patch) => api("/api/settings", patch);

function esc(s) { return String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c])); }
function fmtBytes(n) { if (n == null) return "?"; return n > 1e6 ? (n / 1e6).toFixed(1) + " MB" : (n / 1e3).toFixed(0) + " KB"; }
function fmtSec(s) { s = Math.round(s || 0); return s >= 60 ? Math.floor(s / 60) + " 分 " + (s % 60) + " 秒" : s + " 秒"; }

// 正在编辑的输入框不被轮询覆盖
function setVal(el, v) { if (document.activeElement !== el) el.type === "checkbox" ? (el.checked = !!v) : (el.value = v); }
function setHTML(el, html) { if (el.dataset.html !== html) { el.dataset.html = html; el.innerHTML = html; } }
function setOptions(sel, items) {
  const key = JSON.stringify(items);
  if (sel.dataset.key === key) return;
  sel.dataset.key = key;
  sel.innerHTML = items.map(([v, t]) => `<option value="${esc(v)}">${esc(t)}</option>`).join("");
}

// ---------------------------------------------------------------- render
function render(s) {
  S = s;
  const st = s.settings, capturing = CAPTURING.includes(s.phase), busy = s.busy;
  const live = s.devices.filter((d) => !d.lost);

  // 顶部状态
  const pill = $("#status");
  pill.textContent = (PHASE_TEXT[s.phase] || s.phase) + " · " + s.status;
  pill.className = "pill " + (capturing ? "run" : busy ? "busy" : s.phase === "ready" ? "ok" : "");
  $("#error").textContent = s.error ? "⚠️ " + s.error : "";
  $("#error").classList.toggle("hidden", !s.error);

  // ① 扫描结果
  const cand = s.scanned.filter((x) => !x.connected);
  setHTML($("#scanned"), cand.length ? `<div class="chips">${cand.map((x) => `
      <label class="chip"><input type="checkbox" data-mac="${x.mac}" ${unchecked.has(x.mac) ? "" : "checked"}>
      <b>${esc(x.role)}</b><span class="muted">${x.mac}</span></label>`).join("")}</div>` : "");
  $("#btn-scan").disabled = busy;
  $("#btn-connect").disabled = busy || !cand.some((x) => !unchecked.has(x.mac));
  $("#btn-disconnect").disabled = busy || !s.devices.length;

  // ① 已连接
  $("#devices").classList.toggle("hidden", !s.devices.length);
  const tb = $("#devices tbody");
  const rows = s.devices.map((d) => d.mac).join(",");
  if (tb.dataset.rows !== rows) {
    tb.dataset.rows = rows;
    tb.innerHTML = s.devices.map((d) => `<tr data-mac="${d.mac}">
      <td><input type="text" class="role" data-mac="${d.mac}"></td><td class="muted">${d.mac}</td>
      <td class="bat"></td><td class="rate"></td><td class="flt"></td><td class="recs"></td><td class="fw muted"></td><td class="hz"></td>
      <td><button class="small identify" data-mac="${d.mac}">闪灯</button></td></tr>`).join("");
  }
  for (const d of s.devices) {
    const tr = tb.querySelector(`tr[data-mac="${d.mac}"]`);
    setVal(tr.querySelector(".role"), d.role);
    tr.querySelector(".bat").innerHTML = d.lost ? '<span class="bad-t">已断开</span>' :
      (d.battery == null ? "?" : `<span class="${d.battery < 20 ? "bad-t" : ""}">${d.battery}%${d.charging ? " ⚡" : ""}</span>`);
    tr.querySelector(".rate").textContent = d.rate ? d.rate + " Hz" : "?";
    tr.querySelector(".flt").textContent = d.filter || "?";
    tr.querySelector(".recs").textContent = d.recordings == null ? "?" : d.recordings + " 段";
    tr.querySelector(".fw").textContent = d.firmware || "";
    tr.querySelector(".hz").textContent = d.hz ? d.hz + " Hz" : "";
    tr.querySelector(".identify").disabled = busy || d.lost;
  }

  // ② 设置
  document.querySelectorAll("#mode button").forEach((b) => { b.classList.toggle("on", b.dataset.v === st.mode); b.disabled = capturing; });
  const rates = st.mode === "live" ? s.options.live_rates : s.options.record_rates;
  setOptions($("#rate"), rates.map((r) => [r, r === 120 ? "120（仅芯片录制）" : String(r)]));
  setOptions($("#payload"), s.options.payloads);
  setOptions($("#filter"), s.options.filters.map((f) => [f, f]));
  setVal($("#rate"), st.rate); setVal($("#payload"), st.payload); setVal($("#filter"), st.filter);
  setVal($("#duration"), st.duration); setVal($("#countdown"), st.countdown); setVal($("#tag"), st.tag); setVal($("#sync"), st.sync);
  $("#lbl-payload").classList.toggle("hidden", st.mode !== "live");
  $("#lbl-sync").classList.toggle("hidden", st.mode !== "live");
  for (const id of ["rate", "payload", "filter", "duration", "countdown", "tag", "sync"]) $("#" + id).disabled = capturing;

  // ③ 保存位置
  $("#out-dir").textContent = s.out_dir;
  $("#btn-pick").disabled = capturing;

  // 开始 / 停止
  const exporting = s.usb_job && !s.usb_job.finished;
  $("#btn-start").classList.toggle("hidden", capturing);
  $("#btn-stop").classList.toggle("hidden", !capturing && !exporting);
  $("#btn-stop").textContent = exporting ? "中止导出" : "停止";
  $("#btn-start").disabled = busy || s.phase !== "ready" || !live.length;
  $("#btn-start").textContent = st.mode === "live" ? "开始采集" : "开始芯片录制";
  const c = s.capture;
  let txt = "", pct = 0;
  if (!live.length) txt = "先在第 1 步连接传感器";
  else if (!capturing && !(c && c.done)) txt = `将以 ${st.rate} Hz ${st.mode === "live" ? "实时采集" : "在芯片里录制"} ${st.duration > 0 ? fmtSec(st.duration) : "（手动停止）"}，${live.length} 个传感器`;
  if (capturing && c) {
    pct = c.duration > 0 ? Math.min(100, (c.elapsed / c.duration) * 100) : (c.elapsed % 10) * 10;
    txt = `${s.status}  已 ${fmtSec(c.elapsed)}${c.duration > 0 ? " / " + fmtSec(c.duration) : "（点「停止」结束）"}`;
  } else if (capturing) txt = s.status;
  $("#bar").style.width = pct + "%";
  $("#run-text").textContent = txt;

  // 实时曲线
  const showLive = capturing && c && c.mode === "live";
  const liveBox = $("#live");
  liveBox.classList.toggle("hidden", !showLive);
  if (showLive) {
    const ids = live.map((d) => d.mac).join(",");
    if (liveBox.dataset.ids !== ids) {
      liveBox.dataset.ids = ids;
      liveBox.innerHTML = live.map((d) => `<div class="live" data-mac="${d.mac}"><div class="t"><b>${esc(d.role)}</b><span class="info"></span></div><canvas></canvas></div>`).join("");
    }
    for (const d of live) {
      const box = liveBox.querySelector(`.live[data-mac="${d.mac}"]`);
      if (!box) continue;
      box.querySelector(".info").textContent = `${LIVE.label || ""}  ${d.hz ? d.hz + " Hz" : "等待数据…"} · ${d.count} 帧`;
    }
  }

  // 结果
  const res = $("#result");
  if (c && c.done && !capturing) {
    res.classList.remove("hidden");
    const lines = (c.summary || []).map((x) => c.mode === "live"
      ? `<li>${esc(x.role)}：${x.rows ? `<span class="ok-t">${x.rows} 行（${x.hz} Hz）</span>` : '<span class="bad-t">没有数据</span>'}</li>`
      : `<li>${esc(x.role)}：已存入芯片，第 ${x.recording_index ?? "?"} 段</li>`).join("");
    res.innerHTML = `<b>✅ ${c.mode === "live" ? "采集完成" : "芯片录制完成（记得用下方 USB 导出）"}</b>，用时 ${fmtSec(c.elapsed)}<ul>${lines}</ul>
      <div class="row"><code class="path">${esc(c.dir)}</code><button class="small" id="btn-open-last">打开这个文件夹</button></div>`;
    $("#btn-open-last").onclick = () => api("/api/open_dir", { path: c.dir });
  } else res.classList.add("hidden");

  // USB
  $("#usb").classList.toggle("hidden", !s.usb.length);
  setHTML($("#usb tbody"), s.usb.map((u) => `<tr>
    <td><input type="checkbox" class="usb-sel" data-did="${u.did}" ${usbUnchecked.has(u.did) ? "" : "checked"}></td>
    <td><b>${esc(u.role)}</b></td><td class="muted">${u.mac}</td><td>${u.recordings ?? "?"}</td><td>${fmtBytes(u.used)}</td>
    <td class="muted">${(u.recs || []).map((r) => `#${r.index} ${r.start} · ${r.seconds ?? "?"}s`).join("<br>") || "（空）"}</td></tr>`).join(""));
  const anyRecs = s.usb.some((u) => (u.recs || []).length);
  const selUsb = s.usb.filter((u) => !usbUnchecked.has(u.did));
  $("#btn-usb-scan").disabled = busy;
  $("#btn-usb-export").disabled = busy || !anyRecs || !selUsb.some((u) => (u.recs || []).length);
  $("#btn-usb-erase").disabled = busy || !selUsb.length;
  const j = s.usb_job;
  $("#usb-job").classList.toggle("hidden", !j);
  if (j) $("#usb-job").innerHTML = j.finished
    ? `✅ 导出完成：${j.files.length} 个文件 <button class="small" id="btn-open-out">打开保存位置</button>`
    : `导出中：${esc(j.current)}（${j.done}/${j.total} 段），已收到 ${j.packets} 帧…`;
  const bo = $("#btn-open-out"); if (bo) bo.onclick = () => api("/api/open_dir", {});

  // 日志
  const log = $("#log"), text = s.log.join("\n");
  if (log.textContent !== text) {
    const atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 5;
    log.textContent = text;
    if (atBottom) log.scrollTop = log.scrollHeight;
  }
}

// ---------------------------------------------------------------- 实时曲线（平滑回放）
// 蓝牙数据是一阵一阵到的；这里按传感器时间、延迟 DELAY_S 匀速回放，60 fps 滚动显示最近 WINDOW_S 秒。
const WINDOW_S = 5, DELAY_S = 0.3;
const LIVE = { session: null, since: {}, buf: {}, play: {}, scale: {}, label: "", inflight: false };

function liveVisible() { return !$("#live").classList.contains("hidden"); }

async function pollLive() {
  if (!liveVisible() || LIVE.inflight) return;
  LIVE.inflight = true;
  try {
    const r = await fetch("/api/live", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ since: LIVE.since }) });
    const j = await r.json();
    if (j.session !== LIVE.session) Object.assign(LIVE, { session: j.session, since: {}, buf: {}, play: {}, scale: {} });
    LIVE.label = j.label || "";
    for (const [mac, d] of Object.entries(j.devices || {})) {
      if (!d.pts.length) continue;
      const buf = (LIVE.buf[mac] ||= []);
      for (const [seq, t, v] of d.pts) buf.push({ t, v });
      LIVE.since[mac] = d.pts[d.pts.length - 1][0];
      const keepFrom = buf[buf.length - 1].t - WINDOW_S - 1;
      let k = 0; while (k < buf.length && buf[k].t < keepFrom) k++;
      if (k) buf.splice(0, k);
    }
  } catch (e) { /* 下一次再取 */ } finally { LIVE.inflight = false; }
}

function frame(now) {
  requestAnimationFrame(frame);
  if (!liveVisible()) return;
  for (const box of document.querySelectorAll("#live .live")) {
    const mac = box.dataset.mac, buf = LIVE.buf[mac], cv = box.querySelector("canvas");
    const w = cv.clientWidth, h = cv.clientHeight, dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    const g = cv.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    if (!buf || buf.length < 2) continue;
    // 播放头：随真实时间匀速前进；落后太多或跑到数据前面就重新对齐
    const newest = buf[buf.length - 1].t;
    let p = LIVE.play[mac];
    if (!p) p = LIVE.play[mac] = { t: newest - DELAY_S, wall: now };
    let cur = p.t + (now - p.wall) / 1000;
    if (cur > newest) { cur = newest; p.t = cur; p.wall = now; }
    else if (cur < newest - DELAY_S - 0.7) { cur = newest - DELAY_S; p.t = cur; p.wall = now; }
    const t0 = cur - WINDOW_S;
    let lo = Infinity, hi = -Infinity;
    for (const s of buf) if (s.t >= t0 && s.t <= cur) { if (s.v < lo) lo = s.v; if (s.v > hi) hi = s.v; }
    if (!isFinite(lo)) continue;
    if (hi - lo < 0.05) { const m = (hi + lo) / 2; lo = m - 0.025; hi = m + 0.025; }
    // 纵轴范围平滑变化，避免曲线上下跳
    const sc = (LIVE.scale[mac] ||= { lo, hi });
    sc.lo += (lo - sc.lo) * (lo < sc.lo ? 0.5 : 0.05);
    sc.hi += (hi - sc.hi) * (hi > sc.hi ? 0.5 : 0.05);
    const span = sc.hi - sc.lo || 1;
    g.strokeStyle = "#2f6fed"; g.lineWidth = 1.5; g.lineJoin = "round"; g.beginPath();
    let started = false;
    for (const s of buf) {
      if (s.t < t0 - 0.1) continue;
      if (s.t > cur) break;
      const x = ((s.t - t0) / WINDOW_S) * w, y = h - 4 - ((s.v - sc.lo) / span) * (h - 8);
      started ? g.lineTo(x, y) : g.moveTo(x, y);
      started = true;
    }
    g.stroke();
    g.fillStyle = "#6b7489"; g.font = "11px sans-serif";
    g.fillText(sc.hi.toFixed(2), 2, 11); g.fillText(sc.lo.toFixed(2), 2, h - 2);
  }
}
setInterval(pollLive, 100);
requestAnimationFrame(frame);

// ---------------------------------------------------------------- events
$("#btn-scan").onclick = () => api("/api/scan");
$("#btn-connect").onclick = () => api("/api/connect", { macs: S.scanned.filter((x) => !x.connected && !unchecked.has(x.mac)).map((x) => x.mac) });
$("#btn-disconnect").onclick = () => confirm("断开全部传感器？") && api("/api/disconnect");
$("#scanned").onchange = (e) => { const m = e.target.dataset.mac; if (m) e.target.checked ? unchecked.delete(m) : unchecked.add(m); render(S); };
$("#devices").onclick = (e) => { if (e.target.classList.contains("identify")) api("/api/identify", { mac: e.target.dataset.mac }); };
$("#devices").addEventListener("change", (e) => { if (e.target.classList.contains("role")) api("/api/role", { mac: e.target.dataset.mac, role: e.target.value }); });
$("#devices").addEventListener("keydown", (e) => { if (e.key === "Enter" && e.target.classList.contains("role")) e.target.blur(); });

document.querySelectorAll("#mode button").forEach((b) => (b.onclick = () => setting({ mode: b.dataset.v })));
for (const id of ["rate", "payload", "filter"]) $("#" + id).onchange = (e) => setting({ [id]: e.target.value });
for (const id of ["duration", "countdown", "tag"]) $("#" + id).onchange = (e) => setting({ [id]: e.target.value });
$("#sync").onchange = (e) => setting({ sync: e.target.checked });

$("#btn-pick").onclick = () => api("/api/pick_dir");
$("#btn-open").onclick = () => api("/api/open_dir", {});
$("#btn-start").onclick = () => api("/api/start");
$("#btn-stop").onclick = () => api("/api/stop");

$("#btn-usb-scan").onclick = () => api("/api/usb/scan");
$("#usb").onchange = (e) => {
  if (e.target.id === "usb-all") { S.usb.forEach((u) => (e.target.checked ? usbUnchecked.delete(u.did) : usbUnchecked.add(u.did))); }
  else if (e.target.dataset.did) { e.target.checked ? usbUnchecked.delete(e.target.dataset.did) : usbUnchecked.add(e.target.dataset.did); }
  render(S);
};
$("#btn-usb-export").onclick = () => api("/api/usb/export", { dids: S.usb.filter((u) => !usbUnchecked.has(u.did)).map((u) => u.did) });
$("#btn-usb-erase").onclick = () => {
  const sel = S.usb.filter((u) => !usbUnchecked.has(u.did));
  const n = sel.reduce((a, u) => a + (u.recordings || 0), 0);
  if (confirm(`将永久删除 ${sel.map((u) => u.role).join("、")} 芯片里的 ${n} 段录制。\n\n确认已经导出过了吗？`))
    api("/api/usb/erase", { dids: sel.map((u) => u.did) });
};

// ---------------------------------------------------------------- poll
let polling = false;
async function poll() {
  if (polling) return;
  polling = true;
  try {
    const r = await fetch("/api/state");
    render(await r.json());
  } catch (e) {
    $("#status").textContent = "与后台断开：请重新双击「启动IMU采集.bat」";
    $("#status").className = "pill run";
  } finally { polling = false; }
}
setInterval(poll, 500);
poll();
