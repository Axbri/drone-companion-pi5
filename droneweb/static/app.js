"use strict";

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 0) => (v === undefined || v === null ? "–" : Number(v).toFixed(d));
const setText = (sel, text) => document.querySelectorAll(sel).forEach((el) => (el.textContent = text));

const FIX = { 0: "None", 1: "No fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK-Float", 6: "RTK-Fixed" };

// ---- telemetry (~5 Hz) -----------------------------------------------------
async function pollTelemetry() {
  try {
    const t = await (await fetch("/api/telemetry", { cache: "no-store" })).json();
    const fresh = t.connected && t.updated && Date.now() / 1000 - t.updated < 3;

    const link = $("link");
    link.className = "pill " + (fresh ? "pill-good" : "pill-bad");
    link.textContent = fresh ? "MAVLink OK" : "MAVLink…";

    $("mode").textContent = t.mode || "—";
    const armed = $("armed");
    armed.textContent = t.armed ? "ARMED" : "DISARMED";
    armed.className = "pill " + (t.armed ? "pill-armed" : "");

    // Battery + Attitude cards appear on both Pilot view and Precision landing tabs.
    setText(".f-volt", fmt(t.voltage, 2));
    setText(".f-curr", fmt(t.current, 1));
    setText(".f-batt", fmt(t.battery_remaining));
    setBar(".f-batt-bar", t.battery_remaining, [20, 40], true);

    setText(".f-roll", fmt(t.roll, 1));
    setText(".f-pitch", fmt(t.pitch, 1));
    setText(".f-hdg", fmt(t.heading));
    setText(".f-alt", fmt(t.alt, 1));
    setText(".f-gps", (FIX[t.fix_type] || "–") + (t.satellites != null ? ` · ${t.satellites} sat` : ""));

    if (hudOn) drawHud("hud-pilot", t.roll || 0, t.pitch || 0, t.heading || 0);
    drawADI("adi-precland", t.roll || 0, t.pitch || 0);
  } catch (e) {
    $("link").className = "pill pill-bad";
    $("link").textContent = "offline";
  }
}

// ---- Pi stats (~1 Hz) -------------------------------------------------------
async function pollStats() {
  try {
    const s = await (await fetch("/api/stats", { cache: "no-store" })).json();
    $("cpu").textContent = fmt(s.cpu_percent);
    setBar("#cpu-bar", s.cpu_percent, [60, 85]);
    $("mem").textContent = `${s.mem_used_mb} / ${s.mem_total_mb} MB`;
    setBar("#mem-bar", s.mem_percent, [75, 90]);
    $("net").textContent = `${fmt(s.net_rx_kbps, 0)} / ${fmt(s.net_tx_kbps, 0)}`;
    $("temp").textContent = fmt(s.temp_c, 1);
  } catch (e) {}
}

function setBar(sel, pct, [warn, bad], invert = false) {
  if (pct == null) return;
  const width = Math.max(0, Math.min(100, pct)) + "%";
  let color = "var(--good)";
  if (invert) {
    if (pct <= warn) color = "var(--bad)";
    else if (pct <= bad) color = "var(--warn)";
  } else {
    if (pct >= bad) color = "var(--bad)";
    else if (pct >= warn) color = "var(--warn)";
  }
  document.querySelectorAll(sel).forEach((el) => {
    el.style.width = width;
    el.style.background = color;
  });
}

// ---- artificial horizon (roll/pitch) ---------------------------------------
function drawADI(canvasId, roll, pitch) {
  const c = $(canvasId);
  if (!c) return;
  const ctx = c.getContext("2d");
  const w = c.width, h = c.height, r = w / 2;
  ctx.clearRect(0, 0, w, h);
  ctx.save();
  ctx.beginPath();
  ctx.arc(r, r, r - 2, 0, Math.PI * 2);
  ctx.clip();
  ctx.translate(r, r);
  ctx.rotate((-roll * Math.PI) / 180);
  const horizon = (pitch / 45) * r;
  ctx.translate(0, horizon);
  ctx.fillStyle = "#4a7fb5";                 // sky
  ctx.fillRect(-r, -r * 2, w, r * 2);
  ctx.fillStyle = "#6b4a2f";                 // ground
  ctx.fillRect(-r, 0, w, r * 2);
  ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(-r, 0); ctx.lineTo(r, 0); ctx.stroke();
  ctx.restore();
  // fixed aircraft symbol
  ctx.strokeStyle = "#ffcc00"; ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(r - 22, r); ctx.lineTo(r - 6, r);
  ctx.moveTo(r + 6, r); ctx.lineTo(r + 22, r);
  ctx.moveTo(r, r - 6); ctx.lineTo(r, r + 4);
  ctx.stroke();
}

// ---- FPV HUD overlay (pitch ladder, bank indicator, heading tape) --------
const hudCanvas = $("hud-pilot"), hudToggle = $("hud-toggle");
let hudOn = localStorage.getItem("hudOn") !== "0";
hudToggle.checked = hudOn;
hudCanvas.style.display = hudOn ? "block" : "none";
hudToggle.addEventListener("change", () => {
  hudOn = hudToggle.checked;
  localStorage.setItem("hudOn", hudOn ? "1" : "0");
  hudCanvas.style.display = hudOn ? "block" : "none";
});

function drawHud(canvasId, roll, pitch, heading) {
  const c = $(canvasId);
  const rect = c.getBoundingClientRect();
  if (!rect.width || !rect.height) return;   // hidden tab or not laid out yet
  const dpr = window.devicePixelRatio || 1;
  const w = Math.round(rect.width * dpr), h = Math.round(rect.height * dpr);
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }

  const ctx = c.getContext("2d");
  ctx.clearRect(0, 0, w, h);
  const cx = w / 2, cy = h / 2, scale = Math.min(w, h);
  ctx.strokeStyle = "#ffcc00"; ctx.fillStyle = "#ffcc00";
  ctx.lineWidth = Math.max(1.5, scale * 0.0025);
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  ctx.shadowColor = "rgba(0,0,0,0.7)"; ctx.shadowBlur = scale * 0.004;
  ctx.font = `600 ${Math.round(scale * 0.026)}px system-ui, sans-serif`;

  // ---- pitch ladder, rotated with roll around the reticle ----
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate((-roll * Math.PI) / 180);
  const pxPerDeg = scale * 0.012, halfW = scale * 0.15;
  for (let deg = -90; deg <= 90; deg += 10) {
    const y = (deg - pitch) * pxPerDeg;
    if (Math.abs(y) > scale * 0.42) continue;
    const major = deg % 30 === 0;
    const hw = deg === 0 ? halfW * 1.7 : (major ? halfW : halfW * 0.5);
    ctx.beginPath();
    if (deg === 0) {   // horizon line, broken in the middle for the reticle
      ctx.moveTo(-hw, y); ctx.lineTo(-halfW * 0.35, y);
      ctx.moveTo(halfW * 0.35, y); ctx.lineTo(hw, y);
    } else {
      ctx.moveTo(-hw, y); ctx.lineTo(hw, y);
    }
    ctx.stroke();
    if (major && deg !== 0) {
      ctx.fillText(String(Math.abs(deg)), -hw - scale * 0.035, y);
      ctx.fillText(String(Math.abs(deg)), hw + scale * 0.035, y);
    }
  }
  ctx.restore();

  // ---- bank (roll) arc, fixed, top center — ticks fixed, pointer rotates ----
  const arcY = cy - scale * 0.30, arcR = scale * 0.13;
  ctx.save();
  ctx.beginPath();
  ctx.arc(cx, arcY, arcR, Math.PI * 1.22, Math.PI * 1.78);
  ctx.stroke();
  [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60].forEach((deg) => {
    const a = Math.PI * 1.5 - (deg * Math.PI) / 180;
    const r2 = arcR - (deg % 30 === 0 ? scale * 0.02 : scale * 0.012);
    ctx.beginPath();
    ctx.moveTo(cx + arcR * Math.cos(a), arcY + arcR * Math.sin(a));
    ctx.lineTo(cx + r2 * Math.cos(a), arcY + r2 * Math.sin(a));
    ctx.stroke();
  });
  const pa = Math.PI * 1.5 + (roll * Math.PI) / 180;
  const tipR = arcR + scale * 0.015, baseR = arcR - scale * 0.01, spread = 0.05;
  ctx.beginPath();
  ctx.moveTo(cx + tipR * Math.cos(pa), arcY + tipR * Math.sin(pa));
  ctx.lineTo(cx + baseR * Math.cos(pa - spread), arcY + baseR * Math.sin(pa - spread));
  ctx.lineTo(cx + baseR * Math.cos(pa + spread), arcY + baseR * Math.sin(pa + spread));
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  // ---- heading tape, fixed, near top edge ----
  const tapeY = scale * 0.06, pxPerHdgDeg = scale * 0.011;
  ctx.save();
  ctx.beginPath(); ctx.moveTo(0, tapeY); ctx.lineTo(w, tapeY); ctx.stroke();
  const start = Math.floor((heading - 60) / 10) * 10;
  for (let hv = start; hv <= heading + 60; hv += 10) {
    const x = cx + (hv - heading) * pxPerHdgDeg;
    if (x < 0 || x > w) continue;
    const hd = ((hv % 360) + 360) % 360;
    const major = hd % 30 === 0;
    ctx.beginPath();
    ctx.moveTo(x, tapeY); ctx.lineTo(x, tapeY + (major ? scale * 0.02 : scale * 0.012));
    ctx.stroke();
    if (major) {
      const label = hd === 0 ? "N" : hd === 90 ? "E" : hd === 180 ? "S" : hd === 270 ? "W" : String(hd);
      ctx.fillText(label, x, tapeY + scale * 0.045);
    }
  }
  ctx.beginPath();
  ctx.moveTo(cx, tapeY - scale * 0.012); ctx.lineTo(cx - scale * 0.012, tapeY - scale * 0.03);
  ctx.lineTo(cx + scale * 0.012, tapeY - scale * 0.03); ctx.closePath(); ctx.fill();
  ctx.font = `700 ${Math.round(scale * 0.03)}px system-ui, sans-serif`;
  ctx.fillText(`${Math.round(heading)}°`, cx, tapeY - scale * 0.05);
  ctx.restore();

  // ---- fixed center reticle ----
  ctx.save();
  ctx.lineWidth = Math.max(1.5, scale * 0.003);
  ctx.beginPath();
  ctx.moveTo(cx - scale * 0.03, cy); ctx.lineTo(cx - scale * 0.012, cy);
  ctx.moveTo(cx + scale * 0.012, cy); ctx.lineTo(cx + scale * 0.03, cy);
  ctx.moveTo(cx, cy - scale * 0.018); ctx.lineTo(cx, cy - scale * 0.006);
  ctx.stroke();
  ctx.beginPath(); ctx.arc(cx, cy, scale * 0.006, 0, Math.PI * 2); ctx.fill();
  ctx.restore();
}

// ---- video hints --------------------------------------------------------
$("video").addEventListener("load", () => ($("video-hint").style.display = "none"));
$("video").addEventListener("error", () => {
  $("video-hint").style.display = "block";
  $("video-hint").textContent = "No video – check droneweb/camera";
});
$("video-hq").addEventListener("load", () => ($("video-hq-hint").style.display = "none"));
$("video-hq").addEventListener("error", () => {
  $("video-hq-hint").style.display = "block";
  $("video-hq-hint").textContent = "No video – check droneweb/HQ camera";
});

// ---- shutdown ------------------------------------------------------------
$("shutdown").addEventListener("click", async () => {
  if (!confirm("Shut down the Raspberry Pi?\n\nThe drone will then lose RTK, telemetry, and video until it is restarted manually.")) return;
  try {
    await fetch("/api/shutdown", { method: "POST" });
    document.body.innerHTML =
      '<div style="display:grid;place-items:center;height:100vh;color:#8b98a8;font-family:system-ui">Pi is shutting down…</div>';
  } catch (e) {
    alert("Shutdown failed: " + e);
  }
});

// ---- precision landing ---------------------------------------------------
let plArmed = false;
const plBtn = $("pl-arm"), plCard = $("precland-card");

async function pollPrecland() {
  try {
    const p = await (await fetch("/api/precland", { cache: "no-store" })).json();
    plArmed = !!p.armed;
    plBtn.textContent = plArmed ? "Disarm" : "Arm";
    plBtn.classList.toggle("armed", plArmed);
    plCard.classList.toggle("armed", plArmed);
    $("pl-phase").textContent = p.phase || (plArmed ? "…" : "off");
    $("pl-source").textContent = p.source || "–";
    $("pl-agl").textContent = p.agl != null ? Number(p.agl).toFixed(2) : "–";
    $("pl-offset").textContent = p.offset ? `${p.offset[0]}, ${p.offset[1]}` : "–";
    $("pl-tx").textContent = p.sent ? `sending (${p.tx})` : (p.tx ? `paused (${p.tx})` : "–");
    $("pl-rf").textContent = p.rangefinder_ok ? "OK" : "none";
    const running = p.armed || p.recording;
    $("pl-lat").textContent = running && p.latency_ms != null ? p.latency_ms : "–";
    $("pl-hz").textContent = running && p.loop_hz != null ? p.loop_hz : "–";
    updateRecBtn(p.recording);
    exposureCtl.seed(p.exposure);
    renderCalib(p.calib);
    seedScale(p.cmd_scale);
  } catch (e) {}
}

// ---- control scale (command scaling to ArduPilot) ------------------------
const scaleEl = $("pl-scale");
let scaleSeeded = false, scaleTimer = null;

function reflectScale() {
  $("pl-scale-val").textContent = Number(scaleEl.value).toFixed(2);
}

function seedScale(v) {
  if (v == null || scaleSeeded) return;
  scaleSeeded = true;
  scaleEl.value = v;
  reflectScale();
}

scaleEl.addEventListener("input", () => {
  reflectScale();
  clearTimeout(scaleTimer);
  scaleTimer = setTimeout(() => {
    fetch("/api/landscale", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ scale: Number(scaleEl.value) }),
    }).catch(() => {});
  }, 120);
});
reflectScale();

function renderCalib(c) {
  $("cal-px").textContent = c && c.px != null ? c.px : "–";
  $("cal-f").textContent = c && c.f_meas != null ? c.f_meas : "–";
  $("cal-goff").textContent = c && c.goff_cm != null ? c.goff_cm : "–";
  const el = $("cal-scale");
  if (c && c.f_meas != null && c.assumed_f) {
    const sc = c.f_meas / c.assumed_f;
    el.textContent = sc.toFixed(2) + "×";
    const d = Math.abs(sc - 1);
    el.style.color = d < 0.1 ? "var(--good)" : (d < 0.25 ? "var(--warn)" : "var(--bad)");
  } else {
    el.textContent = "–";
    el.style.color = "";
  }
}

// ---- exposure controls (precland cam + HQ/pilot cam use the same pattern) -
function setupExposureControl(endpoint, ids) {
  const auto = $(ids.auto), us = $(ids.us), gain = $(ids.gain);
  const usVal = $(ids.usVal), gainVal = $(ids.gainVal), card = $(ids.card);
  let seeded = false, timer = null;

  function reflect() {
    usVal.textContent = us.value;
    gainVal.textContent = Number(gain.value).toFixed(1);
    card.classList.toggle("auto", auto.checked);
  }

  function send() {
    fetch(endpoint, {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        auto: auto.checked,
        exposure_us: Number(us.value),
        gain: Number(gain.value),
      }),
    }).catch(() => {});
  }

  function onInput() {
    reflect();
    clearTimeout(timer);
    timer = setTimeout(send, 120);   // debounce live drag
  }

  auto.addEventListener("change", () => { reflect(); send(); });
  us.addEventListener("input", onInput);
  gain.addEventListener("input", onInput);
  reflect();

  return {
    // seeds the controls from server values the first time (e.g. previously set)
    seed(exp) {
      if (!exp || seeded) return;
      seeded = true;
      auto.checked = !!exp.auto;
      us.value = exp.exposure_us;
      gain.value = exp.gain;
      reflect();
    },
  };
}

const exposureCtl = setupExposureControl("/api/exposure", {
  auto: "exp-auto", us: "exp-us", gain: "exp-gain",
  usVal: "exp-us-val", gainVal: "exp-gain-val", card: "exposure-card",
});
const exposureHqCtl = setupExposureControl("/api/exposure_hq", {
  auto: "exp2-auto", us: "exp2-us", gain: "exp2-gain",
  usVal: "exp2-us-val", gainVal: "exp2-gain-val", card: "exposure-hq-card",
});

async function pollExposureHq() {
  try {
    const exp = await (await fetch("/api/exposure_hq", { cache: "no-store" })).json();
    exposureHqCtl.seed(exp);
  } catch (e) {}
}

// ---- recording -------------------------------------------------------
const recBtn = $("pl-rec");
let recording = false;

function updateRecBtn(rec) {
  recording = !!rec;
  recBtn.textContent = recording ? "■ Stop recording" : "● Record";
  recBtn.classList.toggle("recording", recording);
}

recBtn.addEventListener("click", async () => {
  try {
    await fetch("/api/record", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ recording: !recording }),
    });
    pollPrecland();
    setTimeout(pollRecordings, 500);
  } catch (e) { alert("Error: " + e); }
});

async function pollRecordings() {
  try {
    const list = await (await fetch("/api/recordings", { cache: "no-store" })).json();
    const el = $("rec-list");
    if (!list.length) { el.innerHTML = '<div class="rec-empty">No recordings yet</div>'; return; }
    el.innerHTML = list.map((r) => `
      <div class="rec-item">
        <span class="rec-name" title="${r.name}">${r.name.replace("rec_", "")}</span>
        <span class="rec-sz">${r.size_mb} MB</span>
        <a href="/recordings/${r.name}.avi" download>video</a>
        ${r.csv ? `<a href="/recordings/${r.name}.csv" download>data</a>` : ""}
        <button class="rec-del" data-name="${r.name}" title="Delete">✕</button>
      </div>`).join("");
    el.querySelectorAll(".rec-del").forEach((b) =>
      b.addEventListener("click", async () => {
        if (!confirm("Delete recording " + b.dataset.name + "?")) return;
        await fetch("/api/recordings/delete", {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ name: b.dataset.name }),
        });
        pollRecordings();
      }));
  } catch (e) {}
}

plBtn.addEventListener("click", async () => {
  const want = !plArmed;
  if (want && !confirm("Arm precision landing?\n\nThe Pi will start sending LANDING_TARGET to ArduPilot once a target is visible. Only use when landing over the pad.")) return;
  try {
    await fetch("/api/precland", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ armed: want }),
    });
    pollPrecland();
  } catch (e) { alert("Error: " + e); }
});

// ---- network (home WiFi vs 4G dongle) -------------------------------------
let netWifiEnabled = true;

async function pollNetwork() {
  try {
    const n = await (await fetch("/api/network", { cache: "no-store" })).json();
    netWifiEnabled = n.wifi_enabled;
    const a = $("net-active");
    a.textContent = n.active === "4g" ? "4G dongle"
      : n.active === "wifi" ? "WiFi (home)" : "none";
    a.className = "pill " + (n.active === "none" ? "pill-bad" : "pill-good");
    $("net-wifi").textContent = n.wifi_enabled ? (n.wifi_ssid || "on (not connected)") : "OFF";
    $("net-wwan").textContent = n.wwan_ip || "down";
    const btn = $("net-force4g");
    btn.textContent = n.wifi_enabled ? "Force 4G (turn off WiFi)" : "Turn WiFi back on";
    btn.classList.toggle("armed", !n.wifi_enabled);
    $("net-revert").textContent = n.revert_in != null
      ? `WiFi turns back on automatically in ~${Math.ceil(n.revert_in / 60)} min` : "";
  } catch (e) {}
}

$("net-force4g").addEventListener("click", async () => {
  const turnOff = netWifiEnabled;   // WiFi on now → button turns it off (forces 4G)
  if (turnOff && !confirm(
      "Turn off WiFi and force 4G?\n\nIf you're on this page via home WiFi you'll lose it — reach it via Tailscale (dronepi:8080) instead. WiFi turns back on automatically after ~10 min.")) return;
  try {
    await fetch("/api/wifi", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled: !turnOff }),
    });
    setTimeout(pollNetwork, 800);
  } catch (e) { alert("Error: " + e); }
});

// ---- RTK corrections -------------------------------------------------------
function fmtBps(b) {
  if (b == null) return "–";
  return b >= 1000 ? (b / 1000).toFixed(1) + " kB/s" : b + " B/s";
}

function fmtAge(s) {
  if (s == null) return "–";
  if (s < 10) return s.toFixed(1) + "s ago";
  s = Math.round(s);
  return s < 90 ? s + "s ago" : Math.floor(s / 60) + "m ago";
}

async function pollRtk() {
  try {
    const r = await (await fetch("/api/rtk", { cache: "no-store" })).json();
    const flowing = r.base_ok && r.base_bps > 0;
    const base = $("rtk-base");
    base.textContent = r.base_ok ? "connected" : "down";
    base.className = "pill " + (flowing ? "pill-good" : (r.base_ok ? "pill-warn" : "pill-bad"));
    $("rtk-bps").textContent = r.base_ok
      ? (r.base_bps > 0 ? fmtBps(r.base_bps) : "no flow")
      : (r.err || "–");
    const age = $("rtk-age");
    age.textContent = fmtAge(r.rtcm_age);
    age.style.color = r.rtcm_age == null ? "var(--bad)"
      : r.rtcm_age < 5 ? "var(--good)" : (r.rtcm_age < 20 ? "var(--warn)" : "var(--bad)");
    $("rtk-inj").textContent = r.injector ? "active" : "inactive";
    const fix = $("rtk-fix");
    fix.textContent = FIX[r.fix_type] || "–";
    fix.style.color = r.fix_type >= 5 ? "var(--good)" : (r.fix_type === 4 ? "var(--warn)" : "");
  } catch (e) {}
}

// ---- draggable splitter between video and panel (one per tab with video) --
function setupSplitter(view, splitter, storageKey) {
  if (!view || !splitter) return { applySaved() {} };
  let dragging = false, leftPx = parseInt(localStorage.getItem(storageKey), 10) || 0;

  function apply(px) {
    const bound = view.clientWidth;
    if (!bound) return;
    px = Math.max(240, Math.min(bound - 240, px));   // min video / min panel
    leftPx = px;
    view.style.setProperty("--left", px + "px");
    localStorage.setItem(storageKey, px);
  }
  function down(e) { dragging = true; view.classList.add("dragging"); if (e.cancelable) e.preventDefault(); }
  function move(e) {
    if (!dragging) return;
    apply((e.touches ? e.touches[0].clientX : e.clientX) - view.getBoundingClientRect().left);
    if (e.cancelable) e.preventDefault();
  }
  function up() { dragging = false; view.classList.remove("dragging"); }

  splitter.addEventListener("mousedown", down);
  splitter.addEventListener("touchstart", down, { passive: false });
  window.addEventListener("mousemove", move);
  window.addEventListener("touchmove", move, { passive: false });
  window.addEventListener("mouseup", up);
  window.addEventListener("touchend", up);
  window.addEventListener("resize", () => { if (leftPx && view.classList.contains("active")) apply(leftPx); });

  return { applySaved() { if (leftPx) apply(leftPx); } };
}

const splitters = {
  pilot: setupSplitter($("view-pilot"), $("splitter-pilot"), "splitLeft_pilot"),
  precland: setupSplitter($("view-precland"), $("splitter-precland"), "splitLeft_precland"),
};

// ---- tabs ------------------------------------------------------------------
// Video streams are attached/detached on tab switch (not left running in a hidden
// tab) so the idle camera stays stopped and no bandwidth is wasted — same on-demand
// principle as camera.py/camera_hq.py.
const tabBtns = document.querySelectorAll(".tab-btn");
const views = document.querySelectorAll(".view");
const videoEl = $("video"), videoHqEl = $("video-hq");

function showTab(name) {
  tabBtns.forEach((b) => b.classList.toggle("active", b.dataset.tab === name));
  views.forEach((v) => v.classList.toggle("active", v.dataset.view === name));
  localStorage.setItem("activeTab", name);

  if (name === "pilot") videoHqEl.src = "/video_hq.mjpg"; else videoHqEl.removeAttribute("src");
  if (name === "precland") videoEl.src = "/video.mjpg"; else videoEl.removeAttribute("src");

  const s = splitters[name];
  if (s) s.applySaved();
}

tabBtns.forEach((b) => b.addEventListener("click", () => showTab(b.dataset.tab)));

// ---- loop ------------------------------------------------------------------
showTab(localStorage.getItem("activeTab") || "pilot");
if (hudOn) drawHud("hud-pilot", 0, 0, 0);
drawADI("adi-precland", 0, 0);
pollTelemetry(); setInterval(pollTelemetry, 200);
pollStats(); setInterval(pollStats, 1000);
pollPrecland(); setInterval(pollPrecland, 500);
pollExposureHq();
pollRecordings(); setInterval(pollRecordings, 4000);
pollNetwork(); setInterval(pollNetwork, 3000);
pollRtk(); setInterval(pollRtk, 1000);   // 1 Hz → sub-second age still feels live
