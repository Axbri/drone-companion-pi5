"use strict";

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 0) => (v === undefined || v === null ? "–" : Number(v).toFixed(d));

const FIX = { 0: "Ingen", 1: "Ingen fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK-Float", 6: "RTK-Fixed" };

// ---- telemetri (~5 Hz) -------------------------------------------------
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

    $("volt").textContent = fmt(t.voltage, 2);
    $("curr").textContent = fmt(t.current, 1);
    $("batt").textContent = fmt(t.battery_remaining);
    setBar("batt-bar", t.battery_remaining, [20, 40], true);

    $("roll").textContent = fmt(t.roll, 1);
    $("pitch").textContent = fmt(t.pitch, 1);
    $("hdg").textContent = fmt(t.heading);
    $("alt").textContent = fmt(t.alt, 1);
    $("gps").textContent =
      (FIX[t.fix_type] || "–") + (t.satellites != null ? ` · ${t.satellites} sat` : "");

    drawADI(t.roll || 0, t.pitch || 0);
  } catch (e) {
    $("link").className = "pill pill-bad";
    $("link").textContent = "offline";
  }
}

// ---- Pi-stats (~1 Hz) --------------------------------------------------
async function pollStats() {
  try {
    const s = await (await fetch("/api/stats", { cache: "no-store" })).json();
    $("cpu").textContent = fmt(s.cpu_percent);
    setBar("cpu-bar", s.cpu_percent, [60, 85]);
    $("mem").textContent = `${s.mem_used_mb} / ${s.mem_total_mb} MB`;
    setBar("mem-bar", s.mem_percent, [75, 90]);
    $("net").textContent = `${fmt(s.net_rx_kbps, 0)} / ${fmt(s.net_tx_kbps, 0)}`;
    $("temp").textContent = fmt(s.temp_c, 1);
  } catch (e) {}
}

function setBar(id, pct, [warn, bad], invert = false) {
  const el = $(id);
  if (pct == null) return;
  el.style.width = Math.max(0, Math.min(100, pct)) + "%";
  let color = "var(--good)";
  if (invert) {
    if (pct <= warn) color = "var(--bad)";
    else if (pct <= bad) color = "var(--warn)";
  } else {
    if (pct >= bad) color = "var(--bad)";
    else if (pct >= warn) color = "var(--warn)";
  }
  el.style.background = color;
}

// ---- artificial horizon (roll/pitch) ----------------------------------
function drawADI(roll, pitch) {
  const c = $("adi");
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
  ctx.fillStyle = "#4a7fb5";                 // himmel
  ctx.fillRect(-r, -r * 2, w, r * 2);
  ctx.fillStyle = "#6b4a2f";                 // mark
  ctx.fillRect(-r, 0, w, r * 2);
  ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(-r, 0); ctx.lineTo(r, 0); ctx.stroke();
  ctx.restore();
  // fast flygsymbol
  ctx.strokeStyle = "#ffcc00"; ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(r - 22, r); ctx.lineTo(r - 6, r);
  ctx.moveTo(r + 6, r); ctx.lineTo(r + 22, r);
  ctx.moveTo(r, r - 6); ctx.lineTo(r, r + 4);
  ctx.stroke();
}

// ---- video-hint --------------------------------------------------------
$("video").addEventListener("load", () => ($("video-hint").style.display = "none"));
$("video").addEventListener("error", () => {
  $("video-hint").style.display = "block";
  $("video-hint").textContent = "Ingen video – kontrollera droneweb/kameran";
});

// ---- avstängning -------------------------------------------------------
$("shutdown").addEventListener("click", async () => {
  if (!confirm("Stänga av Raspberry Pi:n?\n\nDrönaren tappar då RTK, telemetri och video tills den startas om manuellt.")) return;
  try {
    await fetch("/api/shutdown", { method: "POST" });
    document.body.innerHTML =
      '<div style="display:grid;place-items:center;height:100vh;color:#8b98a8;font-family:system-ui">Pi:n stängs av…</div>';
  } catch (e) {
    alert("Avstängning misslyckades: " + e);
  }
});

// ---- loop --------------------------------------------------------------
drawADI(0, 0);
pollTelemetry(); setInterval(pollTelemetry, 200);
pollStats(); setInterval(pollStats, 1000);
