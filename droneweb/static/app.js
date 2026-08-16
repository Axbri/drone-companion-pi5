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

// ---- precisionslandning ------------------------------------------------
let plArmed = false;
const plBtn = $("pl-arm"), plCard = $("precland-card");

async function pollPrecland() {
  try {
    const p = await (await fetch("/api/precland", { cache: "no-store" })).json();
    plArmed = !!p.armed;
    plBtn.textContent = plArmed ? "Avaktivera" : "Aktivera";
    plBtn.classList.toggle("armed", plArmed);
    plCard.classList.toggle("armed", plArmed);
    $("pl-phase").textContent = p.phase || (plArmed ? "…" : "av");
    $("pl-source").textContent = p.source || "–";
    $("pl-agl").textContent = p.agl != null ? Number(p.agl).toFixed(2) : "–";
    $("pl-offset").textContent = p.offset ? `${p.offset[0]}, ${p.offset[1]}` : "–";
    $("pl-tx").textContent = p.sent ? `skickar (${p.tx})` : (p.tx ? `paus (${p.tx})` : "–");
    $("pl-rf").textContent = p.rangefinder_ok ? "OK" : "ingen";
    updateRecBtn(p.recording);
    seedExposure(p.exposure);
  } catch (e) {}
}

// ---- exponering (fältjustering) ----------------------------------------
const expAuto = $("exp-auto"), expUs = $("exp-us"), expGain = $("exp-gain");
const expCard = $("exposure-card");
let expSeeded = false, expTimer = null;

function reflectExposureUI() {
  $("exp-us-val").textContent = expUs.value;
  $("exp-gain-val").textContent = Number(expGain.value).toFixed(1);
  expCard.classList.toggle("auto", expAuto.checked);
}

function sendExposure() {
  fetch("/api/exposure", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      auto: expAuto.checked,
      exposure_us: Number(expUs.value),
      gain: Number(expGain.value),
    }),
  }).catch(() => {});
}

function onExposureInput() {
  reflectExposureUI();
  clearTimeout(expTimer);
  expTimer = setTimeout(sendExposure, 120);   // debounce live-släpning
}

// seedar reglagen från serverns värden första gången (t.ex. tidigare inställt)
function seedExposure(exp) {
  if (!exp || expSeeded) return;
  expSeeded = true;
  expAuto.checked = !!exp.auto;
  expUs.value = exp.exposure_us;
  expGain.value = exp.gain;
  reflectExposureUI();
}

expAuto.addEventListener("change", () => { reflectExposureUI(); sendExposure(); });
expUs.addEventListener("input", onExposureInput);
expGain.addEventListener("input", onExposureInput);
reflectExposureUI();

// ---- inspelning --------------------------------------------------------
const recBtn = $("pl-rec");
let recording = false;

function updateRecBtn(rec) {
  recording = !!rec;
  recBtn.textContent = recording ? "■ Stoppa inspelning" : "● Spela in";
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
  } catch (e) { alert("Fel: " + e); }
});

async function pollRecordings() {
  try {
    const list = await (await fetch("/api/recordings", { cache: "no-store" })).json();
    const el = $("rec-list");
    if (!list.length) { el.innerHTML = '<div class="rec-empty">Inga inspelningar än</div>'; return; }
    el.innerHTML = list.map((r) => `
      <div class="rec-item">
        <span class="rec-name" title="${r.name}">${r.name.replace("rec_", "")}</span>
        <span class="rec-sz">${r.size_mb} MB</span>
        <a href="/recordings/${r.name}.avi" download>video</a>
        ${r.csv ? `<a href="/recordings/${r.name}.csv" download>data</a>` : ""}
        <button class="rec-del" data-name="${r.name}" title="Radera">✕</button>
      </div>`).join("");
    el.querySelectorAll(".rec-del").forEach((b) =>
      b.addEventListener("click", async () => {
        if (!confirm("Radera inspelning " + b.dataset.name + "?")) return;
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
  if (want && !confirm("Aktivera precisionslandning?\n\nPi:n börjar skicka LANDING_TARGET till ArduPilot när ett mål syns. Använd bara vid landning över plattan.")) return;
  try {
    await fetch("/api/precland", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ armed: want }),
    });
    pollPrecland();
  } catch (e) { alert("Fel: " + e); }
});

// ---- nätverk (hemma-WiFi vs 4G-dongle) ---------------------------------
let netWifiEnabled = true;

async function pollNetwork() {
  try {
    const n = await (await fetch("/api/network", { cache: "no-store" })).json();
    netWifiEnabled = n.wifi_enabled;
    const a = $("net-active");
    a.textContent = n.active === "4g" ? "4G-dongle"
      : n.active === "wifi" ? "WiFi (hemma)" : "ingen";
    a.className = "pill " + (n.active === "none" ? "pill-bad" : "pill-good");
    $("net-wifi").textContent = n.wifi_enabled ? (n.wifi_ssid || "på (ej ansluten)") : "AV";
    $("net-wwan").textContent = n.wwan_ip || "nere";
    const btn = $("net-force4g");
    btn.textContent = n.wifi_enabled ? "Tvinga 4G (stäng av WiFi)" : "Slå på WiFi igen";
    btn.classList.toggle("armed", !n.wifi_enabled);
    $("net-revert").textContent = n.revert_in != null
      ? `WiFi slås på automatiskt om ~${Math.ceil(n.revert_in / 60)} min` : "";
  } catch (e) {}
}

$("net-force4g").addEventListener("click", async () => {
  const turnOff = netWifiEnabled;   // WiFi på nu → knappen stänger av (tvingar 4G)
  if (turnOff && !confirm(
      "Stänga av WiFi och tvinga 4G?\n\nNår du sidan via hemma-WiFi tappar du den — öppna den då via Tailscale (dronepi:8080). WiFi slås på igen automatiskt efter ~10 min.")) return;
  try {
    await fetch("/api/wifi", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled: !turnOff }),
    });
    setTimeout(pollNetwork, 800);
  } catch (e) { alert("Fel: " + e); }
});

// ---- dragbar avdelare mellan video och panel ---------------------------
(function () {
  const splitter = $("splitter"), main = document.querySelector("main");
  if (!splitter || !main) return;
  let dragging = false, leftPx = parseInt(localStorage.getItem("splitLeft"), 10) || 0;

  function apply(px) {
    px = Math.max(240, Math.min(main.clientWidth - 240, px));   // min video / min panel
    leftPx = px;
    main.style.setProperty("--left", px + "px");
    localStorage.setItem("splitLeft", px);
  }
  function down(e) { dragging = true; main.classList.add("dragging"); if (e.cancelable) e.preventDefault(); }
  function move(e) {
    if (!dragging) return;
    apply((e.touches ? e.touches[0].clientX : e.clientX) - main.getBoundingClientRect().left);
    if (e.cancelable) e.preventDefault();
  }
  function up() { dragging = false; main.classList.remove("dragging"); }

  splitter.addEventListener("mousedown", down);
  splitter.addEventListener("touchstart", down, { passive: false });
  window.addEventListener("mousemove", move);
  window.addEventListener("touchmove", move, { passive: false });
  window.addEventListener("mouseup", up);
  window.addEventListener("touchend", up);
  window.addEventListener("resize", () => { if (leftPx) apply(leftPx); });   // håll inom skärmen
  if (leftPx) apply(leftPx);                                                 // återställ sparad position
})();

// ---- loop --------------------------------------------------------------
drawADI(0, 0);
pollTelemetry(); setInterval(pollTelemetry, 200);
pollStats(); setInterval(pollStats, 1000);
pollPrecland(); setInterval(pollPrecland, 500);
pollRecordings(); setInterval(pollRecordings, 4000);
pollNetwork(); setInterval(pollNetwork, 3000);
