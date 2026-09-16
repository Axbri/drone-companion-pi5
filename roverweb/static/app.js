"use strict";
const $ = (id) => document.getElementById(id);
const fmt = (v, d = 1, u = "") => (v === null || v === undefined) ? "--" + (u ? " " + u : "") : v.toFixed(d) + (u ? " " + u : "");
const esc = (s) => String(s).replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]));

let drivable = false;   // true when armed + MANUAL/STEERING (set from telemetry)
let armedNow = false;   // armerad? används av avstängningsknappen (set from telemetry)
let vidSynced = false;  // one-time sync of the video dropdowns to the actual setting

/* ---------- WebSocket telemetry ---------- */
let ws, wsTimer;
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => setPill($("conn"), true, "ansluten", "frånkopplad");
  ws.onclose = () => { setPill($("conn"), false, "ansluten", "frånkopplad"); clearTimeout(wsTimer); wsTimer = setTimeout(connect, 1500); };
  ws.onmessage = (e) => {
    const d = JSON.parse(e.data);
    if (d.type === "mission") { drawMission(d); return; }
    if (d.type === "av") { renderAV(d); return; }
    if (d.type === "lidar") { renderLidar(d); return; }
    if (d.type === "fusion") { renderFusion(d); return; }
    render(d);
  };
}
function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

function setPill(el, ok, okTxt, badTxt, warn) {
  el.textContent = ok ? okTxt : badTxt;
  el.className = "pill " + (ok ? (warn ? "pill-warn" : "pill-good") : "pill-bad");
}

function render(s) {
  const l = s.link;
  armedNow = !!l.armed;
  $("t-mode").textContent = l.mode;
  $("t-armed").textContent = l.armed ? "ARMERAD" : "disarmed";
  $("t-armed").style.color = l.armed ? "var(--bad)" : "var(--good)";
  $("t-volt").textContent = fmt(s.batt.voltage, 2, "V");
  $("t-curr").textContent = fmt(s.batt.current, 1, "A");
  $("t-rem").textContent = s.batt.remaining == null ? "-- %" : s.batt.remaining + " %";
  $("t-speed").textContent = fmt(s.motion.speed, 1, "m/s");
  $("t-hdg").textContent = Math.round(s.motion.heading) + "°";
  $("t-fix").textContent = s.gps.fix_str;
  $("t-fix").style.color = s.gps.fix >= 6 ? "var(--good)" : (s.gps.fix >= 4 ? "var(--warn)" : "var(--bad)");
  $("t-sats").textContent = s.gps.sats;
  $("t-hdop").textContent = fmt(s.gps.hdop, 2);
  $("t-pos").textContent = (s.gps.lat == null) ? "--, --" : s.gps.lat.toFixed(6) + ", " + s.gps.lon.toFixed(6);

  $("p-temp").textContent = fmt(s.pi.temp, 1, "°C");
  $("p-cpu").textContent = fmt(s.pi.cpu, 0, "%");
  $("p-load").textContent = s.pi.load == null ? "--" : s.pi.load;
  $("p-ram").textContent = s.pi.ram
    ? `${s.pi.ram.used_gb.toFixed(1)}/${s.pi.ram.total_gb.toFixed(1)} GB (${s.pi.ram.pct}%)` : "--";
  if (s.net) {
    $("p-up").textContent = s.net.up_mbps.toFixed(2) + " Mbit/s";
    $("p-down").textContent = s.net.down_mbps.toFixed(2) + " Mbit/s";
    $("p-gbph").textContent = "~" + s.net.gbph.toFixed(2) + " GB/h";
    if ($("wifi-ssid") && s.net.ssid !== undefined) {   // live nuvarande nät i Nätverk-panelen
      $("wifi-ssid").textContent = s.net.ssid || "ej anslutet";
      $("wifi-sig").textContent = (s.net.signal == null) ? "--" : s.net.signal + " %";
    }
  }
  if (s.video && s.video.bitrate) $("vid-br").textContent = "mål ~" + (s.video.bitrate / 1e6).toFixed(1) + " Mbit/s";
  if (s.video && !vidSynced) {          // reflect the actual current setting once on load
    $("vid-res").value = `${s.video.w}x${s.video.h}`;
    $("vid-fps").value = String(s.video.fps);
    vidSynced = true;
  }
  setPill($("hb"), s.homebase.connected, "ansluten", "ej ansluten");

  // dpad enable state
  const canDrive = l.armed && (l.mode === "MANUAL" || l.mode === "STEERING");
  if (!canDrive && typeof anyInput === "function" && anyInput()) allStop();  // safety: lost drive mode
  drivable = canDrive;
  $("dpad").classList.toggle("disabled", !canDrive);
  $("drive-note").classList.toggle("hot", l.armed);

  // messages (STATUSTEXT / arm results)
  if (s.msgs) {
    $("msgs").innerHTML = s.msgs.slice(-5).reverse().map((m) => {
      const cls = m.sev <= 3 ? "sev-bad" : (m.sev <= 4 ? "sev-warn" : "sev-dim");
      return `<div class="msg ${cls}">${esc(m.text)}</div>`;
    }).join("");
  }

  $("t-curwp").textContent = (s.cur_wp === null || s.cur_wp === undefined) ? "--" : s.cur_wp;
  if (s.cur_wp !== undefined && s.cur_wp !== lastCurWp) { lastCurWp = s.cur_wp; drawCurWp(s.cur_wp); }

  if (s.bumper) renderBumper(s.bumper, s.escape);

  updateMap(s);
}

/* Bumper + flyktmanöver. Kommer i huvud-telemetrin (state), inte som eget
   ws-meddelande. Röd prick = träff (eller kabelbrott — fail-safe). */
let bumperSynced = false;   // engångs-sync av kryssrutan till serverns enable
function renderBumper(b, e) {
  const st = $("bumper-state");
  if (!b.ok) { st.textContent = "ingen GPIO"; st.className = "pill pill-bad"; }
  else if (b.enable) { st.textContent = "auto-flykt på"; st.className = "pill pill-good"; }
  else { st.textContent = "auto-flykt av"; st.className = "pill pill-dim"; }
  $("bump-l").classList.toggle("hit", !!b.left);
  $("bump-r").classList.toggle("hit", !!b.right);
  if (!bumperSynced) { $("bumper-enable").checked = !!b.enable; bumperSynced = true; }
  if (e) {
    const PH = { IDLE: "vilar", REVERSE: "backar", TURN: "svänger", RESUME: "återupptar" };
    const el = $("esc-phase");
    el.textContent = PH[e.phase] || e.phase;
    el.style.color = e.active ? "var(--warn)" : "";
    $("esc-count").textContent = e.count;
  }
}

/* ---------- Controls ---------- */
$("btn-arm").onclick = () => { if (confirm("Armera rovern? Den kan börja röra sig.")) send({ cmd: "arm", value: true }); };
$("btn-disarm").onclick = () => send({ cmd: "arm", value: false });
$("btn-mode").onclick = () => send({ cmd: "mode", value: $("mode-sel").value });

/* Avstängning av Pi:n. Servern vägrar när rovern är armerad — men vi visar det
   redan här, så att man slipper klicka först och få nej sen. */
$("btn-shutdown").onclick = () => {
  const b = $("btn-shutdown");
  if (b.disabled) return;
  if (armedNow) {
    alert("Rovern är ARMERAD. Disarmera först — går Pi:n ned tar den med sig\n" +
          "video, telemetri och den manuella körningen.");
    return;
  }
  if (!confirm("Stänga av companion-datorn?\n\n" +
               "Video, telemetri och webbgränssnittet försvinner. Rovern måste\n" +
               "strömsättas manuellt för att komma tillbaka.\n\n" +
               "Vänta tills lysdioden slocknat innan du bryter strömmen.")) return;
  send({ cmd: "shutdown", confirm: "SHUTDOWN" });
  b.disabled = true;
  b.textContent = "Stänger av ...";
};
$("gain").addEventListener("input", () => { $("gain-val").textContent = $("gain").value + "%"; });

/* AUTO: jump the mission to a given waypoint */
function setWp(v) {
  if (Number.isFinite(v) && v >= 0) send({ cmd: "set_wp", value: v });
}
$("btn-setwp").onclick = () => setWp(parseInt($("wp-input").value, 10));
$("btn-restart").onclick = () => setWp(1);
$("wp-input").addEventListener("keydown", (e) => {
  if (e.key === "Enter") { e.preventDefault(); setWp(parseInt($("wp-input").value, 10)); }
});

/* ---------- WiFi / nätverk ----------
   Nuvarande SSID + signal kommer live via telemetrin (render). IP, sparade nät
   och scan hämtas via HTTP (/wifi/*). "Lägg till" sparar bara ett nät (kopplar
   inte om). "Anslut" byter nät med serverns auto-återgång. */
function wifiMsg(t, bad) {
  const el = $("wifi-msg"); if (!el) return;
  el.textContent = t; el.className = "note " + (bad ? "sev-bad" : "dim");
}
async function wifiList() {
  try {
    const d = await (await fetch("/wifi/list")).json();
    const c = d.current || {};
    if ($("wifi-ip")) $("wifi-ip").textContent = c.ip || "--";
    if (c.ssid && $("wifi-ssid")) $("wifi-ssid").textContent = c.ssid;
    $("wifi-saved").innerHTML = (d.saved || []).map((s) => {
      const active = c.name && s.name === c.name;
      return `<div class="wifi-row${active ? " active" : ""}">
        <span class="wifi-name">${esc(s.name)}${active ? ' <span class="pill pill-good">aktiv</span>' : ""}</span>
        <span class="wifi-btns">${active ? "" :
          `<button class="btn small" data-wc="${esc(s.name)}">Anslut</button>` +
          `<button class="btn small btn-disarm" data-wf="${esc(s.name)}" title="Radera">✕</button>`}</span>
      </div>`;
    }).join("") || '<div class="note dim">Inga sparade nät.</div>';
    $("wifi-scan-sel").innerHTML = '<option value="">— välj ur scan eller skriv nedan —</option>' +
      (d.available || []).map((a) => `<option value="${esc(a.ssid)}">${esc(a.ssid)} · ${a.signal}% · ${esc(a.security)}</option>`).join("");
  } catch (e) { wifiMsg("Kunde inte läsa nätverk: " + e, true); }
}
$("wifi-refresh").onclick = () => { wifiMsg("Scannar …"); wifiList().then(() => wifiMsg("")); };
$("wifi-scan-sel").onchange = () => { const v = $("wifi-scan-sel").value; if (v) $("wifi-ssid-in").value = v; };
$("wifi-add-btn").onclick = async () => {
  const ssid = $("wifi-ssid-in").value.trim(), password = $("wifi-pass-in").value;
  if (!ssid) { wifiMsg("Ange ett SSID.", true); return; }
  wifiMsg("Sparar '" + ssid + "' …");
  try {
    const d = await (await fetch("/wifi/add", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ ssid, password }) })).json();
    if (d.ok) { wifiMsg("Nätet '" + ssid + "' sparat — autoansluter när det är i räckvidd."); $("wifi-pass-in").value = ""; $("wifi-ssid-in").value = ""; wifiList(); }
    else wifiMsg("Kunde inte spara: " + (d.error || "okänt fel"), true);
  } catch (e) { wifiMsg("Fel: " + e, true); }
};
$("wifi-saved").addEventListener("click", async (e) => {
  const cn = e.target.closest("[data-wc]"), fn = e.target.closest("[data-wf]");
  if (cn) {
    const name = cn.getAttribute("data-wc");
    if (!confirm("Byta till '" + name + "'?\n\nPi:ns nät kopplas om. Ger det nya nätet inte internet\nåtergår den AUTOMATISKT till nuvarande nät efter ~25 s.\nFörbindelsen kan blinka till under bytet.")) return;
    wifiMsg("Byter till '" + name + "' … (auto-återgång om det misslyckas)");
    try { await fetch("/wifi/connect", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name }) }); } catch (e2) { /* svaret kan tappas när nätet kopplas om — väntat */ }
    setTimeout(wifiList, 30000);
  }
  if (fn) {
    const name = fn.getAttribute("data-wf");
    if (!confirm("Radera sparat nät '" + name + "'?")) return;
    try {
      const d = await (await fetch("/wifi/forget", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name }) })).json();
      if (d.ok) wifiList(); else wifiMsg("Kunde inte radera: " + (d.error || ""), true);
    } catch (e3) { wifiMsg("Fel: " + e3, true); }
  }
});
wifiList();

/* Drive: multiple simultaneous inputs (steer + throttle), dead-man, release = stop.
   Sources: D-pad buttons (pointer, multi-touch) and keyboard (arrows / WASD). */
const inputs = { fwd: false, back: false, left: false, right: false };
let driveTimer = null;

function driveVector() {
  return {
    throttle: (inputs.fwd ? 1 : 0) + (inputs.back ? -1 : 0),
    steer: (inputs.right ? 1 : 0) + (inputs.left ? -1 : 0),
  };
}
const anyInput = () => inputs.fwd || inputs.back || inputs.left || inputs.right;

function gain() { return (parseInt($("gain").value, 10) || 100) / 100; }

function pushDrive() {
  const { steer, throttle } = driveVector();
  const g = gain();
  send({ cmd: "drive", steer: steer * g, throttle: throttle * g });
  document.querySelectorAll(".drive").forEach((b) => {
    const d = b.dataset.dir;
    b.classList.toggle("held", d !== "stop" && inputs[d]);
  });
}
function ensureLoop() {
  if (driveTimer) return;
  driveTimer = setInterval(() => {
    if (anyInput()) pushDrive();
    else { clearInterval(driveTimer); driveTimer = null; }
  }, 100);   // 10 Hz keep-alive so the server watchdog stays fed
}
function setInput(dir, on) {
  if (on && !drivable) return;          // only when armed + MANUAL/STEERING
  if (inputs[dir] === on) return;       // ignore key auto-repeat
  inputs[dir] = on;
  pushDrive();                          // send updated vector immediately (incl. 0,0)
  if (anyInput()) ensureLoop();
}
function allStop() {
  inputs.fwd = inputs.back = inputs.left = inputs.right = false;
  pushDrive();
}

document.querySelectorAll(".drive").forEach((b) => {
  const dir = b.dataset.dir;
  const press = (e) => { e.preventDefault(); if (dir === "stop") allStop(); else setInput(dir, true); };
  const release = () => { if (dir !== "stop") setInput(dir, false); };
  b.addEventListener("pointerdown", press);
  b.addEventListener("pointerup", release);
  b.addEventListener("pointerleave", release);
  b.addEventListener("pointercancel", release);
});

const KEYMAP = {
  ArrowUp: "fwd", KeyW: "fwd", ArrowDown: "back", KeyS: "back",
  ArrowLeft: "left", KeyA: "left", ArrowRight: "right", KeyD: "right",
};
addEventListener("keydown", (e) => {
  const dir = KEYMAP[e.code];
  if (!dir || e.repeat) return;
  if (e.target.matches("input, select, textarea")) return;
  e.preventDefault();
  setInput(dir, true);
});
addEventListener("keyup", (e) => { const dir = KEYMAP[e.code]; if (dir) setInput(dir, false); });
addEventListener("blur", allStop);   // window loses focus -> stop

/* ---------- Video (WHEP from mediamtx) ---------- */
let videoPC = null;
function stopVideo() {                       // end the WebRTC session server-side right away
  if (videoPC) { try { videoPC.close(); } catch (e) { /* ignore */ } videoPC = null; }
}
addEventListener("pagehide", stopVideo);     // reload/close must not leave a reader behind

/* Vy-läge (V4.6): panelen visar antingen "video" (WebRTC-strömmen) eller
   "hinder" (stereons /stereo.jpg). Sedan stereo och mediamtx samexisterar
   finns BÅDA samtidigt, så det är operatörens val. Deklarerad här uppe så
   startVideo/renderAV aldrig läser den före deklarationen. */
let viewMode = "video";
/* Stereobildens tillstånd — deklareras HÄR, före setViewMode()/booten som
   anropar showStereoImage(). Låg ner (vid funktionen) hamnade det i temporal
   dead zone och kraschade hela boot-scriptet. */
let stereoImgTimer = null, stereoImgWanted = false;

/* Radarn ritar två oberoende lager i samma diagram: kameran (gröna prickar,
   ~90° framåt) och 360°-lidarn (blå prickar). De kommer i skilda
   websocket-meddelanden ("av" resp. "lidar") och uppdateras var för sig, så det
   senaste av varje hålls här och drawRadar() läser båda. Deklarerat HÖGT UPP:
   drawRadar() anropas redan vid boot, en let längre ner hade legat i temporal
   dead zone (samma fälla som stereoImg ovan). */
let radar = { av: null, avStale: true, lidar: null, lidarStale: true,
              fusion: null, fusionStale: true };
/* Av/på per radarlager, styrs av kryssrutorna under radarn. */
let radarShow = { lidar: true, cam: true, fusion: true };

async function startVideo() {
  stopVideo();                               // never run two sessions at once
  const vEl = $("video"), st = $("vid-state");
  try {
    const pc = new RTCPeerConnection();
    videoPC = pc;
    pc.addTransceiver("video", { direction: "recvonly" });
    pc.ontrack = (e) => { vEl.srcObject = e.streams[0]; setPill(st, true, "live", ""); };
    pc.onconnectionstatechange = () => {
      if (pc !== videoPC) return;            // stale callback from an old session
      if (["failed", "disconnected", "closed"].includes(pc.connectionState)) {
        setPill(st, false, "", "återansluter…");
        setTimeout(() => {
          if (pc === videoPC && viewMode === "video") startVideo();
        }, 2000);
      }
    };
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    await new Promise((r) => {
      if (pc.iceGatheringState === "complete") return r();
      pc.addEventListener("icegatheringstatechange", () => pc.iceGatheringState === "complete" && r());
      setTimeout(r, 800);
    });
    const res = await fetch(`http://${location.hostname}:8889/rover/whep`, {
      method: "POST", headers: { "Content-Type": "application/sdp" }, body: pc.localDescription.sdp,
    });
    await pc.setRemoteDescription({ type: "answer", sdp: await res.text() });
  } catch (err) {
    /* mediamtx nere eller ingen publicerare — försök igen, men bara om
       operatören fortfarande vill se video (annars vore det bortkastat). */
    setPill(st, false, "", "ingen video");
    setTimeout(() => { if (viewMode === "video") startVideo(); }, 3000);
  }
}

/* Väljaren video/hinderbild. Byter vilket element panelen visar och
   startar/stoppar rätt källa. */
function setViewMode(mode) {
  viewMode = mode;
  $("view-video").classList.toggle("on", mode === "video");
  $("view-hinder").classList.toggle("on", mode === "hinder");
  if (mode === "video") {
    showStereoImage(false);
    startVideo();
  } else {
    stopVideo();                              // släpp WebRTC-strömmen, spar band
    setPill($("vid-state"), false, "", "hinderbild");
    showStereoImage(true);
  }
}
$("view-video").addEventListener("click", () => setViewMode("video"));
$("view-hinder").addEventListener("click", () => setViewMode("hinder"));

/* Lidarns motorstyrning. Start skickar MotorOn + SetRPM 300 (tjänsten sköter
   båda), Stopp skickar MotorOff. Funkar oavsett om Hinderdata är på, men live-
   siffrorna syns bara när Hinderdata strömmar. */
$("lidar-start").addEventListener("click", () => send({ cmd: "lidar_motor", value: "on" }));
$("lidar-stop").addEventListener("click", () => send({ cmd: "lidar_motor", value: "off" }));

/* Bumper: auto-flykt på/av + bänktest. Testet varnar först — det armerar inget
   men kör hela sekvensen (mode-byten + RC-override), så hjulen ska vara upp. */
$("bumper-enable").addEventListener("change", () =>
  send({ cmd: "bumper_enable", value: $("bumper-enable").checked }));
$("escape-test").addEventListener("click", () => {
  if (!confirm("Testa flyktsekvensen?\n\nDetta kör backning -> sväng -> lägesbyte till AUTO\n" +
               "med RC-override. HJULEN SKA VARA UPP. Disarmera för att avbryta.")) return;
  send({ cmd: "escape_test", side: "left" });
});

/* Kryssrutor under radarn: av/på per lager (blå lidar, grön kamera, röd = det som
   skickas till ArduPilot). Ritar bara om — datan fortsätter komma. */
for (const [id, key] of [["show-lidar", "lidar"], ["show-cam", "cam"], ["show-fusion", "fusion"]]) {
  $(id).addEventListener("change", () => { radarShow[key] = $(id).checked; drawRadar(); });
}

/* Video-vakthund. Det första startVideo() vid boot kan tyst missa att fästa
   strömmen (kapplöpning innan mediamtx-WHEP svarar rent), och ontrack/catch
   uppdaterar då aldrig läget. En återkommande koll som startar om videon när
   den INTE är live täcker det, och dessutom mediamtx-omstarter och tappade
   anslutningar — allt med en mekanism. Billig: en koll var 3:e sekund. */
function videoIsLive() {
  const v = $("video");
  return v.srcObject && v.srcObject.getTracks().some(t => t.readyState === "live");
}
setInterval(() => {
  if (viewMode !== "video" || videoIsLive()) return;
  const st = videoPC && videoPC.connectionState;
  if (!videoPC || ["failed", "disconnected", "closed"].includes(st)) startVideo();
}, 3000);

/* ---------- Video quality controls ---------- */
function sendVideo() {
  const [w, h] = $("vid-res").value.split("x").map(Number);
  send({ cmd: "set_video", w, h, fps: parseInt($("vid-fps").value, 10) });
}
$("vid-res").addEventListener("change", sendVideo);
$("vid-fps").addEventListener("change", sendVideo);

/* ---------- Map (Leaflet + OSM / Esri satellite) ---------- */
let map, roverMarker, trail, missionLayer, currentBase, centered = false, lastMapTs = 0;
let fenceLayer, rallyLayer, ringsLayer, curWpLayer;
// maxNativeZoom = deepest zoom the provider actually has; beyond it Leaflet upscales
// the last real tiles (pixelated) instead of showing "map data not available".
const BASE = {
  map: L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
    { zIndex: 1, maxNativeZoom: 19, maxZoom: 21, attribution: "© OpenStreetMap" }),
  sat: L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    { zIndex: 1, maxNativeZoom: 18, maxZoom: 21, attribution: "Imagery © Esri, Maxar, Earthstar Geographics" }),
};
BASE.topo = L.tileLayer("https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
  { zIndex: 1, maxNativeZoom: 17, maxZoom: 21, attribution: "© OpenTopoMap (CC-BY-SA)" });
BASE.cycl = L.tileLayer("https://{s}.tile-cyclosm.openstreetmap.fr/cyclosm/{z}/{x}/{y}.png",
  { zIndex: 1, maxNativeZoom: 20, maxZoom: 21, attribution: "© CyclOSM, © OpenStreetMap" });
// Transparent overlays; zIndex keeps them above whichever base is active
const SEAMARK = L.tileLayer("https://tiles.openseamap.org/seamark/{z}/{x}/{y}.png",
  { zIndex: 10, maxNativeZoom: 18, maxZoom: 21, attribution: "© OpenSeaMap" });
const HILLSHADE = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/Elevation/World_Hillshade/MapServer/tile/{z}/{y}/{x}",
  { zIndex: 5, opacity: 0.45, maxNativeZoom: 16, maxZoom: 21, attribution: "Hillshade © Esri" });
function initMap() {
  map = L.map("map", { zoomControl: true }).setView([58.6005, 16.1319], 17);
  currentBase = BASE.map;
  currentBase.addTo(map);
  $("basemap").addEventListener("change", () => {
    map.removeLayer(currentBase);
    currentBase = BASE[$("basemap").value] || BASE.map;
    currentBase.addTo(map);
  });
  $("seamarks").addEventListener("change", () => {   // independent of the chosen base
    if ($("seamarks").checked) SEAMARK.addTo(map);
    else map.removeLayer(SEAMARK);
  });
  $("relief").addEventListener("change", () => {
    if ($("relief").checked) HILLSHADE.addTo(map);
    else map.removeLayer(HILLSHADE);
  });

  fenceLayer = L.layerGroup();
  rallyLayer = L.layerGroup();
  ringsLayer = L.layerGroup();
  curWpLayer = L.layerGroup().addTo(map);        // current-target ring is always shown
  const toggles = [["lyr-fence", () => fenceLayer], ["lyr-rally", () => rallyLayer],
                   ["lyr-rings", () => ringsLayer]];
  toggles.forEach(([id, get]) => {
    const apply = () => { if ($(id).checked) get().addTo(map); else map.removeLayer(get()); };
    $(id).addEventListener("change", apply);
    apply();                                     // honour the initial checked state
  });
  trail = L.polyline([], { color: "#ff2d2d", weight: 3 }).addTo(map);   // rover trail (red)
  missionLayer = L.layerGroup().addTo(map);
  map.createPane("roverPane");                       // dedicated pane so the rover marker
  map.getPane("roverPane").style.zIndex = 650;       // is ALWAYS above waypoint markers (600)
  map.on("dragstart", () => { $("follow").checked = false; });   // panning turns off follow
}
const ROUTE = { color: "#2d8fff", weight: 2.5, dashArray: "6,6" };
const FENCE_LINE = { color: "#a855f7", weight: 2.5, dashArray: "6,6", fill: false };
const FENCE_DOT = { radius: 4, color: "#a855f7", weight: 2,
                    fillColor: "#a855f7", fillOpacity: 1 };
// exclusion (keep-out) zones: red dashed outline + diagonal hatch fill (SVG pattern).
// fillOpacity here is only the fallback if the pattern can't be applied.
const FENCE_EXCL = { color: "#ff3b30", weight: 2.5, dashArray: "6,6",
                     fillColor: "#ff3b30", fillOpacity: 0.12 };
const FENCE_DOT_EXCL = { radius: 4, color: "#ff3b30", weight: 2,
                         fillColor: "#ff3b30", fillOpacity: 1 };

const SVGNS = "http://www.w3.org/2000/svg";
function ensureHatch(svg) {
  if (!svg || svg.querySelector("#hatch-excl")) return;
  let defs = svg.querySelector("defs");
  if (!defs) { defs = document.createElementNS(SVGNS, "defs"); svg.insertBefore(defs, svg.firstChild); }
  const pat = document.createElementNS(SVGNS, "pattern");
  pat.setAttribute("id", "hatch-excl");
  pat.setAttribute("patternUnits", "userSpaceOnUse");
  pat.setAttribute("width", "8");
  pat.setAttribute("height", "8");
  pat.setAttribute("patternTransform", "rotate(45)");
  const line = document.createElementNS(SVGNS, "line");
  line.setAttribute("x1", "0"); line.setAttribute("y1", "0");
  line.setAttribute("x2", "0"); line.setAttribute("y2", "8");
  line.setAttribute("stroke", "#ff3b30");
  line.setAttribute("stroke-width", "2.5");
  pat.appendChild(line);
  defs.appendChild(pat);
}
function applyHatch(layer) {
  const path = layer && layer._path;
  if (!path || !path.ownerSVGElement) return;
  ensureHatch(path.ownerSVGElement);
  path.setAttribute("fill", "url(#hatch-excl)");
  path.setAttribute("fill-opacity", "1");
}
let missionData = { items: [], jumps: [], fence: [], rally: [] };
let lastCurWp = null;

function drawFence(shapes) {
  fenceLayer.clearLayers();
  (shapes || []).forEach((s) => {
    const excl = s.inclusion === false;                       // keep-out zone
    const line = excl ? FENCE_EXCL : FENCE_LINE;
    const dot = excl ? FENCE_DOT_EXCL : FENCE_DOT;
    if (s.type === "polygon" && s.points.length > 1) {
      const poly = L.polygon(s.points, line)                  // dashed, closes the loop
        .bindTooltip(excl ? "Fence (förbjudet område)" : "Fence (tillåtet område)");
      if (excl) poly.on("add", () => applyHatch(poly));       // also when the layer is toggled on
      poly.addTo(fenceLayer);
      if (excl) applyHatch(poly);
      s.points.forEach((p) => L.circleMarker(p, dot).addTo(fenceLayer));
    } else if (s.type === "circle") {
      const circ = L.circle([s.lat, s.lon], Object.assign({ radius: s.radius }, line))
        .bindTooltip(`Fence cirkel ${Math.round(s.radius)} m (${excl ? "förbjuden" : "tillåten"})`);
      if (excl) circ.on("add", () => applyHatch(circ));
      circ.addTo(fenceLayer);
      if (excl) applyHatch(circ);
    } else if (s.type === "return") {
      L.circleMarker([s.lat, s.lon], FENCE_DOT).addTo(fenceLayer)
        .bindTooltip("Fence return point");
    }
  });
}

function drawRally(pts) {
  rallyLayer.clearLayers();
  (pts || []).forEach((r) => {
    L.marker([r.lat, r.lon], { icon: L.divIcon({ className: "", iconSize: [26, 22],
      iconAnchor: [13, 11], html: `<div class="wp-marker rally">R${r.seq}</div>` }) })
      .addTo(rallyLayer).bindTooltip("Rally " + r.seq);
  });
}

function drawRings() {
  ringsLayer.clearLayers();
  const home = missionData.items.find((w) => w.seq === 0);
  if (!home) return;
  [50, 100, 200].forEach((r) => {
    L.circle([home.lat, home.lon], { radius: r, color: "#8aa0b3", weight: 1,
      dashArray: "3,6", fill: false }).addTo(ringsLayer).bindTooltip(r + " m");
  });
}

function drawCurWp(seq) {
  if (!curWpLayer) return;
  curWpLayer.clearLayers();
  const w = missionData.items.find((x) => x.seq === seq);
  if (!w || seq === 0) return;
  L.circleMarker([w.lat, w.lon], { radius: 15, color: "#2d8fff", weight: 3, fill: false })
    .addTo(curWpLayer).bindTooltip("Kör mot WP " + seq);
}

function drawMission(d) {
  if (!missionLayer) return;
  missionData = { items: d.items || [], jumps: d.jumps || [],
                  fence: d.fence || [], rally: d.rally || [] };
  const items = missionData.items, jumps = missionData.jumps;
  missionLayer.clearLayers();
  const pts = [], bySeq = {};
  items.forEach((w) => {
    const ll = [w.lat, w.lon]; pts.push(ll); bySeq[w.seq] = ll;
    const home = w.seq === 0;   // ArduPilot mission item 0 is Home, not a real waypoint
    const icon = L.divIcon({
      className: "",
      iconSize: home ? [46, 22] : [22, 22],
      iconAnchor: home ? [23, 11] : [11, 11],
      html: home ? `<div class="wp-marker home">Home</div>` : `<div class="wp-marker">${w.seq}</div>`,
    });
    L.marker(ll, { icon }).addTo(missionLayer).bindTooltip(home ? "Home" : "WP " + w.seq);
  });
  if (pts.length > 1) L.polyline(pts, ROUTE).addTo(missionLayer);
  // DO_JUMP: draw the loop-back leg from the last waypoint before the jump to its target
  (jumps || []).forEach((j) => {
    const before = items.filter((w) => w.seq < j.seq).pop();
    const target = bySeq[j.target];
    if (before && target) {
      L.polyline([[before.lat, before.lon], target], ROUTE).addTo(missionLayer)
        .bindTooltip(`DO_JUMP → WP ${j.target}`);
    }
  });
  drawFence(missionData.fence);
  drawRally(missionData.rally);
  drawRings();
  drawCurWp(lastCurWp);
}
$("btn-readmission").onclick = () => send({ cmd: "read_mission" });
$("btn-clearwp").onclick = () => {          // clear everything drawn from the mission read
  [missionLayer, fenceLayer, rallyLayer, ringsLayer, curWpLayer]
    .forEach((l) => l && l.clearLayers());
  missionData = { items: [], jumps: [], fence: [], rally: [] };
  lastCurWp = null;
};
function updateMap(s) {
  if (!map || s.gps.lat == null) return;
  const now = Date.now();
  if (now - lastMapTs < 500) return;      // throttle to 2 Hz
  lastMapTs = now;
  const ll = [s.gps.lat, s.gps.lon];
  const icon = L.divIcon({
    className: "", html: `<div class="rover-marker" style="transform:rotate(${s.motion.heading - 90}deg)">➤</div>`,
    // -90: tecknet ➤ pekar åt HÖGER i grundläget medan heading 0 betyder
    // norr, alltså uppåt. Utan korrigeringen pekade pilen 90° fel.
    iconSize: [24, 24], iconAnchor: [12, 12],
  });
  if (!roverMarker) roverMarker = L.marker(ll, { icon, pane: "roverPane" }).addTo(map);
  else { roverMarker.setLatLng(ll); roverMarker.setIcon(icon); }
  const pts = trail.getLatLngs(); pts.push(ll);
  if (pts.length > 500) pts.shift();
  trail.setLatLngs(pts);
  if ($("follow").checked) {           // only recenter when "Följ rover" is ticked
    map.setView(ll, centered ? map.getZoom() : 18);
    centered = true;
  }
}

/* ---------- resizable splitters ---------- */
function makeResizer(id, read, apply) {
  const sp = $(id);
  sp.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    sp.classList.add("drag");
    const x0 = e.clientX, v0 = read();
    const move = (ev) => { apply(v0 + (ev.clientX - x0)); if (map) map.invalidateSize(); };
    const up = () => {
      document.removeEventListener("pointermove", move);
      document.removeEventListener("pointerup", up);
      sp.classList.remove("drag"); document.body.style.cursor = "";
      if (map) map.invalidateSize();
    };
    document.addEventListener("pointermove", move);
    document.addEventListener("pointerup", up);
    document.body.style.cursor = "col-resize";
  });
}
const left = document.querySelector(".col-left");
const vid = document.querySelector(".video-panel");
makeResizer("split1", () => left.offsetWidth, (w) => { left.style.width = Math.max(240, Math.min(720, w)) + "px"; });
makeResizer("split2", () => vid.offsetWidth, (w) => { vid.style.flex = "0 0 " + Math.max(150, w) + "px"; });

/* Lodrätt draghandtag för radarns storlek. Samma mönster som makeResizer men i
   Y-led: att dra UPPÅT (mot videon) gör radarn större, nedåt mindre. Radarn är
   kvadratisk, så storleken sätts som bredd via CSS-variabeln --radar-w på panelen
   (höjden följer). Videon (.vid-wrap, flex:1) krymper/växer med det som blir kvar. */
function makeResizerV(id, read, apply) {
  const sp = $(id);
  if (!sp) return;
  sp.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    sp.classList.add("drag");
    const y0 = e.clientY, v0 = read();
    const move = (ev) => apply(v0 + (y0 - ev.clientY));   // uppåt = större
    const up = () => {
      document.removeEventListener("pointermove", move);
      document.removeEventListener("pointerup", up);
      sp.classList.remove("drag"); document.body.style.cursor = "";
    };
    document.addEventListener("pointermove", move);
    document.addEventListener("pointerup", up);
    document.body.style.cursor = "row-resize";
  });
}
const radarEl = $("cv-radar");
makeResizerV("split-radar", () => radarEl.offsetWidth, (size) => {
  // Övre gränsen hålls inom panelen så dragningen inte "fastnar" mot max-width:100%
  // (då hade read() och den satta bredden glidit isär och radarn ryckt).
  const max = Math.max(240, vid.clientWidth - 24);
  vid.style.setProperty("--radar-w", Math.max(220, Math.min(max, size)) + "px");
});

/* ---------- boot ---------- */
initMap();
connect();
setViewMode("video");    // startar i videoläge; väljaren byter till hinderbild


/* ---------- CV: överlägg på videon + proximity-radar ---------- */
/* Överlägget ritas i normaliserade koordinater som roverav skickar (0..1 av
   bildrutan). Servern behöver därför inte veta något om videoupplösning, och
   överlägget sitter rätt även om kvaliteten byts under drift. */

$("cv-on").addEventListener("change", () => {
  const on = $("cv-on").checked;
  send({ cmd: "set_overlay", value: on });
  if (!on) {
    const c = $("cv-overlay");
    c.getContext("2d").clearRect(0, 0, c.width, c.height);
    /* Datan slutar komma när rutan kryssas ur. Utan detta blir radarn stående
       med sin sista bild och ser ut att visa nuläget — värre än att vara tom. */
    setPill($("cv-state"), false, "", "avstängd");
    ["cv-near", "cv-sec", "cv-surf", "cv-fps"].forEach((id) => $(id).textContent = "--");
    /* Alla dataströmmar stannar när överlägget kryssas ur — töm alla lager så
       radarn blir tom i stället för att frysa fast. */
    radar.av = null; radar.avStale = true; radar.lidar = null; radar.lidarStale = true;
    radar.fusion = null; radar.fusionStale = true;
    drawRadar();
    updateLidarInfo(null);            // töm lidar-siffrorna med
  }
});

/* object-fit: contain brevlådar videon inuti elementet. Ritar man rakt på
   elementets yta hamnar överlägget fel så fort bildens proportioner skiljer
   sig från panelens — därför räknas den faktiskt ritade videorutan ut här. */
function videoRect(v, el) {
  const ew = el.clientWidth, eh = el.clientHeight;
  const vw = v.videoWidth || 4, vh = v.videoHeight || 3;
  const scale = Math.min(ew / vw, eh / vh);
  const w = vw * scale, h = vh * scale;
  return { x: (ew - w) / 2, y: (eh - h) / 2, w, h };
}

/* Två detektorer delar den här vyn: V3 (roverav, färgbaserad) och V4
   (roverstereo). De kan inte köra samtidigt — båda vill ha cam0 — så vyn
   visar den som faktiskt skriver data, och SÄGER vilken det är. Fälten
   betyder olika saker för de två; att blanda ihop dem vore värre än att inte
   visa dem alls. */
function renderAV(d) {
  const stale = d.age > 2.0;
  const stereo = d.source === "stereo";
  setPill($("cv-state"), !stale && !d.blind, stale ? "gammal" : "aktiv",
          stale ? "ingen data" : "blind", !!d.blind);
  const src = $("cv-src");
  src.textContent = stale ? "--" : (stereo ? "V4 stereo" : "V3 färg");
  src.className = "pill " + (stale ? "pill-dim" : (stereo ? "pill-good" : "pill-warn"));

  $("cv-near").textContent = d.nearest_m ? d.nearest_m.toFixed(2) + " m" : "fritt";
  $("cv-sec").textContent = `${d.blocked} / 72` +
    (d.raw !== d.blocked ? ` (rå ${d.raw})` : "");
  $("cv-fps").textContent = d.fps ? d.fps.toFixed(1) + " Hz" : "--";

  /* Färgdetektorns underlagsminne finns inte i stereo, och stereons markplan
     finns inte i färgdetektorn. Rutorna byts därför ut, inte fylls med
     platshållare — en tom ruta ser ut som ett fel. */
  show($("kv-surf"), !stereo);
  show($("kv-plane"), stereo);
  show($("kv-sight"), stereo);
  if (stereo) {
    const p = $("cv-plane");
    p.textContent = d.plane_deg == null ? "--" :
      `${d.plane_deg.toFixed(1)}° · ${Math.round(d.plane_h_m * 100)} cm`;
    /* Marklinjens status (V4.5): "att" = attitud-låst (normalt), "ransac" =
       attituden hade för lite mark, föll på RANSAC (gult, en varning),
       "unreliable" = ingen linje alls (rött, tjänsten är blind). "held"/"ok"
       är kvar för bakåtkompatibilitet med gamla loggar. */
    p.style.color = d.ground_status === "unreliable" ? "var(--bad)" :
                    (d.ground_status === "ransac" || d.ground_status === "held")
                    ? "var(--warn)" : "";
    $("cv-sight").textContent = d.roi_range_m ? d.roi_range_m.toFixed(1) + " m" : "--";
    $("cv-diag").textContent = stale ? "--" :
      [d.recording ? `⏺ SPELAR IN "${d.recording}"` : null,
       /* Skymd mark: bara när regeln lyser — ett omatchat nära hinder fångat
          på att marken bakom det saknas. */
       (d.occlusion_frac || 0) > 0.35 ? `⚠ SKYMD MARK ${Math.round(d.occlusion_frac * 100)} %` : null,
       `fart ${d.speed_ms != null ? d.speed_ms.toFixed(2) : "--"} m/s`,
       `nick ${d.pitch_deg != null ? (d.pitch_deg > 0 ? "+" : "") + d.pitch_deg.toFixed(1) : "--"}°`,
       `giltiga ${Math.round(d.valid_frac * 100)} %`,
       `närblind ${Math.round(d.near_blind * 100)} %`,
       `förkastade ${d.rejected} px`,
       `dt ${d.dt_ms > 0 ? "+" : ""}${d.dt_ms.toFixed(0)} ms`,
       `gir ${d.yaw_deg_s > 0 ? "+" : ""}${d.yaw_deg_s.toFixed(1)}°/s`,
       `hinder ${d.min_height_mm}–${d.max_height_mm} mm`,
       `sänder ${d.send_hz != null ? d.send_hz.toFixed(1) : "--"} Hz`
      ].filter(Boolean).join(" · ");
  } else {
    $("cv-surf").textContent = d.surfaces;
    $("cv-diag").textContent = d.blind ? "blind: " + d.blind : "";
  }
  if (d.blind && !stale) $("cv-diag").textContent = "BLIND: " + d.blind;

  /* Vyn (video eller hinderbild) styrs av väljaren, inte härifrån. Den här
     funktionen ritar bara radar + siffror. */
  drawOverlay(d, stale || stereo);      // konturen är färgdetektorns, inte stereons
  radar.av = d; radar.avStale = stale;
  drawRadar();
}

/* Lidarn (roverlidar) — eget websocket-meddelande, eget radarlager. Uppdaterar
   bara sitt lager och ritar om; kameran och lidarn kommer i olika takt och ska
   inte behöva vänta på varandra. */
function renderLidar(d) {
  radar.lidar = d;
  radar.lidarStale = d.age > 1.5;       // lidarn skickar ~5 Hz när överlägget är på
  drawRadar();
  updateLidarInfo(d);
}

/* Fusionen (roverfusion) — det som FAKTISKT skickas till ArduPilot. Eget lager
   (röda prickar) ovanpå de andra. */
function renderFusion(d) {
  radar.fusion = d;
  radar.fusionStale = d.age > 1.5;
  drawRadar();
}

/* Lidar-sektionen i Hinder-panelen: motorvarvtal, träffar före/efter filtren,
   lutning och motorstatus. Skilt från radarn så siffrorna kan uppdateras även om
   ritningen inte ändras. */
const fmtDeg = (v) => (v > 0 ? "+" : "") + (v == null ? "--" : v.toFixed(1)) + "°";
function updateLidarInfo(d) {
  const st = $("lidar-state");
  const stale = !d || d.age > 1.5;
  if (stale) {
    st.textContent = "ingen data"; st.className = "pill pill-bad";
    ["lidar-rpm", "lidar-hits", "lidar-filtered", "lidar-att"]
      .forEach((id) => $(id).textContent = "--");
    return;
  }
  if (d.motor_on) { st.textContent = "kör"; st.className = "pill pill-good"; }
  else { st.textContent = "motor av"; st.className = "pill pill-warn"; }
  $("lidar-rpm").textContent = (d.motor_on ? d.rpm : 0) + " rpm";
  $("lidar-hits").textContent = `${d.raw_valid} → ${d.valid}`;
  $("lidar-filtered").textContent =
    `−${d.ground_rejected || 0} mark · −${d.cluster_removed || 0} klunga`;
  $("lidar-att").textContent = d.attitude
    ? `${fmtDeg(d.pitch_deg)} / ${fmtDeg(d.roll_deg)}` : "ingen attityd";
}

function show(el, on) { if (el) el.style.display = on ? "" : "none"; }

/* Stereons bild hämtas som vanlig HTTP i 2 Hz, inte över websocketen: ~25 kB
   per bild hade dränkt telemetrikanalen. Hämtningen startas om först när den
   förra laddat klart — annars köar de på sig när länken är långsam och
   bilderna blir gamla utan att någon märker det.
   (Tillståndet stereoImgTimer/stereoImgWanted deklareras HÖGT UPP, inte här —
   showStereoImage anropas redan vid boot via setViewMode, och en let-deklaration
   nedanför den anroparen hade legat i temporal dead zone och kraschat booten.) */
function showStereoImage(on) {
  const img = $("stereo-img");
  if (!img) return;
  stereoImgWanted = on;
  if (!on) { img.style.display = "none"; return; }
  if (stereoImgTimer !== null) return;
  const tick = () => {
    if (!stereoImgWanted) { stereoImgTimer = null; return; }
    img.src = "/stereo.jpg?t=" + Date.now();
  };
  /* 150 ms mellan hämtningarna, inte 500: tjänsten skriver 5 Hz och utan
     videoström är det här enda bilden. Nästa hämtning startar först när den
     förra laddat KLART, så en långsam länk sänker takten av sig själv i
     stället för att köa gamla bilder. */
  img.onload = () => {
    if (stereoImgWanted) img.style.display = "block";
    stereoImgTimer = setTimeout(tick, 150);
  };
  /* Ingen bild är ett giltigt läge — tjänsten kan köra med --no-live-image.
     Då ska rutan vara svart, inte visa webbläsarens trasiga-bild-ikon. */
  img.onerror = () => { img.style.display = "none"; stereoImgTimer = setTimeout(tick, 1000); };
  stereoImgTimer = setTimeout(tick, 0);
}

function drawOverlay(d, stale) {
  const c = $("cv-overlay"), v = $("video");
  const w = c.clientWidth, h = c.clientHeight;
  if (!w || !h) return;
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  const g = c.getContext("2d");
  g.clearRect(0, 0, w, h);
  if (stale || !d.contour || !d.contour.length) return;

  const r = videoRect(v, c);
  const n = d.contour.length;
  const px = (i) => r.x + (i + 0.5) / n * r.w;
  const py = (yn) => r.y + yn * r.h;

  /* Fri yta = allt under hinderkonturen. Skuggas svagt grönt så att man ser
     vad rovern anser körbart, inte bara var den tycker att hindren står. */
  g.beginPath();
  g.moveTo(r.x, r.y + r.h);
  let open = false;
  for (let i = 0; i < n; i++) {
    const yn = d.contour[i];
    const y = yn < 0 ? r.y + r.h : py(yn);
    if (!open) { g.lineTo(px(i), y); open = true; } else g.lineTo(px(i), y);
  }
  g.lineTo(r.x + r.w, r.y + r.h);
  g.closePath();
  g.fillStyle = "rgba(46,204,113,0.16)";
  g.fill();

  /* Hinderlinjen. Röd innanför avoidance-marginalen, gul utanför — samma
     färgspråk som i analysbilderna. */
  const marginY = d.ref && d.ref["0.8"] != null ? py(d.ref["0.8"]) : null;
  g.lineWidth = 3;
  for (let i = 0; i < n; i++) {
    const yn = d.contour[i];
    if (yn < 0) continue;
    const y = py(yn);
    g.strokeStyle = (marginY !== null && y > marginY) ? "#e74c3c" : "#f1c40f";
    g.beginPath();
    g.moveTo(px(i) - r.w / n / 2, y);
    g.lineTo(px(i) + r.w / n / 2, y);
    g.stroke();
  }

  /* Referenslinjer för avstånd — utan dem går bilden inte att läsa metriskt. */
  g.setLineDash([6, 6]); g.lineWidth = 1; g.font = "11px system-ui";
  for (const [dist, yn] of Object.entries(d.ref || {})) {
    const y = py(yn);
    g.strokeStyle = dist === "0.8" ? "rgba(231,76,60,.9)" : "rgba(255,255,255,.35)";
    g.beginPath(); g.moveTo(r.x, y); g.lineTo(r.x + r.w, y); g.stroke();
    g.fillStyle = g.strokeStyle;
    g.fillText(dist === "0.8" ? "0,8 m (marginal)" : dist + " m", r.x + 6, y - 3);
  }
  g.setLineDash([]);
}

/* HELRUND 360°-radar med två lager i SAMMA diagram: kameran (~90° framåt, gröna
   prickar) och den roterande lidarn (360° runt om, blå prickar). Rovern sitter i
   MITTEN med nosen uppåt. Avstånds-FÄRGERNA är medvetet borttagna — färg betyder
   nu KÄLLA (grön=kamera, blå=lidar), inte närhet; avståndet läses ur ringarna. */
const CAM_COLOR = "#2ecc71";     // grön = kamerasystemet (stereo/färg)
const LIDAR_COLOR = "#37b6ff";   // blå = 360°-lidarn
const FUSION_COLOR = "#ff5252";  // röd = det som FAKTISKT skickas till ArduPilot

function drawRadar() {
  const c = $("cv-radar"), g = c.getContext("2d");
  const W = c.width, H = c.height;
  g.clearRect(0, 0, W, H);

  const av = radar.av, lid = radar.lidar, fus = radar.fusion;
  const camOK = av && !radar.avStale && av.sectors;
  const lidOK = lid && !radar.lidarStale && lid.dist_cm;
  const fusOK = fus && !radar.fusionStale && fus.sectors;

  /* Allt skalas mot k (canvasbredd/300) så text och prickar följer med när
     panelen ändrar storlek. Rovern i mitten, cirkeln runt om. */
  const k = W / 300;
  const cx = W / 2, cy = H / 2, rpx = Math.min(W, H) / 2 - 16 * k;
  /* Yttre ringen rymmer alla lagers räckvidd så inget klipps bort. */
  const RMAX = Math.max((av && av.max_range_m) || 0,
                        (lid && lid.max_range_m) || 0,
                        (fus && fus.max_cm ? fus.max_cm / 100 : 0), 4);
  const toXY = (deg, m) => {
    const a = deg * Math.PI / 180, rr = Math.min(m, RMAX) / RMAX * rpx;
    return [cx + rr * Math.sin(a), cy - rr * Math.cos(a)];
  };

  // Avståndsringar (hela cirklar) + siffror rakt uppåt.
  g.strokeStyle = "rgba(255,255,255,.14)"; g.fillStyle = "rgba(255,255,255,.45)";
  g.font = (10 * k).toFixed(0) + "px system-ui"; g.lineWidth = Math.max(1, k);
  for (let m = 1; m <= Math.floor(RMAX); m++) {
    const rr = m / RMAX * rpx;
    g.beginPath(); g.arc(cx, cy, rr, 0, 2 * Math.PI); g.stroke();
    g.fillText(m + " m", cx + 3 * k, cy - rr + 11 * k);
  }
  // Hårkors genom mitten — ger känsla för fram/bak/vänster/höger.
  g.strokeStyle = "rgba(255,255,255,.08)";
  g.beginPath(); g.moveTo(cx, cy - rpx); g.lineTo(cx, cy + rpx);
  g.moveTo(cx - rpx, cy); g.lineTo(cx + rpx, cy); g.stroke();

  // Kamerans synfält (~90° framåt) som en svag grön kil — visar var det gröna
  // lagret kan finnas; utanför den ser bara lidarn.
  const fov = (av && av.fov) || 66;
  const fr = fov * Math.PI / 360;                  // halva synfältet i radianer
  g.fillStyle = "rgba(46,204,113,.06)";
  g.beginPath(); g.moveTo(cx, cy);
  g.arc(cx, cy, rpx, -Math.PI / 2 - fr, -Math.PI / 2 + fr);
  g.closePath(); g.fill();
  g.strokeStyle = "rgba(46,204,113,.22)";
  for (const s of [-fov / 2, fov / 2]) {
    const [x, y] = toXY(s, RMAX);
    g.beginPath(); g.moveTo(cx, cy); g.lineTo(x, y); g.stroke();
  }

  // Kamerans närgräns (stereons blinda innerzon) som svag streckad båge inom
  // synfältet — en ärlig blind fläck, inte fri mark. Bara framåt, där den gäller.
  const nl = av && av.near_limit_m;
  if (nl) {
    const rr = nl / RMAX * rpx;
    g.strokeStyle = "rgba(231,76,60,.35)"; g.setLineDash([3 * k, 3 * k]);
    g.beginPath(); g.arc(cx, cy, rr, -Math.PI / 2 - fr, -Math.PI / 2 + fr); g.stroke();
    g.setLineDash([]);
  }

  // Lidar-lagret (blå): en prick per grad med färskt, giltigt avstånd. 360°.
  // Kryssrutorna under radarn styr vilka lager som ritas.
  if (lidOK && radarShow.lidar) {
    g.fillStyle = LIDAR_COLOR;
    const arr = lid.dist_cm;
    for (let deg = 0; deg < 360; deg++) {
      const cm = arr[deg];
      if (cm == null) continue;
      const [x, y] = toXY(deg, cm / 100);
      g.beginPath(); g.arc(x, y, 1.5 * k, 0, 2 * Math.PI); g.fill();
    }
  }

  // Kamera-lagret (grön): en prick per hinder-sektor i synfältet.
  if (camOK && radarShow.cam) {
    const n = av.sectors.length, step = fov / n;
    g.fillStyle = CAM_COLOR;
    for (let i = 0; i < n; i++) {
      const cm = av.sectors[i];
      if (cm == null) continue;                     // fri sektor
      const a = -fov / 2 + step * (i + 0.5);
      const [x, y] = toXY(a, cm / 100);
      g.beginPath(); g.arc(x, y, 2.6 * k, 0, 2 * Math.PI); g.fill();
    }
  }

  // Fusion-lagret (RÖD): 72 sektorer à 5°, element 0 rakt fram, medurs — exakt
  // det som skickas till ArduPilot. Ritas SIST (ovanpå de andra) och lite MINDRE
  // så de underliggande prickarna ringar runt och man ser båda.
  if (fusOK && radarShow.fusion) {
    g.fillStyle = FUSION_COLOR;
    const secs = fus.sectors, sd = fus.sector_deg || 5;
    for (let si = 0; si < secs.length; si++) {
      const cm = secs[si];
      if (cm == null) continue;
      const [x, y] = toXY(si * sd, cm / 100);
      g.beginPath(); g.arc(x, y, 1.2 * k, 0, 2 * Math.PI); g.fill();
    }
  }

  // Rovern i mitten: liten triangel med nosen uppåt.
  g.fillStyle = "rgba(255,255,255,.85)";
  g.beginPath();
  g.moveTo(cx, cy - 7 * k);
  g.lineTo(cx - 5 * k, cy + 5 * k);
  g.lineTo(cx + 5 * k, cy + 5 * k);
  g.closePath(); g.fill();

  // Förklaringen ligger nu som HTML-kryssrutor under radarn (inte på canvasen).
  if (!camOK && !lidOK && !fusOK) {
    g.fillStyle = "rgba(255,255,255,.5)";
    const txt = $("cv-on").checked ? "ingen data" : "överlägg avstängt";
    g.fillText(txt, cx - g.measureText(txt).width / 2, cy - 20 * k);
  }
}

drawRadar();                 // tom radar direkt, i stället för en blank ruta

/* Rita om överlägget när panelen ändrar storlek, annars skalas det fel tills
   nästa bildruta kommer. */
new ResizeObserver(() => {
  const c = $("cv-overlay");
  if (c.clientWidth) { c.width = c.clientWidth; c.height = c.clientHeight; }
}).observe(document.querySelector(".vid-wrap"));
