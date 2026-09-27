"use strict";

(() => {
  const $ = (id) => document.getElementById(id);
  const finite = (n) => typeof n === "number" && Number.isFinite(n);
  let token = "";
  try { token = sessionStorage.getItem("warm-wheels-token") || ""; } catch (_) { /* Session storage may be disabled. */ }
  const fragmentToken = new URLSearchParams(location.hash.slice(1)).get("token");
  if (fragmentToken) {
    token = fragmentToken;
    try { sessionStorage.setItem("warm-wheels-token", token); } catch (_) {}
    history.replaceState(null, "", location.pathname + location.search);
  }
  const remotePi = location.pathname === "/pi";
  const apiPrefix = remotePi ? "/api/pi/proxy" : "";
  if (remotePi) {
    if ($("scan-view-link")) $("scan-view-link").href = "/scan?source=pi";
    document.title = "W.A.R.M wheels · Raspberry Pi live";
    const banner = document.createElement("div");
    banner.className = "notice";
    banner.textContent = "WIRELESS RASPBERRY PI · Live stream and supervised controls. Metre boundaries are not physically enforced without localization.";
    document.querySelector("main").prepend(banner);
    document.querySelector('label[for="token"]').textContent = "Laptop session token";
  }
  $("token").value = token;
  let state = null, lastSuccess = 0, connected = false, lastMode = "", lastEstop = null;
  let polling = false, loadingFrames = false, frameURLs = {}, tokenVersion = 0;
  let displayedColorFrameId = null;
  let driveTimer = null, drive = null, driveGeneration = 0, driveBusy = false;
  let selectedTarget = null, routeOverride = null, toastTimer = null;
  let detectionSignature = "", driveStatus = "", errorSignature = "";
  const mapCanvas = $("map-canvas"), cloudCanvas = $("cloud-canvas"), missionCanvas = $("mission-canvas");
  const loggedAssessments = new Set();
  let lastMissionPhase = "";
  const normalDriveStatuses = new Set(["", "hold a drive control to move", "disarmed", "operator hold-to-drive", "stopped"]);
  const orbit = { yaw: 0.2, pitch: 0.28, zoom: 1, drag: null };
  const assessmentLabels = {
    unassessed: "Unassessed",
    needs_assistance: "Needs assistance — operator report",
    can_self_evacuate: "Can self-evacuate — operator report",
    child_reported: "Child — operator report",
  };

  function log(message) {
    const item = document.createElement("li"), time = document.createElement("time"), content = document.createElement("span");
    time.textContent = new Date().toLocaleTimeString([], { hour12: false });
    content.textContent = message;
    item.append(time, content);
    $("event-log").prepend(item);
    while ($("event-log").children.length > 30) $("event-log").lastElementChild.remove();
  }

  function toast(message) {
    $("toast").textContent = message; $("toast").hidden = false;
    clearTimeout(toastTimer); toastTimer = setTimeout(() => { $("toast").hidden = true; }, 5000);
  }

  async function api(path, body, options = {}) {
    const headers = token ? { Authorization: `Bearer ${token}` } : {};
    if (body !== undefined) headers["Content-Type"] = "application/json";
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), options.timeout || 1800);
    try {
      const response = await fetch(apiPrefix + path, {
        method: body === undefined ? "GET" : "POST", headers,
        body: body === undefined ? undefined : JSON.stringify(body),
        cache: "no-store", signal: controller.signal,
      });
      if (!response.ok) {
        let detail = "";
        try { const data = await response.json(); detail = typeof data.detail === "string" ? data.detail : data.error || ""; } catch (_) {}
        const message = response.status === 401 || response.status === 403
          ? "Access denied. Enter the rover access token and reconnect."
          : detail || `Rover request failed (${response.status}).`;
        const error = new Error(message); error.status = response.status; throw error;
      }
      if (options.blob) return { blob: await response.blob(), frameId: response.headers.get("X-Frame-Id") };
      return response.status === 204 ? {} : response.json();
    } finally { clearTimeout(timeout); }
  }

  function connectionLost(error) {
    const wasConnected = connected;
    connected = false;
    stopDrive(true);
    $("connection").className = "connection offline";
    $("connection").lastChild.textContent = " Connection lost";
    const message = error?.name === "AbortError" ? "Rover connection timed out. Motion controls are locked." : error?.message || "Waiting for a fresh rover connection.";
    $("notice").className = "notice error"; $("notice").lastElementChild.textContent = message;
    $("feed-label").textContent = "STALE / OFFLINE";
    if (message !== errorSignature) { log(message); errorSignature = message; }
    if (wasConnected) toast("Connection lost. Drive commands stopped; rover watchdog must stop the motors.");
    updateControls();
  }

  async function poll() {
    if (polling || document.hidden) return;
    polling = true;
    const version = tokenVersion;
    try {
      const next = await api("/api/state");
      if (version !== tokenVersion) return;
      if (!next || !["demo", "hardware"].includes(next.mode)) throw new Error("Unexpected rover status response.");
      if (!connected) log("Rover status connected.");
      connected = true; lastSuccess = performance.now(); errorSignature = ""; state = next;
      renderState();
      if (!loadingFrames) loadFrames(version);
    } catch (error) { if (version === tokenVersion) connectionLost(error); }
    finally { polling = false; }
  }

  async function loadFrames(version) {
    loadingFrames = true;
    try {
      await Promise.all(["color", "depth"].map(async (view) => {
        try {
          const { blob, frameId } = await api(`/api/frame.jpg?view=${view}`, undefined, { blob: true, timeout: 2200 });
          if (version !== tokenVersion || !connected) return;
          const url = URL.createObjectURL(blob), old = frameURLs[view];
          frameURLs[view] = url;
          const img = $(`${view}-image`);
          img.onload = () => { if (view === "color") { displayedColorFrameId = frameId; drawDetections(); } };
          img.src = url; img.hidden = false; $(`${view}-placeholder`).hidden = true;
          if (old) URL.revokeObjectURL(old);
        } catch (error) {
          if (version !== tokenVersion) return;
          if (error.status === 401 || error.status === 403) connectionLost(error);
          else if (!state?.camera_ok) {
            $(`${view}-image`).hidden = true;
            $(`${view}-placeholder`).hidden = false;
          }
        }
      }));
    } finally { loadingFrames = false; }
  }

  function renderState() {
    const demo = state.mode === "demo";
    $("mode-badge").className = `badge ${demo ? "demo" : "hardware"}`;
    $("mode-badge").textContent = demo ? "SYNTHETIC DEMO" : "HARDWARE LIVE";
    $("connection").className = "connection online";
    $("connection").lastChild.textContent = " Rover connected";
    $("notice").className = `notice${demo ? " demo" : state.error ? " error" : ""}`;
    $("notice").lastElementChild.textContent = state.error ? String(state.error) : demo
      ? "Synthetic demo: camera imagery, scene observations, and geometry are simulated. No physical rover is moving."
      : "Hardware prototype: supervised, controlled tests only. Local depth is not a complete house scan; this kit is not suitable for an active fire.";
    $("metric-camera").textContent = state.camera_ok ? demo ? "Synthetic feed" : "Camera online" : "Camera unavailable";
    $("metric-frame").textContent = finite(state.frame_age_s) ? `Frame age ${state.frame_age_s.toFixed(2)} s` : "No current camera frame";
    $("frame-time").textContent = displayedColorFrameId === null ? "FRAME ID UNKNOWN" : `VIEW #${displayedColorFrameId}`;
    $("metric-clearance").innerHTML = finite(state.clearance_m) ? `${state.clearance_m.toFixed(2)} <small>m</small>` : "— <small>m</small>";
    $("metric-depth").innerHTML = finite(state.depth_valid_fraction) ? `${Math.round(state.depth_valid_fraction * 100)} <small>%</small>` : "— <small>%</small>";
    $("metric-drive").textContent = state.estop ? "Emergency stop" : state.armed ? "Armed" : "Disarmed";
    $("metric-motor").textContent = state.motor?.error || (state.motor?.simulated ? (demo ? "Simulated motor controller" : "Camera only · Arduino not opened") : state.motor?.connected ? "Arduino connected" : "Arduino disconnected");
    $("feed-label").textContent = demo ? "SIMULATED" : state.camera_ok ? "LIVE VIEW" : "NO CAMERA";
    $("inference-label").textContent = typeof state.inference_status === "string" ? state.inference_status : "Perception status unavailable";
    $("estop").classList.toggle("latched", Boolean(state.estop));
    $("estop").textContent = state.estop ? "■ Emergency stop latched" : "■ Emergency stop";
    if (lastMode !== state.mode) { log(demo ? "Synthetic demonstration mode. Observations are simulated." : "Hardware mode. Human supervision required."); lastMode = state.mode; }
    if (lastEstop !== null && lastEstop !== state.estop) log(state.estop ? "Emergency stop latched." : "Emergency latch cleared. Rover remains disarmed.");
    lastEstop = state.estop;
    for (const report of state.events || []) {
      if (!report.assessment) continue;
      const key = `${report.id}:${report.time}:${report.assessment}`;
      if (!loggedAssessments.has(key)) {
        loggedAssessments.add(key);
        log(`Saved operator report · ${report.id}: ${assessmentLabels[report.assessment] || report.assessment}.`);
      }
    }
    updateControls(); renderDetections(); renderMission(); drawDetections(); drawMap(); drawCloud();
  }

  function motionReason() {
    if (document.hidden) return "Return to this tab to operate the rover.";
    if (!connected || !state || performance.now() - lastSuccess > 1400) return "Fresh rover status is required. Controls are locked.";
    if (state.estop) return "Emergency stop is latched. Clear the scene, then reset the latch.";
    if (!state.armed) return "Rover is disarmed. Check the path and arm to enable supervised drive.";
    // Gate permission is independent of the human-readable status label.
    // A normal label such as "Stopped" does not disable a permitted drive.
    if (typeof state.can_drive === "boolean") return state.can_drive ? "" : state.block_reason || "Rover motion safety check is blocking movement.";
    return legacyEnvironmentReason();
  }

  function legacyEnvironmentReason() {
    if (!state.camera_ok || (finite(state.frame_age_s) && state.frame_age_s > 1)) return "Camera frame is unavailable or stale. Motion is locked.";
    if (state.mode !== "demo" && !state.motor?.connected) return "Arduino connection is unavailable. Motion is locked.";
    if (state.mode !== "demo" && !state.motor?.motors_enabled) return "Physical motor output is disabled in rover configuration.";
    if (state.motor?.error) return state.motor.error;
    if (state.motion_enabled === false) return state.block_reason || "Rover motion safety check is blocking movement.";
    if (state.block_reason && !normalDriveStatuses.has(String(state.block_reason).trim().toLowerCase())) return state.block_reason;
    return "";
  }

  function updateControls() {
    const fresh = connected && state && performance.now() - lastSuccess < 1400;
    const reason = motionReason();
    $("drive-reason").textContent = reason || state?.block_reason || (state?.mode === "demo" ? "Simulated drive ready. Hold a direction to preview motor commands." : "Ready for supervised drive. Hold to move, release to stop.");
    $("arm-status").textContent = state?.estop ? "STOP LATCHED" : state?.armed ? "ARMED" : "DISARMED";
    const canArm = state && (typeof state.can_arm === "boolean" ? state.can_arm : !legacyEnvironmentReason());
    $("arm").disabled = !fresh || document.hidden || !canArm || Boolean(state?.armed) || Boolean(state?.estop);
    $("disarm").disabled = !fresh || !state?.armed;
    $("reset").disabled = !fresh || !state?.estop;
    const phase = state?.mission?.phase;
    $("mission-start").disabled = !fresh || state?.mode !== "demo" || !state?.mission || Boolean(state?.estop) || ["exploring", "returning"].includes(phase);
    $("mission-return").disabled = !fresh || state?.mode !== "demo" || !state?.mission || Boolean(state?.estop) || !["exploring", "blocked"].includes(phase);
    for (const name of ["left", "forward", "right"]) $(`drive-${name}`).disabled = Boolean(reason);
    if (reason && drive) stopDrive(true);
    const status = state?.estop ? "estop" : state?.armed ? "armed" : "disarmed";
    if (state && status !== driveStatus) { log(`Drive status: ${status}.`); driveStatus = status; }
  }

  async function action(path, body = {}, success = "") {
    try {
      const result = await api(path, body);
      if (success) { log(success); }
      await poll();
      return result;
    } catch (error) {
      if (error.status === 401 || error.status === 403 || error.name === "AbortError" || error instanceof TypeError) connectionLost(error);
      toast(error.message || "Rover request failed."); log(error.message || "Rover request failed.");
      return null;
    }
  }

  function stopDrive(send = false) {
    const hadDrive = Boolean(drive);
    driveGeneration++;
    clearInterval(driveTimer); driveTimer = null; drive = null;
    document.querySelectorAll(".drive-button").forEach((button) => button.classList.remove("pressed"));
    // Zero is issued immediately, including while an earlier drive request is pending.
    // The server and Arduino independently expire every nonzero drive command.
    if (send && hadDrive) api("/api/drive", { left: 0, right: 0 }, { timeout: 800 }).catch(() => {});
  }

  async function driveTick() {
    if (!drive || driveBusy) return;
    if (motionReason()) { stopDrive(true); return; }
    driveBusy = true;
    const generation = driveGeneration;
    try {
      await api("/api/drive", { left: drive.left, right: drive.right }, { timeout: 700 });
      if (generation !== driveGeneration) await api("/api/drive", { left: 0, right: 0 }, { timeout: 800 }).catch(() => {});
    } catch (error) {
      stopDrive(true);
      if (error.status === 401 || error.status === 403 || error.name === "AbortError" || error instanceof TypeError) connectionLost(error);
      else { toast(error.message); log(error.message); }
    } finally { driveBusy = false; }
  }

  for (const [name, left, right] of [["left", 12, 25], ["forward", 25, 25], ["right", 25, 12]]) {
    const button = $(`drive-${name}`);
    button.addEventListener("pointerdown", (event) => {
      if (event.button !== 0 || motionReason() || drive) return;
      event.preventDefault(); button.setPointerCapture(event.pointerId);
      drive = { left, right, pointerId: event.pointerId }; driveGeneration++;
      button.classList.add("pressed"); driveTick(); driveTimer = setInterval(driveTick, 120);
    });
    for (const event of ["pointerup", "pointercancel", "lostpointercapture"]) button.addEventListener(event, () => stopDrive(true));
    button.addEventListener("contextmenu", (event) => event.preventDefault());
  }
  window.addEventListener("pointerup", () => stopDrive(true));
  window.addEventListener("blur", () => stopDrive(true));
  window.addEventListener("pagehide", () => stopDrive(true));
  document.addEventListener("visibilitychange", () => { if (document.hidden) stopDrive(true); else poll(); });
  document.addEventListener("keydown", (event) => { if (event.key === "Escape") { stopDrive(true); action("/api/drive", { left: 0, right: 0 }); } });
  $("drive-stop").addEventListener("click", () => { stopDrive(false); action("/api/drive", { left: 0, right: 0 }, "Stop command sent."); });
  $("estop").addEventListener("click", () => { stopDrive(false); if (state) state.estop = true; updateControls(); action("/api/estop", {}, "Emergency stop command sent."); });
  $("arm").addEventListener("click", () => action("/api/arm", {}, "Arm request sent."));
  $("disarm").addEventListener("click", () => { stopDrive(false); if (state) state.armed = false; updateControls(); action("/api/disarm", {}, "Disarm command sent."); });
  $("reset").addEventListener("click", () => { stopDrive(false); action("/api/reset", {}, "Emergency reset requested. Re-arm separately when ready."); });
  $("mission-start").addEventListener("click", () => action("/api/mission/start", {}, "Started simulated exploration and return. No physical motor commands."));
  $("mission-return").addEventListener("click", () => action("/api/mission/return", {}, "Simulated return to home requested."));
  $("auth-form").addEventListener("submit", (event) => {
    event.preventDefault(); stopDrive(true);
    token = $("token").value.trim(); tokenVersion++;
    try { if (token) sessionStorage.setItem("warm-wheels-token", token); else sessionStorage.removeItem("warm-wheels-token"); } catch (_) {}
    connected = false; lastSuccess = 0; updateControls();
    $("auth-note").textContent = "Connecting. Token stays in this browser session.";
    poll();
  });

  function renderDetections() {
    const detections = Array.isArray(state.detections) ? state.detections : [];
    $("detection-count").textContent = detections.length;
    const signature = JSON.stringify(detections.map((d) => [d.id, d.kind, d.assessment, Math.round((d.confidence || 0) * 100), finite(d.distance_m) ? d.distance_m.toFixed(1) : null, d.source, d.posture_hint, d.source_frame_id, finite(d.age_s) ? d.age_s.toFixed(1) : null, finite(state.detection_age_s) ? state.detection_age_s.toFixed(1) : null]));
    // Preserve an open native select while its operator is choosing an assessment.
    if (signature === detectionSignature || $("detections").contains(document.activeElement)) return;
    detectionSignature = signature;
    $("detections").replaceChildren();
    if (!detections.length) {
      const empty = document.createElement("div"); empty.className = "empty-state";
      empty.innerHTML = '<span aria-hidden="true">◎</span><p>No current observations</p><small>Absence of detections does not confirm an empty or safe room.</small>';
      $("detections").append(empty); return;
    }
    for (const detection of detections) {
      const person = ["person", "human"].includes(detection.kind);
      const card = document.createElement("article"); card.className = `detection-card ${person ? "person" : "hazard"}`;
      const top = document.createElement("div"); top.className = "detection-top";
      const title = document.createElement("strong"); title.textContent = person ? `Person · ${detection.id}` : "Potential flame / visual cue";
      const confidence = document.createElement("span"); confidence.textContent = finite(detection.confidence) ? `${Math.round(detection.confidence * 100)}%` : "—";
      top.append(title, confidence);
      const details = document.createElement("div"); details.className = "detection-details";
      const distance = document.createElement("span"); distance.textContent = finite(detection.distance_m) ? `${detection.distance_m.toFixed(2)} m from camera` : "Depth unavailable";
      const label = document.createElement("span"); label.textContent = state.mode === "demo" ? "SIMULATED" : "UNCONFIRMED";
      details.append(distance, label); card.append(top, details);
      if (person) {
        const selectLabel = document.createElement("label"); selectLabel.className = "assessment-label"; selectLabel.textContent = "Operator assessment";
        const select = document.createElement("select"); select.setAttribute("aria-label", `Assessment for person ${detection.id}`);
        for (const [value, text] of Object.entries(assessmentLabels)) { const option = document.createElement("option"); option.value = value; option.textContent = text; select.append(option); }
        select.value = detection.assessment || "unassessed";
        select.addEventListener("change", async () => {
          const assessment = select.value; select.disabled = true;
          const result = await action("/api/assessment", { id: detection.id, assessment }, `Operator report for ${detection.id}: ${assessmentLabels[assessment]}.`);
          if (result === null) select.value = detection.assessment || "unassessed";
          select.disabled = false; select.blur(); detectionSignature = "";
        });
        selectLabel.append(select); card.append(selectLabel);
      } else {
        const note = document.createElement("p"); note.className = "hazard-label"; note.textContent = "Requires visual confirmation. Color alone cannot establish fire."; card.append(note);
      }
      const source = document.createElement("span"); source.className = "source-tag";
      source.textContent = `Source: ${detection.source || "not reported"}${detection.posture_hint ? ` · Posture cue: ${detection.posture_hint} (unconfirmed)` : ""}`;
      const age = [detection.age_s, detection.detection_age_s, state.detection_age_s].find(finite);
      const timing = document.createElement("span"); timing.className = "source-tag";
      timing.textContent = `Observation ${finite(age) ? `${age.toFixed(2)} s old` : "age unavailable"} · source frame ${detection.source_frame_id ?? "unknown"}`;
      card.append(source, timing); $("detections").append(card);
    }
  }

  function setupCanvas(canvas) {
    const box = canvas.getBoundingClientRect(), ratio = Math.min(devicePixelRatio || 1, 2);
    if (!box.width || !box.height) return null;
    const width = Math.round(box.width * ratio), height = Math.round(box.height * ratio);
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
    const ctx = canvas.getContext("2d"); ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    return { ctx, w: box.width, h: box.height };
  }

  function drawDetections() {
    const surface = setupCanvas($("detection-overlay")); if (!surface) return;
    const { ctx, w, h } = surface; ctx.clearRect(0, 0, w, h);
    const image = $("color-image"); if (image.hidden || !state?.camera_ok) return;
    const iw = image.naturalWidth || 640, ih = image.naturalHeight || 480;
    const scale = Math.min(w / iw, h / ih), ox = (w - iw * scale) / 2, oy = (h - ih * scale) / 2;
    for (const d of state.detections || []) {
      // Remote inference can be older than the displayed JPEG. Never place a
      // detection onto a different camera frame even when the box seems close.
      if (displayedColorFrameId === null || d.source_frame_id === undefined || d.source_frame_id === null || String(d.source_frame_id) !== String(displayedColorFrameId)) continue;
      if (!Array.isArray(d.box) || d.box.length !== 4 || !d.box.every(finite)) continue;
      const [x1, y1, x2, y2] = d.box;
      const x = x1 * scale + ox, y = y1 * scale + oy, bw = (x2 - x1) * scale, bh = (y2 - y1) * scale;
      if (bw <= 0 || bh <= 0) continue;
      const person = ["person", "human"].includes(d.kind), color = person ? "#a0e5cf" : "#ffad6e";
      ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.strokeRect(x, y, bw, bh);
      const label = `${person ? "PERSON" : "VISUAL FIRE CUE"}${finite(d.distance_m) ? ` · ${d.distance_m.toFixed(1)} m` : ""}`;
      ctx.font = "600 9px Segoe UI, sans-serif";
      const tw = ctx.measureText(label).width + 12, ty = Math.max(0, y - 19);
      ctx.fillStyle = color; ctx.fillRect(Math.max(0, x), ty, tw, 18);
      ctx.fillStyle = "#101b20"; ctx.fillText(label, Math.max(0, x) + 6, ty + 12);
    }
  }

  function gridLayout(w, h, grid = state?.grid) {
    if (!grid || !grid.width || !grid.height) return null;
    const cell = Math.min((w - 36) / grid.width, (h - 72) / grid.height);
    return { grid, cell, x: (w - grid.width * cell) / 2, y: 38 + (h - 72 - grid.height * cell) / 2 };
  }

  function gridCell(grid, col, row) {
    return Array.isArray(grid.cells?.[0]) ? grid.cells[row]?.[col] : grid.cells?.[row * grid.width + col];
  }

  function drawMap() {
    const surface = setupCanvas(mapCanvas); if (!surface) return;
    const { ctx, w, h } = surface;
    ctx.fillStyle = "#0c151b"; ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = "#19272f"; ctx.lineWidth = .5;
    for (let x = 0; x < w; x += 20) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke(); }
    for (let y = 0; y < h; y += 20) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
    const layout = gridLayout(w, h);
    if (!layout) { ctx.fillStyle = "#7f98a6"; ctx.font = "12px Segoe UI"; ctx.textAlign = "center"; ctx.fillText("Waiting for local depth geometry", w / 2, h / 2); return; }
    const { grid, cell, x, y } = layout;
    for (let row = 0; row < grid.height; row++) for (let col = 0; col < grid.width; col++) {
      const value = gridCell(grid, col, row);
      ctx.fillStyle = value === 0 ? "#264849" : value === 1 ? "#bd7956" : "#132029";
      ctx.fillRect(x + col * cell, y + row * cell, Math.max(.2, cell - .5), Math.max(.2, cell - .5));
    }
    const route = routeOverride || state.route || [];
    if (route.length > 1) {
      ctx.beginPath(); route.forEach(([col, row], i) => { const px = x + (col + .5) * cell, py = y + (row + .5) * cell; i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
      ctx.strokeStyle = "#b6f2d9"; ctx.lineWidth = 2; ctx.setLineDash([4, 4]); ctx.stroke(); ctx.setLineDash([]);
    }
    if (selectedTarget) {
      const px = x + (selectedTarget.col + .5) * cell, py = y + (selectedTarget.row + .5) * cell;
      ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI * 2); ctx.strokeStyle = "#b6f2d9"; ctx.lineWidth = 1.5; ctx.stroke();
    }
    const rx = x + ((grid.origin_col ?? grid.width / 2) + .5) * cell, ry = y + ((grid.origin_row ?? grid.height - 1) + .5) * cell;
    ctx.beginPath(); ctx.arc(rx, ry, 13, 0, Math.PI * 2); ctx.fillStyle = "#b6f2d91b"; ctx.fill();
    ctx.beginPath(); ctx.moveTo(rx, ry - 8); ctx.lineTo(rx + 6, ry + 6); ctx.lineTo(rx, ry + 3); ctx.lineTo(rx - 6, ry + 6); ctx.closePath(); ctx.fillStyle = "#c7f0e2"; ctx.fill();
    let markers = 0;
    if (finite(grid.resolution_m) && grid.resolution_m > 0) {
      for (const detection of state.detections || []) {
        if (state.frame_id === undefined || state.frame_id === null || detection.source_frame_id === undefined || detection.source_frame_id === null || String(detection.source_frame_id) !== String(state.frame_id)) continue;
        const point = detection.position_m;
        if (!Array.isArray(point) || point.length < 3 || !point.slice(0, 3).every(finite)) continue;
        const col = (grid.origin_col ?? grid.width / 2) + point[0] / grid.resolution_m;
        const row = (grid.origin_row ?? grid.height - 1) - point[2] / grid.resolution_m;
        if (col < 0 || row < 0 || col >= grid.width || row >= grid.height) continue;
        const px = x + (col + .5) * cell, py = y + (row + .5) * cell;
        const person = ["person", "human"].includes(detection.kind);
        ctx.fillStyle = person ? "#bcf0dc" : "#ffad6e";
        ctx.strokeStyle = "#0c151b"; ctx.lineWidth = 1.5; ctx.beginPath();
        if (person) ctx.arc(px, py, 5, 0, Math.PI * 2);
        else { ctx.moveTo(px, py - 6); ctx.lineTo(px + 5, py); ctx.lineTo(px, py + 6); ctx.lineTo(px - 5, py); ctx.closePath(); }
        ctx.fill(); ctx.stroke();
        ctx.font = "600 8px Segoe UI"; ctx.textAlign = "left";
        const label = person ? "Person" : "Fire cue", tw = ctx.measureText(label).width;
        const tx = Math.min(w - tw - 5, px + 8), ty = Math.max(33, py - 5);
        ctx.fillStyle = "#0c151be8"; ctx.fillRect(tx - 3, ty - 9, tw + 6, 13);
        ctx.fillStyle = person ? "#bcf0dc" : "#ffad6e"; ctx.fillText(label, tx, ty);
        markers++;
      }
    }
    const age = finite(state.detection_age_s) ? state.detection_age_s : null;
    $("observation-age").textContent = markers
      ? `${markers} approximate depth-position marker${markers === 1 ? "" : "s"} · ${age === null ? "observation age unavailable" : `snapshot age ${age.toFixed(2)} s`}. Teal: person; orange: potential fire cue.`
      : "Markers require a depth position matched to this local scan frame. Older observations stay in the list.";
    ctx.fillStyle = "#8fa7b4"; ctx.font = "8px Segoe UI"; ctx.textAlign = "right";
    if (finite(grid.resolution_m)) ctx.fillText(`${grid.resolution_m.toFixed(2)} m / cell`, w - 13, 23);
  }

  mapCanvas.addEventListener("click", async (event) => {
    if (!connected || !state?.grid) { toast("Connect to the rover before requesting a route preview."); return; }
    const rect = mapCanvas.getBoundingClientRect(), layout = gridLayout(rect.width, rect.height);
    const col = Math.floor((event.clientX - rect.left - layout.x) / layout.cell), row = Math.floor((event.clientY - rect.top - layout.y) / layout.cell);
    if (col < 0 || row < 0 || col >= layout.grid.width || row >= layout.grid.height) return;
    if (gridCell(layout.grid, col, row) !== 0) { toast("Select an observed free cell. Unknown space is not traversable."); return; }
    selectedTarget = { col, row }; routeOverride = []; drawMap();
    $("route-title").textContent = "Calculating a local route preview…";
    const response = await action("/api/route", { col, row });
    if (!response) { $("route-title").textContent = "Route preview unavailable"; return; }
    routeOverride = Array.isArray(response.route) ? response.route : [];
    $("route-title").textContent = routeOverride.length ? `${routeOverride.length} cells · preview only` : "No observed free path to this cell";
    $("route-detail").textContent = response.reason || response.message || (routeOverride.length ? "No motion command was issued. Recheck after the rover moves." : "The rover footprint must fit inside observed free space.");
    log(routeOverride.length ? "Local route preview generated. No autonomous motion." : "Route preview blocked by occupied or unknown cells."); drawMap();
  });

  function renderMission() {
    const mission = state?.mission;
    const phase = mission?.phase || "idle";
    const phases = { idle: "Ready to explore", exploring: "Exploring", returning: "Returning home", complete: "Back at home", blocked: "Path blocked" };
    $("mission-phase").textContent = phases[phase] || phase;
    const coverage = finite(mission?.coverage_percent) ? Math.max(0, Math.min(100, mission.coverage_percent)) : 0;
    $("mission-coverage").textContent = `${coverage.toFixed(1)}%`;
    $("mission-progress").style.width = `${coverage}%`;
    $("mission-detail").textContent = state?.mode !== "demo"
      ? "Hardware search missions are unavailable. Localization and verified obstacle coverage are required before physical autonomous exploration."
      : mission?.reason || (mission ? `${mission.steps || 0} simulated steps · ${mission.unobserved_cells ?? "—"} cells still unobserved. The simulator returns home after reachable frontiers are exhausted.` : "Waiting for the mission simulator.");
    if (mission && phase !== lastMissionPhase) {
      log(`Simulated search mission: ${phases[phase] || phase}.`);
      lastMissionPhase = phase;
    }
    drawMission();
  }

  function drawMission() {
    if ($("mission-view").hidden) return;
    const surface = setupCanvas(missionCanvas); if (!surface) return;
    const { ctx, w, h } = surface;
    ctx.fillStyle = "#0c151b"; ctx.fillRect(0, 0, w, h);
    const mission = state?.mission;
    const layout = gridLayout(w, h, mission?.grid);
    if (!mission?.grid || !layout) {
      ctx.fillStyle = "#8fa7b4"; ctx.font = "11px Segoe UI"; ctx.textAlign = "center";
      ctx.fillText(state?.mode === "hardware" ? "Synthetic mission available in demo mode" : "Waiting for simulated floorplan", w / 2, h / 2); return;
    }
    const { grid, cell, x, y } = layout;
    for (let row = 0; row < grid.height; row++) for (let col = 0; col < grid.width; col++) {
      const value = gridCell(grid, col, row);
      ctx.fillStyle = value === 0 ? "#264849" : value === 1 ? "#bd7956" : "#132029";
      ctx.fillRect(x + col * cell, y + row * cell, Math.max(.2, cell - .5), Math.max(.2, cell - .5));
    }
    const position = (p) => Array.isArray(p) ? [x + (p[0] + .5) * cell, y + (p[1] + .5) * cell] : null;
    if (mission.route?.length > 1) {
      ctx.beginPath(); mission.route.forEach((point, i) => { const p = position(point); if (p) i ? ctx.lineTo(...p) : ctx.moveTo(...p); });
      ctx.strokeStyle = "#b6f2d9"; ctx.lineWidth = 2; ctx.setLineDash([4, 3]); ctx.stroke(); ctx.setLineDash([]);
    }
    const home = position(mission.home);
    if (home) {
      ctx.fillStyle = "#ffd397"; ctx.strokeStyle = "#ffd397"; ctx.lineWidth = 1.5;
      ctx.strokeRect(home[0] - 5, home[1] - 4, 10, 9);
      ctx.beginPath(); ctx.moveTo(home[0] - 7, home[1] - 4); ctx.lineTo(home[0], home[1] - 10); ctx.lineTo(home[0] + 7, home[1] - 4); ctx.stroke();
      ctx.font = "700 8px Segoe UI"; ctx.textAlign = "center"; ctx.fillText("HOME", home[0], home[1] + 17);
    }
    const rover = position(mission.position);
    if (rover) {
      ctx.beginPath(); ctx.arc(...rover, 11, 0, Math.PI * 2); ctx.fillStyle = "#b6f2d928"; ctx.fill();
      ctx.beginPath(); ctx.arc(...rover, 5, 0, Math.PI * 2); ctx.fillStyle = "#c7f0e2"; ctx.fill(); ctx.strokeStyle = "#0c151b"; ctx.lineWidth = 1.5; ctx.stroke();
    }
    ctx.fillStyle = "#f3bd8b"; ctx.font = "700 8px Segoe UI"; ctx.textAlign = "left"; ctx.fillText("SYNTHETIC MISSION / NO PHYSICAL MOVEMENT", 12, 21);
  }

  function drawCloud() {
    if ($("cloud-view").hidden) return;
    const surface = setupCanvas(cloudCanvas); if (!surface) return;
    const { ctx, w, h } = surface;
    ctx.fillStyle = "#0b141a"; ctx.fillRect(0, 0, w, h);
    const points = state?.points || [];
    const sinY = Math.sin(orbit.yaw), cosY = Math.cos(orbit.yaw), sinP = Math.sin(orbit.pitch), cosP = Math.cos(orbit.pitch);
    const project = (px, py, pz) => {
      const z = pz - 2, x1 = cosY * px + sinY * z, z1 = -sinY * px + cosY * z;
      const y1 = cosP * py - sinP * z1, depth = sinP * py + cosP * z1 + 6;
      if (depth <= .15) return null;
      const scale = Math.min(w, h) * .92 * orbit.zoom / depth;
      return { x: w / 2 + x1 * scale, y: h * .5 - y1 * scale, depth, scale };
    };
    ctx.lineWidth = .7; ctx.strokeStyle = "#263b47";
    for (let line = -4; line <= 4; line++) {
      for (const pair of [[[line, -1, 0], [line, -1, 7]], [[-4, -1, line + 3], [4, -1, line + 3]]]) {
        const a = project(...pair[0]), b = project(...pair[1]); if (!a || !b) continue;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      }
    }
    const visible = [];
    const stride = Math.max(1, Math.ceil(points.length / 14000));
    for (let i = 0; i < points.length; i += stride) {
      const p = points[i]; if (!Array.isArray(p) || p.length < 3 || !p.slice(0, 3).every(finite)) continue;
      const q = project(p[0], p[1], p[2]); if (q && q.x >= 0 && q.x < w && q.y >= 0 && q.y < h) visible.push({ ...q, p });
    }
    visible.sort((a, b) => b.depth - a.depth);
    for (const { x, y, p, scale } of visible) {
      const rgb = p.length >= 6 ? p.slice(3, 6).map((c) => Math.max(0, Math.min(255, Math.round(c)))) : [138, 204, 195];
      ctx.fillStyle = `rgb(${rgb.join(",")})`;
      const size = Math.max(1.3, Math.min(3.4, scale * .036)); ctx.fillRect(x, y, size, size);
    }
    ctx.fillStyle = "#8fa7b4"; ctx.font = "9px Segoe UI"; ctx.textAlign = "left"; ctx.fillText(`${points.length.toLocaleString()} points · camera-local`, 12, 23);
    if (!points.length) { ctx.textAlign = "center"; ctx.fillText("Waiting for depth points", w / 2, h / 2); }
  }

  cloudCanvas.addEventListener("pointerdown", (event) => { orbit.drag = { x: event.clientX, y: event.clientY, id: event.pointerId }; cloudCanvas.setPointerCapture(event.pointerId); });
  cloudCanvas.addEventListener("pointermove", (event) => {
    if (!orbit.drag || event.pointerId !== orbit.drag.id) return;
    orbit.yaw += (event.clientX - orbit.drag.x) * .009; orbit.pitch = Math.max(-1.2, Math.min(1.2, orbit.pitch + (event.clientY - orbit.drag.y) * .009));
    orbit.drag.x = event.clientX; orbit.drag.y = event.clientY; drawCloud();
  });
  for (const event of ["pointerup", "pointercancel", "lostpointercapture"]) cloudCanvas.addEventListener(event, () => { orbit.drag = null; });
  cloudCanvas.addEventListener("wheel", (event) => { event.preventDefault(); orbit.zoom = Math.max(.35, Math.min(3.5, orbit.zoom * Math.exp(-event.deltaY * .001))); drawCloud(); }, { passive: false });
  $("reset-cloud").addEventListener("click", () => { orbit.yaw = .2; orbit.pitch = .28; orbit.zoom = 1; drawCloud(); });

  function changeTab(name) {
    for (const tab of ["map", "cloud", "mission"]) {
      $(`tab-${tab}`).classList.toggle("active", name === tab);
      $(`tab-${tab}`).setAttribute("aria-selected", String(name === tab));
      $(`tab-${tab}`).tabIndex = name === tab ? 0 : -1;
      $(`${tab}-view`).hidden = name !== tab;
    }
    $("local-route-summary").hidden = name === "mission";
    $("local-scan-note").hidden = name === "mission";
    if (name === "map") drawMap(); else if (name === "cloud") drawCloud(); else drawMission();
  }
  for (const name of ["map", "cloud", "mission"]) {
    $(`tab-${name}`).addEventListener("click", () => changeTab(name));
    $(`tab-${name}`).addEventListener("keydown", (event) => {
      if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
      event.preventDefault(); const tabs = ["map", "cloud", "mission"];
      const next = event.key === "Home" ? "map" : event.key === "End" ? "mission" : tabs[(tabs.indexOf(name) + (event.key === "ArrowRight" ? 1 : 2)) % 3];
      changeTab(next); $(`tab-${next}`).focus();
    });
  }
  window.addEventListener("resize", () => { drawMap(); drawCloud(); drawMission(); drawDetections(); });
  setInterval(() => { if (connected && performance.now() - lastSuccess > 1400) { connectionLost(new Error("Telemetry is stale. Motion controls are locked.")); } else updateControls(); }, 150);
  setInterval(poll, 500);
  drawMap(); poll();
})();
