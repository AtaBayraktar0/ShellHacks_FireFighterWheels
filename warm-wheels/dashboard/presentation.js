import { MissionViewport } from "/static/presentation-3d.js";

"use strict";

(() => {
  const $ = (id) => document.getElementById(id);
  const finite = (value) => typeof value === "number" && Number.isFinite(value);
  const clamp = (value, low, high) => Math.min(high, Math.max(low, value));
  const reducedMotion = matchMedia("(prefers-reduced-motion: reduce)").matches;
  let token = "";
  try { token = sessionStorage.getItem("warm-wheels-token") || ""; } catch (_) {}
  const hash = new URLSearchParams(location.hash.slice(1));
  if (hash.has("token")) {
    token = hash.get("token") || "";
    try { sessionStorage.setItem("warm-wheels-token", token); } catch (_) {}
    hash.delete("token");
    const remaining = hash.toString();
    history.replaceState(null, "", location.pathname + location.search + (remaining ? `#${remaining}` : ""));
  }
  $("token").value = token;
  let state = null, online = false, lastSuccess = 0, polling = false, controlling = false;
  let authGeneration = 0, toastTimer = null, eventSignature = "", entitySignature = "";
  let boundaryDirty = false, seedDirty = false, lastBoundary = "";
  let latestCallout = "", calloutUntil = 0;
  const missionViewport = new MissionViewport($("mission-map"), $("map-placeholder"));
  const phaseLabels = { idle: "STANDBY", exploring: "EXPLORING", returning: "RETURNING HOME", complete: "BACK AT HOME", blocked: "RETURN BLOCKED", sensor_hold: "SENSOR HOLD", escort_wait: "VERIFYING FOLLOWER", assessing: "CHECKING RESPONSE" };
  const prettyTime = (seconds) => { const value = Math.max(0, Math.floor(Number(seconds) || 0)); return `${String(Math.floor(value / 60)).padStart(2, "0")}:${String(value % 60).padStart(2, "0")}`; };

  function showToast(message) {
    $("toast").textContent = message; $("toast").hidden = false;
    clearTimeout(toastTimer); toastTimer = setTimeout(() => { $("toast").hidden = true; }, 4500);
  }

  async function request(path, body) {
    const controller = new AbortController(), timeout = setTimeout(() => controller.abort(), body === undefined ? 4000 : 15000);
    try {
      const headers = token ? { Authorization: `Bearer ${token}` } : {};
      if (body !== undefined) headers["Content-Type"] = "application/json";
      const response = await fetch(path, { method: body === undefined ? "GET" : "POST", headers,
        body: body === undefined ? undefined : JSON.stringify(body), cache: "no-store", signal: controller.signal });
      if (!response.ok) {
        let detail = "";
        try { const json = await response.json(); detail = typeof json.detail === "string" ? json.detail : json.error || ""; } catch (_) {}
        const error = new Error(response.status === 401 || response.status === 403 ? "Enter the demo session token to connect." : detail || `Demo request failed (${response.status}).`);
        error.status = response.status; throw error;
      }
      return await response.json();
    } finally { clearTimeout(timeout); }
  }

  function setOffline(error) {
    online = false; missionViewport.setOnline(false);
    $("connection").className = "connection offline";
    $("connection").lastChild.textContent = "Disconnected";
    $("connection-notice").hidden = false;
    $("connection-notice").textContent = error?.name === "AbortError" ? "The demo server is taking too long to respond. The displayed mission is no longer live." : error?.message || "Waiting for a live demo connection.";
    $("phase-badge").textContent = state ? "VIEW PAUSED / OFFLINE" : "CONNECT TO BEGIN";
    $("phase-badge").className = "phase-badge alert";
    updateButtons();
  }

  async function poll() {
    if (polling || document.hidden) return;
    polling = true; const generation = authGeneration;
    try {
      const next = await request("/api/presentation");
      if (generation !== authGeneration) return;
      acceptState(next);
    } catch (error) { if (generation === authGeneration) setOffline(error); }
    finally { polling = false; }
  }

  function acceptState(next) {
    if (!next || next.simulation_only !== true || !next.grid) throw new Error("This page requires the synthetic presentation server. No physical motor controls are available here.");
    const now = performance.now();
    const restarted = state?.scenario?.id !== next.scenario?.id || state?.seed !== next.seed || JSON.stringify(state?.boundary) !== JSON.stringify(next.boundary) || (next.metrics?.steps || 0) < (state?.metrics?.steps || 0);
    state = next; online = true; lastSuccess = now;
    missionViewport.update(next);
    $("connection").className = "connection online"; $("connection").lastChild.textContent = "Demo connected";
    $("connection-notice").hidden = true; $("map-placeholder").hidden = missionViewport.available;
    if (restarted) { latestCallout = ""; calloutUntil = 0; $("scene-callout").hidden = true; }
    render();
  }

  async function control(body) {
    if (controlling) return;
    controlling = true; updateButtons();
    try {
      const result = await request("/api/presentation/control", body);
      if (result?.simulation_only === true && result.grid) acceptState(result);
      else await poll();
    } catch (error) {
      showToast(error?.name === "AbortError" ? "The command timed out. Check the connection." : error.message);
      if (error.status === 401 || error.status === 403 || error instanceof TypeError || error.name === "AbortError") setOffline(error);
    } finally { controlling = false; updateButtons(); }
  }

  function updateButtons() {
    const ready = online && !controlling;
    $("play").disabled = !ready;
    $("restart").disabled = !ready;
    $("download").disabled = !ready || !state;
    $("speed").disabled = !ready;
    $("boundary-apply").disabled = !ready;
    $("randomize").disabled = !ready; $("load-seed").disabled = !ready;
    const playing = state?.playing === true;
    $("play-icon").textContent = playing ? "Ⅱ" : "▶";
    $("play-label").textContent = playing ? "Pause mission" : ["complete", "blocked"].includes(state?.phase) ? "Replay mission" : state?.phase === "idle" ? "Play mission" : "Resume mission";
  }

  function render() {
    const scenario = state.scenario || {};
    const copy = {
      outcome: "Discover. Assess. Adapt.",
      summary: "A generated mission can end with a completed search, an assistance report, an escorted actor, or a blocked return. Review the evidence in the timeline.",
    };
    $("scenario-title").textContent = "Mission " + String(state.seed ?? 0);
    $("brief-title").textContent = state.phase === "idle" ? "An unknown interior." : "Decisions from observations.";
    $("brief-copy").textContent = "Explore the space. Observe changing hazards. Check each simulated person's response before deciding what to do next.";
    $("scenario-index").textContent = "PROCEDURAL";
    $("phase-badge").textContent = state.playing ? phaseLabels[state.phase] || "MISSION RUNNING" : state.phase === "idle" ? "READY TO PLAY" : ["complete", "blocked"].includes(state.phase) ? phaseLabels[state.phase] : `PAUSED / ${phaseLabels[state.phase] || "MISSION"}`;
    $("phase-badge").className = `phase-badge${["blocked", "sensor_hold", "escort_wait"].includes(state.phase) ? " alert" : state.phase === "complete" ? " done" : ""}`;
    const metrics = state.metrics || {}, coverage = finite(metrics.coverage_percent) ? clamp(metrics.coverage_percent, 0, 100) : 0;
    $("coverage").replaceChildren(document.createTextNode(coverage.toFixed(coverage === 100 || coverage === 0 ? 0 : 1)), Object.assign(document.createElement("span"), { textContent: "%" }));
    $("coverage-fill").style.width = `${coverage}%`;
    $("steps").textContent = metrics.steps ?? 0;
    $("observation-count").textContent = metrics.detections ?? state.entities?.length ?? 0;
    $("frontier-count").textContent = state.frontiers?.length ?? "—";
    $("elapsed").textContent = prettyTime(metrics.elapsed_sim_s);
    if (document.activeElement !== $("speed")) $("speed").value = String(state.speed || 1);
    const status = {
      idle: ["⌖", "Ready when you are", "Start the scan. People, hazards, and their responses are discovered during the mission."],
      exploring: ["↗", "Looking beyond the known", "The planner targets frontiers bordering observed free space."],
      returning: ["⌂", "Finding the way home", "Following a route through observed free space to the launch point."],
      complete: ["✓", "Mission returned home", "The rover reached its launch point. Unknown real-world areas remain outside this claim."],
      blocked: ["⊘", "Return path blocked", "Movement has stopped. The mission cannot verify a route home."],
      assessing: ["◎", "Checking the person’s response", "Holding position while the simulation supplies repeated observations."],
      sensor_hold: ["◌", "Sensor interruption", "A simulated sensor fault has paused movement in this simulation."],
      escort_wait: ["↝", "Verifying the follower", "The rover holds while the simulated follower is out of view or too far away."],
    }[state.phase] || ["⌖", "Mission in progress", "Waiting for the next planning update."];
    $("status-icon").textContent = status[0]; $("status-title").textContent = status[1];
    $("status-detail").textContent = state.reason || status[2];
    $("status-card").className = `status-card${["blocked", "sensor_hold", "escort_wait", "assessing"].includes(state.phase) ? " alert" : ""}`;
    const outcome = state.outcome || {};
    $("outcome-title").textContent = outcome.status !== "pending" && outcome.title ? outcome.title : copy.outcome;
    $("outcome-copy").textContent = outcome.status !== "pending" && outcome.summary ? outcome.summary : copy.summary;
    const scan = state.scan_mesh || {};
    const vertices = scan.vertices?.length || 0, triangles = scan.triangles?.length || 0;
    $("map-scale").textContent = triangles ? `${triangles.toLocaleString()} triangles` : "Awaiting depth returns";
    $("mesh-vertices").textContent = vertices.toLocaleString();
    $("mesh-triangles").textContent = triangles.toLocaleString();
    $("mesh-source").textContent = scan.horizontal_fov_deg ? `Forward ${scan.horizontal_fov_deg}° depth · retained surfaces` : "Forward synthetic depth · retained surfaces";
    const boundary = state.boundary || { width_m: state.grid.width * state.grid.resolution_m, height_m: state.grid.height * state.grid.resolution_m };
    const boundaryKey = `${boundary.width_m}:${boundary.height_m}`;
    $("boundary-note").textContent = `${boundary.width_m} × ${boundary.height_m} m · Simulation only`;
    if (!boundaryDirty || lastBoundary !== boundaryKey) {
      $("boundary-width").value = boundary.width_m; $("boundary-height").value = boundary.height_m;
      $("width-value").textContent = `${boundary.width_m} m`; $("height-value").textContent = `${boundary.height_m} m`; boundaryDirty = false;
    }
    lastBoundary = boundaryKey;
    if (!seedDirty && document.activeElement !== $("layout-seed")) $("layout-seed").value = state.seed ?? 0;
    $("layout-hash").textContent = `Seed ${state.seed ?? "—"} · ${(state.layout_hash || "pending").slice(0, 8)} · Replay preserves the mission`;
    $("follower-status").hidden = !state.follower;
    if (state.follower) {
      $("follower-title").textContent = state.follower.at_launch_zone ? "Follower reached the launch zone" : state.follower.following_verified ? "Following verified in simulation" : "Following not verified · Hold";
      $("follower-detail").textContent = `${state.follower.visible ? "Visible" : "Not visible"} · ${finite(state.follower.distance_m) ? state.follower.distance_m.toFixed(2) + " m separation" : "Distance unavailable"} · Simulated actor`;
    }
    renderEntities(); renderEvents(); updateButtons();
  }

  function renderEntities() {
    const entities = Array.isArray(state.entities) ? state.entities : [];
    const signature = JSON.stringify(entities);
    if (signature === entitySignature) return;
    entitySignature = signature; $("observations").replaceChildren();
    if (!entities.length) {
      const empty = document.createElement("p"); empty.className = "empty-observations";
      empty.textContent = "Nothing discovered yet. The search keeps going."; $("observations").append(empty); return;
    }
    for (const entity of entities) {
      const hazard = entity.kind !== "person";
      const card = document.createElement("article"); card.className = `observation${hazard ? " hazard" : ""}`;
      const icon = document.createElement("span"); icon.className = "observation-icon"; icon.setAttribute("aria-hidden", "true"); icon.textContent = hazard ? "◇" : "♙";
      const content = document.createElement("div"), title = document.createElement("strong"), note = document.createElement("small");
      title.textContent = entity.label || (entity.kind === "person" ? "Person observation" : entity.kind === "hazard" ? "Hazard candidate" : "Obstruction");
      const assessment = entity.assessment;
      const situation = typeof assessment === "string" ? assessment : assessment?.status || entity.response || entity.situation || "";
      const evidence = typeof assessment === "object" && assessment?.evidence ? (Array.isArray(assessment.evidence) ? assessment.evidence.join(" · ") : assessment.evidence) : "";
      const strength = finite(entity.intensity) ? Math.round(clamp(entity.intensity, 0, 1) * 100) : null;
      note.textContent = hazard
        ? `${strength === null ? "Observed hazard" : "Intensity " + strength + "%"}${entity.spread_cells?.length ? " · " + entity.spread_cells.length + " affected cells" : ""} · Simulated`
        : `${situation ? String(situation).replaceAll("_", " ") : "Response not yet assessed"}${evidence ? " · " + evidence : ""} · Simulated`;
      if (entity.visible === false) note.textContent += " · Last seen";
      if (hazard && strength !== null) {
        const meter = document.createElement("span"); meter.className = "fire-strength";
        const fill = document.createElement("span"); fill.style.width = strength + "%"; meter.append(fill); content.append(meter);
      }
      content.append(title, note); card.append(icon, content); $("observations").append(card);
    }
  }

  function renderEvents() {
    const events = Array.isArray(state.events) ? state.events : [];
    $("event-count").textContent = `${events.length} EVENT${events.length === 1 ? "" : "S"}`;
    const signature = JSON.stringify(events.map((event) => [event.id, event.title, event.message]));
    if (signature === eventSignature) return;
    eventSignature = signature; const list = $("timeline"); list.replaceChildren();
    if (!events.length) { const empty = document.createElement("li"); empty.className = "timeline-empty"; empty.textContent = "Events appear as the simulated rover explores."; list.append(empty); return; }
    for (const event of events.slice(-24)) {
      const item = document.createElement("li");
      if (/hazard|block|fault|sensor|obstruction/i.test(event.type || "")) item.className = "alert";
      const time = document.createElement("time"), title = document.createElement("strong"), message = document.createElement("p");
      time.textContent = `${prettyTime(event.sim_time_s)} · MOVE ${event.step ?? 0}`;
      title.textContent = event.title || event.type || "Mission update"; message.textContent = event.message || "";
      item.append(time, title, message); list.append(item);
    }
    const last = events[events.length - 1];
    if (String(last.id) !== latestCallout && !/start|ready|reset/i.test(last.type || "")) {
      latestCallout = String(last.id); calloutUntil = performance.now() + 4300;
      $("scene-callout").textContent = last.title || last.message || "Mission update";
      $("scene-callout").className = `scene-callout${/hazard|block|fault|sensor|obstruction/i.test(last.type || "") ? " hazard" : ""}`;
      $("scene-callout").hidden = false;
    }
    list.scrollTo({ left: list.scrollWidth, behavior: reducedMotion ? "instant" : "smooth" });
  }

  $("play").addEventListener("click", async () => {
    if (["complete", "blocked"].includes(state?.phase)) { await control({ action: "restart" }); }
    await control({ action: state?.playing ? "pause" : "play" });
  });
  $("restart").addEventListener("click", async () => { await control({ action: "restart" }); if (online) await control({ action: "play" }); });
  $("speed").addEventListener("change", () => control({ action: "speed", speed: Number($("speed").value) }));
  for (const dimension of ["width", "height"]) $("boundary-" + dimension).addEventListener("input", () => { boundaryDirty = true; $(dimension + "-value").textContent = `${$("boundary-" + dimension).value} m`; });
  $("boundary-form").addEventListener("submit", async (event) => { event.preventDefault(); await control({ action: "boundary", width_m: Number($("boundary-width").value), height_m: Number($("boundary-height").value) }); });
  $("randomize").addEventListener("click", () => { seedDirty = false; control({ action: "randomize" }); });
  $("layout-seed").addEventListener("input", () => { seedDirty = true; });
  $("load-seed").addEventListener("click", () => { const seed = Number($("layout-seed").value); if (!Number.isInteger(seed) || seed < 0 || seed > 2147483647) { showToast("Use an integer seed from 0 to 2147483647."); return; } seedDirty = false; control({ action: "randomize", seed }); });
  $("mission-home-view").addEventListener("click", () => missionViewport.home());
  $("mesh-full-height").addEventListener("click", () => {
    const fullHeight = $("mesh-full-height").getAttribute("aria-pressed") !== "true";
    $("mesh-full-height").setAttribute("aria-pressed", String(fullHeight));
    missionViewport.setCutaway(!fullHeight);
  });
  $("fullscreen").addEventListener("click", async () => {
    try { if (document.fullscreenElement) await document.exitFullscreen(); else await document.documentElement.requestFullscreen(); }
    catch (_) { showToast("Fullscreen is unavailable in this browser view. Use your browser’s fullscreen shortcut."); }
  });
  document.addEventListener("fullscreenchange", () => { $("fullscreen").setAttribute("aria-label", document.fullscreenElement ? "Exit fullscreen" : "Enter fullscreen"); });
  $("token-open").addEventListener("click", () => { $("token-dialog").showModal(); $("token").focus(); });
  $("token-close").addEventListener("click", () => $("token-dialog").close());
  $("token-form").addEventListener("submit", (event) => {
    event.preventDefault(); token = $("token").value.trim(); authGeneration++;
    try { if (token) sessionStorage.setItem("warm-wheels-token", token); else sessionStorage.removeItem("warm-wheels-token"); } catch (_) {}
    $("token-dialog").close(); online = false; updateButtons(); poll();
  });
  $("download").addEventListener("click", async () => {
    try {
      const report = await request("/api/presentation/report");
      const blob = new Blob([JSON.stringify(report, null, 2) + "\n"], { type: "application/json" }), url = URL.createObjectURL(blob);
      const link = document.createElement("a"); link.href = url;
      link.download = `WARM-wheels_${String(state?.scenario?.id || "mission").replace(/[^a-z0-9_-]/gi, "")}_${new Date().toISOString().replace(/[:.]/g, "-")}.json`;
      document.body.append(link); link.click(); link.remove(); setTimeout(() => URL.revokeObjectURL(url), 1000);
      showToast("Mission report downloaded. Synthetic simulation evidence only.");
    } catch (error) { showToast(error.message || "The report could not be downloaded."); }
  });
  document.addEventListener("keydown", (event) => {
    if (event.altKey || event.ctrlKey || event.metaKey || event.repeat || $("token-dialog").open || /INPUT|SELECT|TEXTAREA|BUTTON|A/.test(document.activeElement?.tagName || "") || !online || controlling) return;
    if (event.code === "Space") { event.preventDefault(); $("play").click(); }
    else if (event.key.toLowerCase() === "r") $("restart").click();
    else if (event.key.toLowerCase() === "n") $("randomize").click();
  });
  document.addEventListener("visibilitychange", () => { if (!document.hidden) { poll(); } });
  setInterval(() => { if (online && performance.now() - lastSuccess > 3200) setOffline(new Error("Mission updates are stale. The displayed scene is no longer live.")); poll(); }, 350);
  for (const style of ["surface", "wireframe", "points"]) $("mesh-" + style).addEventListener("click", () => {
    missionViewport.setStyle(style);
    for (const name of ["surface", "wireframe", "points"]) $("mesh-" + name).setAttribute("aria-pressed", String(name === style));
  });
  updateButtons(); poll();
  setInterval(() => { if (calloutUntil && performance.now() > calloutUntil) $("scene-callout").hidden = true; }, 250);
})();
