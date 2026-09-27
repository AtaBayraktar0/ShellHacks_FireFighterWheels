import * as THREE from "/static/vendor/three.module.js";
import { OrbitControls } from "/static/vendor/OrbitControls.js";

const $ = (id) => document.getElementById(id);
const finite = (value) => typeof value === "number" && Number.isFinite(value);
const remote = new URLSearchParams(location.search).get("source") === "pi";
const prefix = remote ? "/api/pi/proxy" : "";
let token = "";
try { token = sessionStorage.getItem("warm-wheels-token") || ""; } catch (_) {}
const fragment = new URLSearchParams(location.hash.slice(1));
if (fragment.has("token")) {
  token = fragment.get("token") || "";
  try { sessionStorage.setItem("warm-wheels-token", token); } catch (_) {}
  fragment.delete("token"); const remaining = fragment.toString();
  history.replaceState(null, "", location.pathname + location.search + (remaining ? `#${remaining}` : ""));
}
$("scan-token").value = token;
if (remote) $("console-link").href = "/pi";
const viewport = $("viewport");
let renderer = null, scene = null, camera = null, controls = null, surface = null, wire = null;
let referenceGrid = null, sourceCamera = null, axes = null;
let lastMesh = null, receivedAt = 0, online = false, paused = false, polling = false, depthBusy = false;
let style = "rgb", fitted = false, meshRevision = null, depthURL = null, tokenVersion = 0;
let toastTimer = null, failedWebGL = false;
let pins = [], pinSignature = "";
let retained = new URLSearchParams(location.search).get("map") === "room";
$("scan-source").value = remote ? "pi" : "local";

function toast(message) {
  $("scan-toast").textContent = message; $("scan-toast").hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => { $("scan-toast").hidden = true; }, 4300);
}

function initScene() {
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: "high-performance" });
    renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
    renderer.setClearColor(0x0d1921, 1); renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.domElement.setAttribute("aria-hidden", "true"); viewport.prepend(renderer.domElement);
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(48, 1, .015, 100);
    camera.position.set(2.3, 1.8, 3.3);
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; controls.dampingFactor = .08;
    controls.minDistance = .15; controls.maxDistance = 30;
    controls.target.set(0, .5, -1.7); controls.update();
    referenceGrid = new THREE.GridHelper(10, 20, 0x486776, 0x253e4b);
    referenceGrid.material.transparent = true; referenceGrid.material.opacity = .55;
    referenceGrid.position.set(0, -.012, -2.5); scene.add(referenceGrid);
    axes = new THREE.AxesHelper(.32); axes.position.set(0, .01, 0); scene.add(axes);
    sourceCamera = new THREE.Group();
    const marker = new THREE.Mesh(new THREE.BoxGeometry(.08, .05, .05), new THREE.MeshBasicMaterial({ color: 0xb7c8a5 }));
    sourceCamera.add(marker);
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(.018, .018, .018, 12), new THREE.MeshBasicMaterial({ color: 0x749baa }));
    lens.rotation.x = Math.PI / 2; lens.position.z = -.033; sourceCamera.add(lens);
    sourceCamera.position.set(0, .22, 0); scene.add(sourceCamera);
    const tick = () => { if (renderer && scene && camera) { resize(); controls.update(); renderer.render(scene, camera); } requestAnimationFrame(tick); };
    requestAnimationFrame(tick);
  } catch (_) {
    failedWebGL = true;
    $("viewport-empty").querySelector("strong").textContent = "3D graphics are unavailable";
    $("viewport-empty").querySelector("p").textContent = "Open this page in a browser with WebGL enabled. The depth preview and source status remain available.";
  }
}

function resize() {
  const width = viewport.clientWidth, height = viewport.clientHeight;
  if (!width || !height || !renderer) return;
  const size = renderer.getSize(new THREE.Vector2());
  if (size.x !== width || size.y !== height) { renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix(); }
}

async function api(path, blob = false, body = null) {
  const controller = new AbortController(), timeout = setTimeout(() => controller.abort(), remote ? 4500 : 2200);
  try {
    const response = await fetch(prefix + path, { method: body ? "POST" : "GET", body: body ? JSON.stringify(body) : undefined, headers: { ...(token ? { Authorization: `Bearer ${token}` } : {}), ...(body ? { "Content-Type": "application/json" } : {}) }, cache: "no-store", signal: controller.signal });
    if (!response.ok) {
      let detail = ""; try { const value = await response.json(); detail = typeof value.detail === "string" ? value.detail : value.error || ""; } catch (_) {}
      const error = new Error(response.status === 401 || response.status === 403 ? "Enter the laptop session token to connect." : detail || `Depth source unavailable (${response.status}).`);
      error.status = response.status; throw error;
    }
    return blob ? await response.blob() : await response.json();
  } finally { clearTimeout(timeout); }
}

function fail(error) {
  online = false;
  $("scan-alert").hidden = false;
  $("scan-alert").textContent = (error?.name === "AbortError" ? "The depth source timed out." : error?.message || "The depth source is unavailable.") + (lastMesh ? " Last surface retained for inspection; it is no longer live." : "");
  updateFreshness();
}

function validateMesh(mesh) {
  if (!mesh || !Array.isArray(mesh.vertices) || !Array.isArray(mesh.colors) || !Array.isArray(mesh.indices) || !["camera-local", "scan-origin"].includes(mesh.frame)) throw new Error("The server did not provide a supported depth mesh.");
  const vertices = mesh.vertices, indices = mesh.indices;
  if (vertices.length % 3 || indices.length % 3 || mesh.colors.length !== vertices.length || vertices.length > 900000 || indices.length > 1800000) throw new Error("Invalid or unexpectedly large mesh dimensions.");
  if (!vertices.every(finite) || !mesh.colors.every(finite) || !indices.every((i) => Number.isInteger(i) && i >= 0 && i < vertices.length / 3)) throw new Error("Invalid coordinates, colors, or triangle indices in the depth mesh.");
}

function meshColors(mesh, kind) {
  if (kind !== "height") {
    const colors = new Float32Array(mesh.colors.length), color = new THREE.Color();
    for (let i = 0; i < mesh.colors.length; i += 3) {
      color.setRGB(mesh.colors[i], mesh.colors[i + 1], mesh.colors[i + 2], THREE.SRGBColorSpace);
      colors[i] = color.r; colors[i + 1] = color.g; colors[i + 2] = color.b;
    }
    return colors;
  }
  const colors = new Float32Array(mesh.vertices.length);
  let low = Infinity, high = -Infinity;
  for (let i = 1; i < mesh.vertices.length; i += 3) { low = Math.min(low, mesh.vertices[i]); high = Math.max(high, mesh.vertices[i]); }
  const stops = [new THREE.Color(0x527096), new THREE.Color(0x6fa3a7), new THREE.Color(0xbad3a3), new THREE.Color(0xf1c291)];
  const color = new THREE.Color();
  for (let i = 0; i < mesh.vertices.length; i += 3) {
    const t = Math.max(0, Math.min(1, (mesh.vertices[i + 1] - low) / Math.max(.01, high - low))) * 3;
    const index = Math.min(2, Math.floor(t)); color.copy(stops[index]).lerp(stops[index + 1], t - index);
    colors[i] = color.r; colors[i + 1] = color.g; colors[i + 2] = color.b;
  }
  return colors;
}

function replaceSurface(mesh) {
  if (!renderer || failedWebGL) return;
  if (surface) { scene.remove(surface); surface.geometry.dispose(); surface.material.dispose(); surface = null; }
  if (wire) { scene.remove(wire); wire.geometry.dispose(); wire.material.dispose(); wire = null; }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(mesh.vertices, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(meshColors(mesh, style), 3));
  geometry.setIndex(mesh.indices); geometry.computeBoundingBox(); geometry.computeBoundingSphere();
  surface = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide, transparent: style === "wireframe", opacity: style === "wireframe" ? .2 : 1, depthWrite: style !== "wireframe", polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 }));
  scene.add(surface);
  if (style === "wireframe") {
    wire = new THREE.LineSegments(new THREE.WireframeGeometry(geometry), new THREE.LineBasicMaterial({ color: 0xafc8bf, transparent: true, opacity: .7 })); scene.add(wire);
  }
  updateSourcePose(mesh);
  if (!fitted && mesh.indices.length) { fitted = true; homeView(); }
  $("viewport-empty").hidden = mesh.indices.length > 0;
  if (!mesh.indices.length) { $("viewport-empty").querySelector("strong").textContent = "No connected surface in this frame"; $("viewport-empty").querySelector("p").textContent = "Depth samples could not form valid triangles. Missing space stays unfilled."; }
}

function updateSourcePose(mesh) {
  if (!sourceCamera) return;
  if (Array.isArray(mesh.camera_position) && mesh.camera_position.length === 3 && mesh.camera_position.every(finite)) sourceCamera.position.fromArray(mesh.camera_position);
  if (mesh.camera_rotation?.flat().length === 9 && mesh.camera_rotation.flat().every(finite)) {
    const r = mesh.camera_rotation.flat();
    sourceCamera.setRotationFromMatrix(new THREE.Matrix4().set(r[0],r[1],r[2],0,r[3],r[4],r[5],0,r[6],r[7],r[8],0,0,0,0,1));
  } else sourceCamera.rotation.set(0, 0, 0);
}

function updatePins(mesh) {
  const observations = (Array.isArray(mesh.observations) ? mesh.observations : []).filter((item) =>
    Array.isArray(item.position) && item.position.length === 3 && item.position.every(finite) && item.source_frame_id != null && String(item.source_frame_id) === String(mesh.frame_id));
  const signature = JSON.stringify([mesh.frame_id, observations]);
  if (signature === pinSignature) return;
  pinSignature = signature;
  if (scene) for (const pin of pins) { pin.traverse((child) => { child.geometry?.dispose(); if (child.material) { child.material.map?.dispose(); child.material.dispose(); } }); scene.remove(pin); }
  pins = []; $("mesh-observations").replaceChildren();
  if (!observations.length) { const note = document.createElement("p"); note.textContent = "No frame-matched observations."; $("mesh-observations").append(note); return; }
  for (const item of observations) {
    const person = item.kind === "person", text = person ? "Person observation" : item.kind === "smoke_candidate" ? "Smoke candidate" : "Fire candidate";
    const card = document.createElement("div"); card.className = `mesh-observation${person ? " person" : " hazard"}`;
    const dot = document.createElement("i"), content = document.createElement("div"), title = document.createElement("strong"), detail = document.createElement("span");
    title.textContent = text; detail.textContent = `${finite(item.confidence) ? Math.round(item.confidence * 100) + "% · " : ""}${item.source || "Source unavailable"}`;
    content.append(title, detail); card.append(dot, content); $("mesh-observations").append(card);
    if (!scene) continue;
    const color = person ? 0x86c5d0 : 0xefb476, pin = new THREE.Group(); pin.position.fromArray(item.position);
    const sphere = new THREE.Mesh(new THREE.SphereGeometry(.035, 12, 10), new THREE.MeshBasicMaterial({ color })); pin.add(sphere);
    const stem = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, .18, 0)]), new THREE.LineBasicMaterial({ color })); pin.add(stem);
    const labelCanvas = document.createElement("canvas"); labelCanvas.width = 384; labelCanvas.height = 88;
    const ctx = labelCanvas.getContext("2d"); ctx.fillStyle = "rgba(17,35,44,.92)"; ctx.beginPath(); ctx.roundRect(4, 5, 376, 76, 12); ctx.fill();
    ctx.fillStyle = person ? "#acd8df" : "#edc49a"; ctx.font = "600 28px Segoe UI, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(text, 192, 43, 358);
    const texture = new THREE.CanvasTexture(labelCanvas); texture.colorSpace = THREE.SRGBColorSpace;
    const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false, depthWrite: false })); label.position.y = .23; label.scale.set(.56, .128, 1); label.renderOrder = 5; pin.add(label);
    scene.add(pin); pins.push(pin);
  }
}

function applyMesh(mesh) {
  validateMesh(mesh);
  online = true; receivedAt = performance.now(); lastMesh = mesh;
  $("scan-alert").hidden = true;
  const synthetic = mesh.synthetic === true || mesh.mode === "demo";
  $("source-badge").className = `badge ${synthetic ? "synthetic" : "hardware"}`;
  $("source-badge").textContent = synthetic ? "SYNTHETIC DEPTH" : remote ? "RASPBERRY PI LIVE" : "HARDWARE DEPTH";
  $("source-description").textContent = synthetic ? "Synthetic camera-local depth surface. A visual demonstration, not a real room scan." : "Measured camera-local depth surface. Inspect visible surfaces from the RealSense camera.";
  $("viewport-source").textContent = synthetic ? "SYNTHETIC DEPTH MESH" : "MEASURED LOCAL DEPTH SURFACE";
  $("frame-label").textContent = `FRAME ${mesh.frame_id ?? "—"} · ${mesh.units || "metres"}`;
  $("vertices").textContent = Number(mesh.vertex_count ?? mesh.vertices.length / 3).toLocaleString();
  $("triangles").textContent = Number(mesh.triangle_count ?? mesh.indices.length / 3).toLocaleString();
  const minimum = mesh.bounds?.min, maximum = mesh.bounds?.max;
  if (Array.isArray(minimum) && Array.isArray(maximum) && finite(minimum[2]) && finite(maximum[2])) {
    $("depth-range").textContent = `${Math.max(0, -maximum[2]).toFixed(1)}–${Math.max(0, -minimum[2]).toFixed(1)} m`;
  } else $("depth-range").textContent = "—";
  $("mesh-note").textContent = mesh.note || "Camera-local triangulated depth; invalid samples and depth discontinuities remain gaps. This is not a persistent house map.";
  const revision = `${mesh.frame}:${mesh.session ?? 0}:${mesh.revision ?? mesh.frame_id}`;
  if (revision !== meshRevision) { replaceSurface(mesh); meshRevision = revision; }
  updateSourcePose(mesh);
  if (mesh.frame === "scan-origin") {
    $("source-description").textContent = "Retained measured surfaces · experimental RGB-D visual odometry";
    $("viewport-source").textContent = "RETAINED SCAN · ESTIMATED CAMERA POSE";
    $("viewport").querySelector(".viewport-corner > span:last-child").textContent = "FIRST CAMERA VIEW REFERENCE";
    document.querySelector(".viewport-header strong").textContent = "Retained surface reconstruction";
    document.querySelector(".inspector-intro").textContent = "Surfaces persist across accepted camera poses. This prototype has no loop closure; drift can accumulate.";
    document.querySelector(".inspector-note").textContent = "Coordinates are relative to the first camera view. Start level at the configured camera height; ground orientation is not measured.";
    $("room-status").textContent = `${mesh.tracking?.toUpperCase()} · ${mesh.keyframes}/${mesh.capacity} keyframes · ${mesh.quality?.inliers ?? 0} pose inliers. ${mesh.reason}`;
    $("room-stop").disabled = !["starting", "tracking"].includes(mesh.tracking);
    $("depth-range").textContent = "Scan origin";
    if (mesh.tracking === "lost" || mesh.tracking === "capacity") { $("scan-alert").hidden = false; $("scan-alert").textContent = mesh.reason + ". Last accepted surfaces retained; export before starting again."; }
  }
  updatePins(mesh);
  $("home-view").disabled = !surface; $("top-view").disabled = !surface; $("freeze").disabled = false;
  $("export-mesh").disabled = false;
  updateFreshness();
}

function age() {
  if (!lastMesh || !finite(lastMesh.age_s)) return null;
  return lastMesh.age_s + (performance.now() - receivedAt) / 1000;
}

function updateFreshness() {
  const seconds = age(), stale = !online || seconds === null || seconds > (retained ? 1 : .45) || (retained && lastMesh?.tracking !== "tracking");
  $("frame-age").textContent = seconds === null ? "—" : `${seconds.toFixed(2)} s`;
  $("freshness-badge").className = `freshness ${paused || stale ? "stale" : "fresh"}`;
  $("freshness-badge").lastChild.textContent = paused ? "Frozen snapshot" : !lastMesh ? "No frame yet" : stale ? "Retained / not live" : retained ? "Pose tracking" : "Fresh depth";
  $("freeze").classList.toggle("active", paused);
  $("freeze").querySelector("span").textContent = paused ? "Resume updates" : "Freeze view";
}

async function pollMesh() {
  if (polling || paused || document.hidden) return;
  polling = true; const version = tokenVersion;
  try { const mesh = await api(retained ? "/api/room-map" : "/api/mesh"); if (version === tokenVersion && !paused) applyMesh(mesh); }
  catch (error) { if (version === tokenVersion) fail(error); }
  finally { polling = false; }
}

async function pollDepth() {
  if (depthBusy || paused || document.hidden || !online) return;
  depthBusy = true; const version = tokenVersion;
  try {
    const blob = await api("/api/frame.jpg?view=depth", true);
    if (version !== tokenVersion || paused) return;
    const url = URL.createObjectURL(blob), old = depthURL; depthURL = url;
    $("depth-preview").src = url; $("depth-preview").hidden = false; $("depth-empty").hidden = true;
    if (old) URL.revokeObjectURL(old);
  } catch (_) { /* Mesh status is authoritative; inset is optional. */ }
  finally { depthBusy = false; }
}

function targetBounds() {
  if (!surface) return null;
  const box = surface.geometry.boundingBox;
  if (!box || box.isEmpty()) return null;
  const center = box.getCenter(new THREE.Vector3()), size = box.getSize(new THREE.Vector3());
  return { box, center, span: Math.max(size.x, size.y, size.z, .5) };
}

function homeView() {
  const bounds = targetBounds(); if (!bounds) return;
  controls.target.copy(bounds.center);
  camera.up.set(0, 1, 0);
  camera.position.set(bounds.center.x + bounds.span * .46, bounds.center.y + bounds.span * .35, bounds.box.max.z + bounds.span * .92);
  controls.minDistance = Math.max(.05, bounds.span * .03); controls.maxDistance = Math.max(15, bounds.span * 6);
  camera.near = Math.max(.005, bounds.span / 1000); camera.far = Math.max(60, bounds.span * 15); camera.updateProjectionMatrix(); controls.update();
}

function topView() {
  const bounds = targetBounds(); if (!bounds) return;
  controls.target.copy(bounds.center); camera.up.set(0, 0, -1);
  camera.position.set(bounds.center.x, bounds.box.max.y + bounds.span * 1.2, bounds.center.z + .001); controls.update();
}

function changeStyle(next) {
  style = next;
  for (const name of ["rgb", "height", "wireframe"]) {
    $("style-" + name).classList.toggle("active", name === next);
    $("style-" + name).setAttribute("aria-selected", String(name === next));
    $("style-" + name).tabIndex = name === next ? 0 : -1;
  }
  $("height-legend").hidden = next !== "height";
  if (lastMesh) replaceSurface(lastMesh);
}

for (const name of ["rgb", "height", "wireframe"]) {
  $("style-" + name).addEventListener("click", () => changeStyle(name));
  $("style-" + name).addEventListener("keydown", (event) => {
    if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
    event.preventDefault(); const names = ["rgb", "height", "wireframe"];
    const next = event.key === "Home" ? "rgb" : event.key === "End" ? "wireframe" : names[(names.indexOf(name) + (event.key === "ArrowRight" ? 1 : 2)) % 3]; changeStyle(next); $("style-" + next).focus();
  });
}
$("home-view").addEventListener("click", homeView);
$("top-view").addEventListener("click", topView);
$("grid-toggle").addEventListener("change", () => { if (referenceGrid) referenceGrid.visible = $("grid-toggle").checked; if (axes) axes.visible = $("grid-toggle").checked; });
$("freeze").addEventListener("click", () => { paused = !paused; updateFreshness(); if (!paused) { pollMesh(); pollDepth(); } });
$("scan-source").addEventListener("change", () => { const url = new URL(location.href); if ($("scan-source").value === "pi") url.searchParams.set("source", "pi"); else url.searchParams.delete("source"); location.assign(url.pathname + url.search); });
$("export-mesh").addEventListener("click", () => {
  if (!lastMesh) return;
  const report = { exported_at_utc: new Date().toISOString(), view_source: remote ? "joined_pi" : "local_service", snapshot_age_s: age(), note: lastMesh.note, mesh: lastMesh };
  const url = URL.createObjectURL(new Blob([JSON.stringify(report) + "\n"], { type: "application/json" }));
  const anchor = document.createElement("a"); anchor.href = url; anchor.download = `WARM-wheels_depth-frame-${lastMesh.frame_id ?? "unknown"}.json`; document.body.append(anchor); anchor.click(); anchor.remove(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  toast(retained ? "Retained room scan exported." : "Camera-local depth snapshot exported.");
});
$("room-start").addEventListener("click", async () => {
  $("room-start").disabled = true;
  try { await api("/api/room-map-control", false, { action: "start" }); const url = new URL(location.href); url.searchParams.set("map", "room"); location.assign(url.pathname + url.search); }
  catch (error) { toast(error.message); }
  finally { $("room-start").disabled = false; }
});
$("room-stop").addEventListener("click", async () => { try { await api("/api/room-map-control", false, { action: "stop" }); pollMesh(); } catch (error) { toast(error.message); } });
$("room-local").addEventListener("click", () => { const url = new URL(location.href); url.searchParams.delete("map"); location.assign(url.pathname + url.search); });
$("scan-fullscreen").addEventListener("click", async () => { try { if (document.fullscreenElement) await document.exitFullscreen(); else await viewport.requestFullscreen(); } catch (_) { toast("Fullscreen is unavailable here. Try your browser’s fullscreen shortcut."); } });
$("session-button").addEventListener("click", () => { $("scan-session").showModal(); $("scan-token").focus(); });
$("close-session").addEventListener("click", () => $("scan-session").close());
$("scan-token-form").addEventListener("submit", (event) => {
  event.preventDefault(); token = $("scan-token").value.trim(); tokenVersion++;
  try { if (token) sessionStorage.setItem("warm-wheels-token", token); else sessionStorage.removeItem("warm-wheels-token"); } catch (_) {}
  $("scan-session").close(); online = false; pollMesh();
});
document.addEventListener("visibilitychange", () => { if (!document.hidden) pollMesh(); });
window.addEventListener("pagehide", () => { if (depthURL) URL.revokeObjectURL(depthURL); });
initScene();
setInterval(pollMesh, retained ? 750 : 250); setInterval(pollDepth, 750); setInterval(updateFreshness, 150);
pollMesh();
