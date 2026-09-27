import * as THREE from "/static/vendor/three.module.js";
import { OrbitControls } from "/static/vendor/OrbitControls.js";

const validPoint = (p) => Array.isArray(p) && p.length >= 2 && p.slice(0, 2).every(Number.isFinite);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

export class MissionViewport {
  constructor(canvas, placeholder) {
    this.canvas = canvas; this.placeholder = placeholder; this.state = null; this.available = false;
    this.lastTime = 0; this.pose = null; this.animation = null; this.oldTrail = []; this.online = true;
    this.reduced = matchMedia("(prefers-reduced-motion: reduce)").matches;
    this.dynamic = []; this.entities = new Map(); this.layoutKey = ""; this.meshRevision = null; this.style = "surface";
    this.cutaway = true; this.cutPlane = new THREE.Plane(new THREE.Vector3(0, -1, 0), 1.0);
    this.visualTime = 0; this.lastRendered = 0; this.frameRequest = null; this.drawSize = new THREE.Vector2();
    try {
      this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: "high-performance" });
      this.renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 1.5));
      this.renderer.setClearColor(0x13251f); this.renderer.outputColorSpace = THREE.SRGBColorSpace;
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping; this.renderer.toneMappingExposure = .95;
      this.renderer.localClippingEnabled = true;
      this.scene = new THREE.Scene(); this.scene.fog = new THREE.Fog(0x13251f, 22, 80);
      this.camera = new THREE.PerspectiveCamera(42, 1, .02, 150);
      this.controls = new OrbitControls(this.camera, canvas); this.controls.enableDamping = true; this.controls.dampingFactor = .075;
      this.controls.maxPolarAngle = Math.PI * .49; this.controls.minDistance = .8; this.controls.maxDistance = 100;
      this.controls.target.set(0, 0, 0); this.camera.position.set(6, 7, 8); this.controls.update();
      this.scene.add(new THREE.HemisphereLight(0xe9f1ce, 0x354c41, 1.7));
      const sun = new THREE.DirectionalLight(0xffe5b6, 1.25); sun.position.set(-4, 11, 6);
      sun.shadow.mapSize.set(2048, 2048); sun.shadow.camera.left = -8; sun.shadow.camera.right = 8; sun.shadow.camera.top = 8; sun.shadow.camera.bottom = -8;
      sun.shadow.bias = -.001; sun.shadow.normalBias = .025; this.sun = sun; this.scene.add(sun);
      const rim = new THREE.DirectionalLight(0x96bdab, .7); rim.position.set(6, 4, -6); this.scene.add(rim);
      // Architecture comes only from accumulated raycast depth triangles. The
      // occupancy grid is reserved for paths, never extruded into wall cubes.
      this.meshGeometry = new THREE.BufferGeometry();
      this.surface = new THREE.Mesh(this.meshGeometry, new THREE.MeshStandardMaterial({ vertexColors: true, roughness: .9, metalness: .02, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 }));
      this.wire = new THREE.Mesh(this.meshGeometry, new THREE.MeshBasicMaterial({ color: 0x94b6a0, wireframe: true, transparent: true, opacity: .04, depthWrite: false }));
      this.points = new THREE.Points(this.meshGeometry, new THREE.PointsMaterial({ vertexColors: true, size: .025, transparent: true, opacity: .9 }));
      this.scene.add(this.surface, this.wire, this.points);
      this.fresh = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0xd3ffd8, size: .025, transparent: true, opacity: .8, depthWrite: false }));
      this.rays = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xc1e5bb, transparent: true, opacity: .055, depthWrite: false }));
      this.scene.add(this.fresh, this.rays);
      this.rover = this.makeRover(); this.scene.add(this.rover);
      this.scanRing = new THREE.Mesh(new THREE.RingGeometry(.97, 1, 80), new THREE.MeshBasicMaterial({ color: 0xd4e8a8, transparent: true, opacity: .15, side: THREE.DoubleSide, depthWrite: false }));
      this.scanRing.rotation.x = -Math.PI / 2; this.scanRing.position.y = .025; this.scene.add(this.scanRing);
      this.frontiers = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0xf0c88b, size: .045, transparent: true, opacity: .85 })); this.scene.add(this.frontiers);
      this.homeMarker = new THREE.Group();
      const pad = new THREE.Mesh(new THREE.CylinderGeometry(.25, .25, .025, 48), new THREE.MeshStandardMaterial({ color: 0xb8c39a, roughness: .75 })); pad.position.y = .015; this.homeMarker.add(pad);
      const ring = new THREE.Mesh(new THREE.TorusGeometry(.27, .012, 8, 48), new THREE.MeshBasicMaterial({ color: 0xe8dca6 })); ring.rotation.x = Math.PI / 2; ring.position.y = .035; this.homeMarker.add(ring);
      const homeLabel = this.label("HOME", "#eadfb8", .48); homeLabel.position.y = .55; this.homeMarker.add(homeLabel); this.scene.add(this.homeMarker);
      this.available = true; this.setStyle(this.style); this.setCutaway(this.cutaway);
      this.loop = (now) => {
        this.frameRequest = null;
        if (!this.available || document.hidden || this.pageHidden) return;
        if (now - this.lastRendered >= 1000 / 30) { this.render(now); this.lastRendered = now; }
        this.frameRequest = requestAnimationFrame(this.loop);
      };
      this.resume = () => {
        if (document.hidden || this.pageHidden) { if (this.frameRequest !== null) cancelAnimationFrame(this.frameRequest); this.frameRequest = null; }
        else if (this.frameRequest === null) this.frameRequest = requestAnimationFrame(this.loop);
      };
      document.addEventListener("visibilitychange", this.resume);
      window.addEventListener("pagehide", () => { this.pageHidden = true; this.resume(); });
      window.addEventListener("pageshow", () => { this.pageHidden = false; this.resume(); });
      canvas.addEventListener("webglcontextlost", (event) => {
        event.preventDefault(); this.available = false;
        placeholder.hidden = false; placeholder.querySelector("strong").textContent = "3D graphics were interrupted";
        placeholder.querySelector("span:last-child").textContent = "Reload this view to restore the depth mesh.";
      });
      this.resume();
    } catch (_) {
      placeholder.hidden = false; placeholder.querySelector("strong").textContent = "3D graphics are unavailable";
      placeholder.querySelector("span:last-child").textContent = "Open in a WebGL-enabled browser. There is no flat fallback for this 3D scene.";
    }
  }

  label(text, color = "#cbdab6", scale = .6) {
    const canvas = document.createElement("canvas"); canvas.width = 384; canvas.height = 96;
    const ctx = canvas.getContext("2d"); ctx.fillStyle = "rgba(23,42,29,.85)"; ctx.beginPath(); ctx.roundRect(8, 9, 368, 78, 13); ctx.fill();
    ctx.fillStyle = color; ctx.font = "600 29px Segoe UI, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(text, 192, 49, 345);
    const texture = new THREE.CanvasTexture(canvas); texture.colorSpace = THREE.SRGBColorSpace;
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false, depthWrite: false })); sprite.scale.set(scale, scale / 4, 1); sprite.renderOrder = 10;
    return sprite;
  }

  makeRover() {
    const rover = new THREE.Group(), green = new THREE.MeshStandardMaterial({ color: 0xd4dfbc, roughness: .45 }), dark = new THREE.MeshStandardMaterial({ color: 0x20362a, roughness: .8 });
    const chassis = new THREE.Mesh(new THREE.BoxGeometry(.34, .09, .27), green); chassis.position.y = .12; chassis.castShadow = true; rover.add(chassis);
    const upper = new THREE.Mesh(new THREE.BoxGeometry(.22, .05, .2), dark); upper.position.set(-.025, .19, 0); upper.castShadow = true; rover.add(upper);
    const lidar = new THREE.Mesh(new THREE.CylinderGeometry(.045, .045, .035, 24), new THREE.MeshStandardMaterial({ color: 0xb6ce93, metalness: .2, roughness: .35 })); lidar.position.set(-.04, .23, 0); rover.add(lidar);
    for (const x of [-.105, .105]) for (const z of [-.16, .16]) {
      const wheel = new THREE.Mesh(new THREE.CylinderGeometry(.067, .067, .045, 16), dark); wheel.rotation.x = Math.PI / 2; wheel.position.set(x, .075, z); wheel.castShadow = true; rover.add(wheel);
      const hub = new THREE.Mesh(new THREE.CylinderGeometry(.028, .028, .048, 12), green); hub.rotation.x = Math.PI / 2; hub.position.copy(wheel.position); rover.add(hub);
    }
    const camera = new THREE.Mesh(new THREE.BoxGeometry(.035, .06, .17), dark); camera.position.set(.18, .165, 0); rover.add(camera);
    for (const z of [-.05, .05]) { const lens = new THREE.Mesh(new THREE.SphereGeometry(.018, 12, 8), new THREE.MeshBasicMaterial({ color: 0xc0e8e6 })); lens.position.set(.2, .17, z); rover.add(lens); }
    return rover;
  }

  particleTexture() {
    const canvas = document.createElement("canvas"); canvas.width = 64; canvas.height = 64;
    const ctx = canvas.getContext("2d"), gradient = ctx.createRadialGradient(32, 32, 1, 32, 32, 31);
    gradient.addColorStop(0, "rgba(255,255,255,1)"); gradient.addColorStop(.28, "rgba(255,255,255,.8)"); gradient.addColorStop(.65, "rgba(255,255,255,.2)"); gradient.addColorStop(1, "rgba(255,255,255,0)");
    ctx.fillStyle = gradient; ctx.fillRect(0, 0, 64, 64); return new THREE.CanvasTexture(canvas);
  }

  makeEntity(entity) {
    const group = new THREE.Group(), data = group.userData;
    if (entity.kind === "person") {
      const body = new THREE.Group(); data.body = body; group.add(body);
      const cloth = new THREE.MeshStandardMaterial({ color: 0xb7ca99, roughness: .9 }), dark = new THREE.MeshStandardMaterial({ color: 0x516b65, roughness: .9 }); data.cloth = cloth;
      const torso = new THREE.Mesh(new THREE.CapsuleGeometry(.14, .38, 8, 14), cloth); torso.position.y = 1.1; body.add(torso);
      const head = new THREE.Mesh(new THREE.SphereGeometry(.12, 20, 14), new THREE.MeshStandardMaterial({ color: 0xd9b38f, roughness: .8 })); head.position.y = 1.56; body.add(head);
      for (const x of [-.09, .09]) { const leg = new THREE.Mesh(new THREE.CapsuleGeometry(.057, .55, 6, 12), dark); leg.position.set(x, .4, 0); body.add(leg); }
      for (const x of [-.21, .21]) { const arm = new THREE.Mesh(new THREE.CapsuleGeometry(.048, .42, 6, 12), cloth); arm.position.set(x, 1.06, 0); arm.rotation.z = x < 0 ? -.12 : .12; body.add(arm); }
      const disk = new THREE.Mesh(new THREE.RingGeometry(.25, .27, 64), new THREE.MeshBasicMaterial({ color: 0xc3da9d, side: THREE.DoubleSide, transparent: true, opacity: .85 })); disk.rotation.x = -Math.PI / 2; disk.position.y = .027; group.add(disk);
    } else if (entity.kind === "hazard" || entity.kind === "fire") {
      const profile = [[0,0],[.18,.035],[.23,.17],[.17,.4],[.105,.65],[.065,.82],[0,1]].map(([x,y]) => new THREE.Vector2(x,y));
      const geometry = new THREE.LatheGeometry(profile, 22), material = new THREE.MeshStandardMaterial({ color: 0xfc9a35, emissive: 0xff620b, emissiveIntensity: 1.05, transparent: true, opacity: .76, roughness: .9, depthWrite: false, side: THREE.DoubleSide });
      data.flames = [];
      for (let i = 0; i < 3; i++) { const flame = new THREE.Mesh(geometry, material); flame.position.set((i - 1) * .1, .035, i % 2 ? .05 : -.035); group.add(flame); data.flames.push(flame); }
      const particles = new THREE.BufferGeometry(); particles.setAttribute("position", new THREE.BufferAttribute(new Float32Array(48 * 3), 3));
      const colors = new Float32Array(48 * 3); for (let i = 0; i < 48; i++) colors.set([1, .32 + i % 7 * .07, .045], i * 3); particles.setAttribute("color", new THREE.BufferAttribute(colors, 3));
      data.particles = new THREE.Points(particles, new THREE.PointsMaterial({ size: .28, vertexColors: true, map: this.particleTexture(), transparent: true, opacity: .8, blending: THREE.AdditiveBlending, depthWrite: false })); data.particles.frustumCulled = false; group.add(data.particles);
      const smoke = new THREE.BufferGeometry(); smoke.setAttribute("position", new THREE.BufferAttribute(new Float32Array(18 * 3), 3));
      data.smoke = new THREE.Points(smoke, new THREE.PointsMaterial({ size: .45, color: 0x9aa39b, map: this.particleTexture(), transparent: true, opacity: .19, depthWrite: false })); data.smoke.frustumCulled = false; group.add(data.smoke);
      data.spread = new THREE.InstancedMesh(new THREE.CircleGeometry(1, 32), new THREE.MeshBasicMaterial({ color: 0xe99045, transparent: true, opacity: .13, side: THREE.DoubleSide, depthWrite: false }), 192);
      data.spread.instanceMatrix.setUsage(THREE.DynamicDrawUsage); data.spread.count = 0; group.add(data.spread);
      data.halo = new THREE.Mesh(new THREE.RingGeometry(.97, 1, 80), new THREE.MeshBasicMaterial({ color: 0xffbc69, transparent: true, opacity: .55, side: THREE.DoubleSide, depthWrite: false })); data.halo.rotation.x = -Math.PI / 2; data.halo.position.y = .025; group.add(data.halo);
    } else {
      const ring = new THREE.Mesh(new THREE.TorusGeometry(.23, .017, 8, 48), new THREE.MeshBasicMaterial({ color: 0xe9ad77 })); ring.rotation.x = Math.PI / 2; ring.position.y = .04; group.add(ring);
    }
    return group;
  }

  updateEntity(entry, entity) {
    const data = entry.object.userData, assessment = entity.assessment?.state || "observing";
    let text, color = "#cfe5b6", height = 1.91;
    if (data.body) {
      // Only an explicit simulated posture changes the body. A missing response
      // alone is not a diagnosis or evidence of a person lying down.
      const posture = entity.posture || entity.assessment?.posture || "standing";
      data.body.rotation.set(0, Number(entity.heading_rad) || 0, posture === "prone" ? -Math.PI / 2 : 0);
      data.body.position.set(posture === "prone" ? -.68 : 0, posture === "prone" ? .16 : 0, 0);
      data.body.scale.set(1, ["seated", "crouched"].includes(posture) ? .64 : 1, 1);
      data.cloth.color.set(["assistance_needed", "no_response"].includes(assessment) ? 0xd8ac78 : 0xb7ca99);
      text = entity.visible === false ? "PERSON · LAST SEEN" : { observing: "PERSON · ASSESSING", mobile: "PERSON · MOVING", assistance_needed: "ASSISTANCE REQUEST", no_response: "NO RESPONSE OBSERVED" }[assessment] || "PERSON OBSERVED";
      color = ["assistance_needed", "no_response"].includes(assessment) ? "#f1c494" : "#cfe5b6";
      height = posture === "prone" ? .52 : ["seated", "crouched"].includes(posture) ? 1.27 : 1.91;
      if (data.observedVisible !== (entity.visible !== false)) {
        data.observedVisible = entity.visible !== false;
        data.body.traverse((child) => { if (child.material) { child.material.transparent = !data.observedVisible; child.material.opacity = data.observedVisible ? 1 : .32; child.material.depthWrite = data.observedVisible; child.material.needsUpdate = true; } });
      }
    } else if (data.flames) {
      data.intensity = clamp(Number(entity.intensity) || .25, .08, 1); data.radius = clamp(Number(entity.radius_m) || .25, .12, 6);
      data.halo.scale.setScalar(data.radius);
      const cells = (entity.spread_cells || []).filter(validPoint).slice(0, 192), dummy = new THREE.Object3D();
      const key = `${cells.map((p) => p.join(",")).join(";")}:${entity.position.join(",")}`;
      if (key !== data.spreadKey) {
        data.spreadKey = key;
        for (let i = 0; i < cells.length; i++) { dummy.position.set((cells[i][0] - entity.position[0]) * this.resolution, .019, (cells[i][1] - entity.position[1]) * this.resolution); dummy.rotation.set(-Math.PI / 2, 0, 0); dummy.scale.setScalar(this.resolution * .7); dummy.updateMatrix(); data.spread.setMatrixAt(i, dummy.matrix); }
        data.spread.count = cells.length; data.spread.instanceMatrix.needsUpdate = true; data.spread.computeBoundingSphere();
      }
      text = `FIRE · ${data.intensity > .72 ? "INTENSIFYING" : data.intensity > .4 ? "GROWING" : "OBSERVED"}`; color = "#f4c48b"; height = .93 + data.intensity * .75;
    } else { text = "OBSTRUCTION"; color = "#e9b483"; height = .45; }
    if (data.labelKey !== text) { if (data.label) this.disposeObject(data.label); data.label = this.label(text, color, entity.kind === "person" ? 1.2 : 1.1); entry.object.add(data.label); data.labelKey = text; }
    data.label.position.y = height;
  }

  world(point, y = .025) { return new THREE.Vector3((point[0] + .5) * this.resolution - this.width / 2, y, (point[1] + .5) * this.resolution - this.height / 2); }

  setStyle(value) {
    if (!["surface", "wireframe", "points"].includes(value)) return;
    this.style = value;
    if (!this.surface) return;
    this.surface.visible = value === "surface";
    this.wire.visible = value !== "points"; this.wire.material.opacity = value === "wireframe" ? .6 : .04;
    this.points.visible = value === "points"; this.fresh.visible = value !== "points";
  }

  setCutaway(value) {
    this.cutaway = Boolean(value);
    // This clips the displayed scan only. Full measured vertices remain available
    // for full-height inspection, point mode, subsequent scans, and export.
    for (const object of [this.surface, this.wire, this.points, this.fresh, this.rays]) {
      if (!object) continue;
      object.material.clippingPlanes = this.cutaway ? [this.cutPlane] : [];
      object.material.needsUpdate = true;
    }
  }

  updateMesh(mesh, reset, now) {
    const revision = mesh?.revision ?? null;
    if (!reset && revision !== null && revision === this.meshRevision) return;
    this.meshRevision = revision;
    const valid3 = (p) => Array.isArray(p) && p.length >= 3 && p.slice(0, 3).every(Number.isFinite);
    const vertices = Array.isArray(mesh?.vertices) ? mesh.vertices : [];
    const colors = Array.isArray(mesh?.colors) ? mesh.colors : [];
    const count = Math.min(vertices.length, 20000), positions = new Float32Array(count * 3), rgb = new Float32Array(count * 3), valid = new Uint8Array(count);
    for (let i = 0; i < count; i++) {
      const p = vertices[i]; if (!valid3(p)) continue; valid[i] = 1;
      positions.set([p[0] - this.width / 2, p[1], p[2] - this.height / 2], i * 3);
      const color = valid3(colors[i]) ? colors[i] : [.49, .63, .53];
      rgb.set(color.slice(0, 3).map((v) => clamp(v, 0, 1)), i * 3);
    }
    const indices = [];
    for (const triangle of (mesh?.triangles || []).slice(0, 30000)) {
      if (Array.isArray(triangle) && triangle.length === 3 && triangle.every((i) => Number.isInteger(i) && i >= 0 && i < count && valid[i])) indices.push(...triangle);
    }
    const geometry = new THREE.BufferGeometry(); geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(rgb, 3)); geometry.setIndex(indices); geometry.computeVertexNormals(); geometry.computeBoundingSphere();
    this.surface.geometry = geometry; this.wire.geometry = geometry; this.points.geometry = geometry;
    this.meshGeometry.dispose(); this.meshGeometry = geometry;
    const fresh = (mesh?.latest_points || []).filter(valid3).slice(0, 384);
    this.fresh.geometry.dispose(); this.fresh.geometry = new THREE.BufferGeometry().setFromPoints(fresh.map((p) => new THREE.Vector3(p[0] - this.width / 2, p[1], p[2] - this.height / 2)));
    this.acquiredAt = now;
    const rays = [], origin = mesh?.scan_origin;
    if (valid3(origin)) for (let i = 0; i < fresh.length; i += Math.max(1, Math.ceil(fresh.length / 18))) {
      rays.push(new THREE.Vector3(origin[0] - this.width / 2, origin[1], origin[2] - this.height / 2));
      rays.push(new THREE.Vector3(fresh[i][0] - this.width / 2, fresh[i][1], fresh[i][2] - this.height / 2));
    }
    this.rays.geometry.dispose(); this.rays.geometry = new THREE.BufferGeometry().setFromPoints(rays);
    this.points.material.size = clamp(Math.max(this.width, this.height) * .004, .02, .085);
  }

  disposeObject(object) {
    object.traverse((child) => { if (child.geometry) child.geometry.dispose(); if (child.material) { for (const material of Array.isArray(child.material) ? child.material : [child.material]) { material.map?.dispose(); material.dispose(); } } });
    object.removeFromParent();
  }

  update(state) {
    if (!this.available) return;
    this.placeholder.hidden = true;
    const boundary = state.boundary || {}, grid = state.grid;
    const key = `${state.seed}:${state.layout_hash}:${boundary.width_m}:${boundary.height_m}`;
    const reset = key !== this.layoutKey || (this.state && state.metrics.steps < this.state.metrics.steps);
    this.layoutKey = key; this.width = boundary.width_m || grid.width * grid.resolution_m; this.height = boundary.height_m || grid.height * grid.resolution_m; this.resolution = grid.resolution_m;
    const now = performance.now(), previous = this.state;
    if (!this.pose || reset) { this.pose = [...state.position]; this.animation = null; }
    else if (validPoint(state.position) && (state.position[0] !== this.pose[0] || state.position[1] !== this.pose[1])) {
      const remaining = this.animation ? this.animation.path.slice(Math.min(this.animation.path.length - 1, this.animation.segment + 1)) : [];
      const added = state.trail.length >= this.oldTrail.length ? state.trail.slice(this.oldTrail.length) : [];
      const path = [[...this.pose], ...remaining, ...added];
      if (!path.length || path[path.length - 1][0] !== state.position[0] || path[path.length - 1][1] !== state.position[1]) path.push([...state.position]);
      const clean = path.filter((p, i) => !i || p[0] !== path[i - 1][0] || p[1] !== path[i - 1][1]);
      const distance = clean.slice(1).reduce((sum, p, i) => sum + Math.hypot(p[0] - clean[i][0], p[1] - clean[i][1]), 0);
      this.animation = { path: clean, start: now, duration: this.reduced ? 0 : clamp(now - this.lastTime, 100, 450), distance, segment: 0 };
    }
    this.oldTrail = state.trail || []; this.lastTime = now; this.state = state; this.online = true;
    this.updateMesh(state.scan_mesh, reset, now);
    if (reset) {
      if (this.boundaryLine) this.disposeObject(this.boundaryLine);
      if (this.referenceGrid) this.disposeObject(this.referenceGrid);
      const gridPoints = [], step = Math.max(1, Math.ceil(Math.max(this.width, this.height) / 30));
      for (let x = step; x < this.width; x += step) gridPoints.push(new THREE.Vector3(x - this.width / 2, -.025, -this.height / 2), new THREE.Vector3(x - this.width / 2, -.025, this.height / 2));
      for (let z = step; z < this.height; z += step) gridPoints.push(new THREE.Vector3(-this.width / 2, -.025, z - this.height / 2), new THREE.Vector3(this.width / 2, -.025, z - this.height / 2));
      this.referenceGrid = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(gridPoints), new THREE.LineBasicMaterial({ color: 0x709681, transparent: true, opacity: .12 })); this.scene.add(this.referenceGrid);
      const corners = [[-this.width / 2, -this.height / 2], [this.width / 2, -this.height / 2], [this.width / 2, this.height / 2], [-this.width / 2, this.height / 2], [-this.width / 2, -this.height / 2]].map(([x, z]) => new THREE.Vector3(x, .025, z));
      this.boundaryLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(corners), new THREE.LineBasicMaterial({ color: 0xb5c69a })); this.scene.add(this.boundaryLine);
      if (this.dimensionLabels) this.dimensionLabels.forEach((label) => this.disposeObject(label));
      const labelScale = Math.max(.5, Math.min(2.5, Math.max(this.width, this.height) * .1));
      const widthLabel = this.label(`${this.width} m`, "#b9cba5", labelScale); widthLabel.position.set(0, .12, this.height / 2 + .2);
      const heightLabel = this.label(`${this.height} m`, "#b9cba5", labelScale); heightLabel.position.set(-this.width / 2 - .25, .12, 0);
      this.dimensionLabels = [widthLabel, heightLabel]; this.scene.add(...this.dimensionLabels);
      this.home();
    }
    for (const object of this.dynamic) this.disposeObject(object); this.dynamic = [];
    const addLine = (path, color, dashed, y = .038) => {
      if (!path?.length || path.length < 2) return;
      const geometry = new THREE.BufferGeometry().setFromPoints(path.map((p) => this.world(p, y)));
      const material = dashed ? new THREE.LineDashedMaterial({ color, dashSize: .10, gapSize: .07, transparent: true, opacity: .95 }) : new THREE.LineBasicMaterial({ color, transparent: true, opacity: .68 });
      const line = new THREE.Line(geometry, material); if (dashed) line.computeLineDistances(); this.scene.add(line); this.dynamic.push(line);
    };
    addLine(state.trail, 0x9bc39d, false); addLine(state.route, 0xd9eaa8, true, .047); if (state.follower?.trail) addLine(state.follower.trail, 0x87bec2, false, .045);
    this.frontiers.geometry.dispose(); this.frontiers.geometry = new THREE.BufferGeometry().setFromPoints((state.frontiers || []).map((p) => this.world(p, .045)));
    this.frontiers.material.size = Math.max(.025, Math.min(.08, this.resolution * .3));
    if (validPoint(state.home)) this.homeMarker.position.copy(this.world(state.home, 0));
    const visibleEntities = (state.entities || []).slice(0, 40);
    const liveIds = new Set(visibleEntities.map((entity) => String(entity.id)));
    for (const [id, entry] of this.entities) if (!liveIds.has(id) || reset) { this.disposeObject(entry.object); this.entities.delete(id); }
    for (const entity of visibleEntities) {
      if (!validPoint(entity.position)) continue;
      let entry = this.entities.get(String(entity.id));
      if (!entry) { const object = this.makeEntity(entity); this.scene.add(object); entry = { object, kind: entity.kind }; this.entities.set(String(entity.id), entry); }
      // Invisible people retain only their last measured position, explicitly
      // ghosted by updateEntity; their unobserved simulated movement is withheld.
      entry.object.visible = true; entry.object.position.copy(this.world(entity.position, 0));
      this.updateEntity(entry, entity);
    }
    if (!previous || reset) this.render(now);
  }

  home() {
    if (!this.available) return;
    const span = Math.max(this.width || 6, this.height || 6);
    this.controls.target.set(0, this.cutaway ? .12 : Math.min(.6, span * .1), 0); this.camera.position.set(span * .67, Math.max(3, span * 1.3), span * .8);
    this.camera.near = Math.max(.01, span / 1500); this.camera.far = Math.max(60, span * 10); this.camera.updateProjectionMatrix();
    this.controls.maxDistance = span * 6 + 4; this.controls.minDistance = Math.max(.3, span * .035);
    this.sun.position.set(-span * .65, span * 1.6, span); this.sun.shadow.camera.left = -span; this.sun.shadow.camera.right = span; this.sun.shadow.camera.top = span; this.sun.shadow.camera.bottom = -span; this.sun.shadow.camera.far = span * 6; this.sun.shadow.camera.updateProjectionMatrix();
    this.scene.fog.near = span * 3; this.scene.fog.far = span * 8; this.controls.update();
  }

  setOnline(value) { this.online = value; if (!value) this.animation = null; }

  animatePose(now) {
    if (!this.animation || !this.online) return;
    const a = this.animation, t = a.duration ? clamp((now - a.start) / a.duration, 0, 1) : 1; let remaining = t * a.distance;
    for (let i = 1; i < a.path.length; i++) { const p = a.path[i - 1], q = a.path[i], distance = Math.hypot(q[0] - p[0], q[1] - p[1]);
      if (remaining <= distance || i === a.path.length - 1) { const f = distance ? clamp(remaining / distance, 0, 1) : 1; this.pose = [p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f]; a.segment = i - 1; break; } remaining -= distance; }
    if (t >= 1) { this.pose = [...this.state.position]; this.animation = null; }
  }

  animateFire(data) {
    const time = this.visualTime, strength = data.intensity || .25;
    const radius = Math.min(.65, .2 + (data.radius || .2) * .25), height = .45 + strength * .9;
    for (let i = 0; i < data.flames.length; i++) {
      const flame = data.flames[i], pulse = this.reduced ? 1 : 1 + Math.sin(time * 7 + i * 2) * .1;
      flame.scale.set(radius * 2.6 * (i === 1 ? 1 : .64), height * pulse * (i === 1 ? 1 : .69), radius * 2.1);
      flame.rotation.z = (i - 1) * .2 + (this.reduced ? 0 : Math.sin(time * 3.1 + i) * .065);
    }
    const particles = data.particles.geometry.attributes.position;
    for (let i = 0; i < particles.count; i++) {
      const age = (i * .618033 + time * (.55 + i % 3 * .08)) % 1, angle = i * 2.39996 + age * .6;
      const spread = radius * (.24 + i % 5 * .18) * (1 - age * .7);
      particles.setXYZ(i, Math.cos(angle) * spread + age * age * .12, .04 + age * height * 1.3, Math.sin(angle) * spread);
    }
    particles.needsUpdate = true; data.particles.material.size = .16 + strength * .22;
    const smoke = data.smoke.geometry.attributes.position;
    for (let i = 0; i < smoke.count; i++) {
      const age = (i * .618033 + time * .18) % 1, angle = i * 2.39996;
      smoke.setXYZ(i, Math.cos(angle) * (.12 + age * .28) + age * .23, height * .75 + age * (1.2 + strength), Math.sin(angle) * (.12 + age * .28));
    }
    smoke.needsUpdate = true; data.smoke.material.size = .35 + strength * .38;
  }

  render(now) {
    if (!this.available) return;
    const width = this.canvas.clientWidth, height = this.canvas.clientHeight, size = this.renderer.getSize(this.drawSize);
    if (width && height && (size.x !== width || size.y !== height)) { this.renderer.setSize(width, height, false); this.camera.aspect = width / height; this.camera.updateProjectionMatrix(); }
    this.animatePose(now);
    if (this.state && this.pose) {
      this.rover.position.copy(this.world(this.pose, 0)); this.rover.rotation.y = -(this.state.heading_rad || 0);
      const running = this.online && this.state.playing;
      const scanning = running && !["sensor_hold", "blocked", "escort_wait"].includes(this.state.phase);
      if (running && !this.reduced) this.visualTime += clamp((now - (this.visualTick || now)) / 1000, 0, .08);
      this.visualTick = now;
      this.scanRing.visible = scanning;
      this.rays.visible = scanning;
      this.fresh.material.opacity = scanning ? .3 + .55 * Math.max(0, 1 - (now - (this.acquiredAt || 0)) / 800) : .18;
      if (scanning) { const phase = this.reduced ? .5 : (this.visualTime % 2.8) / 2.8; const radius = .25 + phase * Math.min(3, Math.max(.65, this.resolution * 8)); this.scanRing.position.copy(this.world(this.pose, .03)); this.scanRing.scale.setScalar(radius); this.scanRing.material.opacity = (1 - phase) * .15; }
      for (const entry of this.entities.values()) if (entry.object.userData.flames) this.animateFire(entry.object.userData);
    }
    this.controls.update(); this.renderer.render(this.scene, this.camera);
  }
}
