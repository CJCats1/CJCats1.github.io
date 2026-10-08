import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const scenes = [
  { key: "spring", name: "SPRING / MASS", description: "A movable anchor, gravity, damping, and a visual spring.", notes: "The first renderer slice keeps the oscillator intentionally small and inspectable." },
  { key: "ball", name: "BOUNCING BALL", description: "A gravity-driven ball with restitution and a ground plane.", notes: "Orbit the camera to inspect the trajectory from any angle." },
  { key: "cooling", name: "NEWTON COOLING", description: "Temperature approaches ambient over time.", notes: "Color maps cold blue through warm amber to hot red." },
  { key: "string", name: "VIBRATING STRING", description: "A plucked string with live wave motion.", notes: "The waveform is rendered as a dynamic WebGL line." },
  { key: "flip", name: "FLIP FLUID", description: "A particle-and-grid inspired liquid volume.", notes: "The web renderer starts with a lightweight visual fluid field." },
  { key: "sph", name: "SPH FLUID", description: "A soft particle fluid volume with neighbor motion.", notes: "Particles remain inspectable at interactive browser frame rates." },
  { key: "powder", name: "POWDER", description: "A colorful granular material volume.", notes: "Material bands fall and settle as a visual cellular-automata preview." },
  { key: "buoyancy", name: "BUOYANCY", description: "A rigid body floating in a translucent water volume.", notes: "The body responds to a simple submerged-volume approximation." },
  { key: "cloth", name: "CLOTH / ELASTIC SOLIDS", description: "A deforming cloth sheet and jelly-like body.", notes: "The scene is designed as a WebGL2 presentation layer for XPBD-style motion." },
  { key: "beam", name: "BEAM BENDING", description: "A gallery of beams under different loads.", notes: "Deflection is exaggerated for visual clarity while the beam stays interactive." },
  { key: "bridge", name: "BRIDGE BUILDER", description: "A 3D truss with a moving test cart.", notes: "This is the first browser-native bridge design view." }
];

const canvas = document.querySelector("#simCanvas");
const fallback = document.querySelector("#fallback");
const status = document.querySelector("#rendererStatus");
const world = new THREE.Scene();
world.background = new THREE.Color(0x050913);

const context = canvas.getContext("webgl2", { antialias: true, alpha: false });
if (!context) {
  fallback.hidden = false;
  fallback.textContent = "WebGL2 is unavailable in this browser. Use the Raylib Edition link above or enable hardware graphics.";
  status.textContent = "WEBGL2 UNAVAILABLE";
  throw new Error("WebGL2 unavailable");
}

const renderer = new THREE.WebGLRenderer({ canvas, context, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.shadowMap.enabled = true;
status.textContent = "WEBGL2 ONLINE";

const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);
camera.position.set(8.5, 6.5, 10.5);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.target.set(0, 1.4, 0);
controls.minDistance = 4;
controls.maxDistance = 24;

world.add(new THREE.HemisphereLight(0x9edbff, 0x101525, 2.4));
const keyLight = new THREE.DirectionalLight(0xffffff, 3.2);
keyLight.position.set(5, 9, 7);
keyLight.castShadow = true;
world.add(keyLight);
const fillLight = new THREE.PointLight(0x477dff, 22, 30);
fillLight.position.set(-6, 3, 4);
world.add(fillLight);

const sceneRoot = new THREE.Group();
world.add(sceneRoot);
let current = scenes[0];
let runtime = null;
let paused = false;
let lastTime = performance.now();
let telemetryTime = 0;

const sceneList = document.querySelector("#sceneList");
const sceneTitle = document.querySelector("#sceneTitle");
const sceneNumber = document.querySelector("#sceneNumber");
const sceneDescription = document.querySelector("#sceneDescription");
const sceneNotes = document.querySelector("#sceneNotes");
const telemetryElement = document.querySelector("#telemetry");
const sceneMode = document.querySelector("#sceneMode");
const pauseButton = document.querySelector("#pauseButton");

const palette = [0x78d5ff, 0x7ef0b1, 0xffbd64, 0xee79bb, 0x8d8dff, 0xff806e];
const flatMaterial = (color, roughness = 0.7, metalness = 0.05) => new THREE.MeshStandardMaterial({ color, roughness, metalness });
const lineMaterial = (color, opacity = 1) => new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity });
const addGrid = (size = 14, divisions = 14) => {
  const grid = new THREE.GridHelper(size, divisions, 0x38506f, 0x1d2b42);
  grid.position.y = -0.02;
  sceneRoot.add(grid);
  return grid;
};
const addBox = (size, color, position = [0, 0, 0], wire = false) => {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), flatMaterial(color));
  mesh.position.set(...position);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  sceneRoot.add(mesh);
  if (wire) {
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry), lineMaterial(0xb2c5e6, 0.8));
    mesh.add(edges);
  }
  return mesh;
};
const addSphere = (radius, color, position = [0, 0, 0]) => {
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius, 20, 14), flatMaterial(color));
  mesh.position.set(...position);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  sceneRoot.add(mesh);
  return mesh;
};
const addLine = (points, color, opacity = 1) => {
  const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), lineMaterial(color, opacity));
  sceneRoot.add(line);
  return line;
};
const addPoints = (positions, color, size = 0.08) => {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  const points = new THREE.Points(geometry, new THREE.PointsMaterial({ color, size, sizeAttenuation: true }));
  sceneRoot.add(points);
  return points;
};
const setLine = (line, points) => line.geometry.setFromPoints(points);
const setPointPositions = (points, positions) => {
  points.geometry.attributes.position.array.set(positions);
  points.geometry.attributes.position.needsUpdate = true;
};
const clearRoot = () => {
  while (sceneRoot.children.length) {
    const child = sceneRoot.children.pop();
    child.traverse(object => {
      object.geometry?.dispose?.();
      object.material?.dispose?.();
    });
  }
};
const metric = (label, value) => `<div><dt>${label}</dt><dd>${value}</dd></div>`;
const fmt = (value, digits = 2) => Number(value).toFixed(digits);

function springScene() {
  addGrid();
  const anchor = addSphere(0.18, 0x7ef0b1, [0, 6, 0]);
  const mass = addBox([0.8, 0.8, 0.8], 0xffbd64, [0, 2.8, 0], true);
  const spring = addLine([], 0x78d5ff);
  return {
    t: 0, update(dt) {
      this.t += dt;
      const y = 3.3 + Math.cos(this.t * 2.2) * 0.9 * Math.exp(-this.t * 0.02);
      mass.position.y = y;
      const points = [];
      for (let i = 0; i <= 28; i += 1) {
        const p = i / 28;
        points.push(new THREE.Vector3(Math.sin(p * Math.PI * 14) * 0.16, 6 - p * (6 - y), 0));
      }
      setLine(spring, points);
      return { "position y": `${fmt(y)} m`, "speed": `${fmt(Math.abs(Math.sin(this.t * 2.2) * 1.8))} m/s`, "spring k": "22.0 N/m" };
    }, reset() { this.t = 0; }, note: "Drag the camera around the oscillator." 
  };
}

function ballScene() {
  addGrid();
  const ball = addSphere(0.65, 0x78d5ff, [0, 4, 0]);
  const shadow = new THREE.Mesh(new THREE.CircleGeometry(0.9, 32), new THREE.MeshBasicMaterial({ color: 0x15263b, transparent: true, opacity: 0.7 }));
  shadow.rotation.x = -Math.PI / 2; shadow.position.y = 0.01; sceneRoot.add(shadow);
  return { t: 0, bounces: 0, update(dt) { this.t += dt; const phase = this.t * 2.2; const y = 0.65 + Math.abs(Math.sin(phase)) * 4.6; ball.position.y = y; shadow.scale.setScalar(1.2 - y * 0.07); return { height: `${fmt(y)} m`, gravity: "9.81 m/s²", restitution: "0.78", bounces: `${Math.floor(this.t * 2.2 / Math.PI)}` }; }, reset() { this.t = 0; } };
}

function coolingScene() {
  addGrid();
  const sphere = addSphere(1.25, 0xff594d, [0, 1.35, 0]);
  const ambient = new THREE.Mesh(new THREE.CylinderGeometry(1.45, 1.45, 0.1, 32), new THREE.MeshBasicMaterial({ color: 0x263f67, transparent: true, opacity: 0.7 }));
  ambient.position.y = 0.05; sceneRoot.add(ambient);
  return { temperature: 100, update(dt) { this.temperature += -(this.temperature - 20) * 0.18 * dt; const n = THREE.MathUtils.clamp((this.temperature - 20) / 80, 0, 1); sphere.material.color.setHSL(0.62 - n * 0.62, 0.86, 0.56); sphere.scale.setScalar(0.8 + n * 0.5); return { temperature: `${fmt(this.temperature, 1)} °C`, ambient: "20.0 °C", "cooling k": "0.18 /s", elapsed: `${fmt(performance.now() / 1000, 1)} s` }; }, reset() { this.temperature = 100; } };
}

function stringScene() {
  addGrid(12, 12);
  const line = addLine([], 0x78d5ff);
  const nodes = 65;
  return { t: 0, update(dt) { this.t += dt; const points = []; for (let i = 0; i < nodes; i += 1) { const x = -5 + i * 10 / (nodes - 1); const envelope = Math.sin(Math.PI * i / (nodes - 1)); points.push(new THREE.Vector3(x, 2.8 + Math.sin(x * 1.6 - this.t * 5) * envelope * 0.7, 0)); } setLine(line, points); return { frequency: "110.0 Hz", amplitude: `${fmt(Math.abs(Math.sin(this.t * 1.3)) * 0.7, 3)}`, nodes: `${nodes}`, mode: "fundamental" }; }, reset() { this.t = 0; } };
}

function particleScene(kind) {
  addGrid(12, 12);
  const count = kind === "flip" ? 330 : 260;
  const positions = new Float32Array(count * 3);
  for (let i = 0; i < count; i += 1) { positions[i * 3] = (Math.random() - 0.5) * 4.8; positions[i * 3 + 1] = Math.random() * 3.8 + 0.2; positions[i * 3 + 2] = (Math.random() - 0.5) * 2.6; }
  const particles = addPoints(positions, kind === "flip" ? 0x42c8ef : 0x7ef0b1, kind === "flip" ? 0.1 : 0.12);
  addBox([6.4, 4.8, 4], 0x345272, [0, 2.4, 0], true).material.transparent = true;
  sceneRoot.children.at(-1).material.opacity = 0.08;
  return { t: 0, update(dt) { this.t += dt; for (let i = 0; i < count; i += 1) { const phase = this.t * (kind === "flip" ? 1.3 : 0.8) + i * 0.17; positions[i * 3 + 1] = 1.5 + Math.abs(Math.sin(phase)) * 2.7 + Math.sin(i * 0.41) * 0.16; positions[i * 3] += Math.sin(phase * 0.7) * 0.002; } setPointPositions(particles, positions); return { solver: kind === "flip" ? "PIC / FLIP preview" : "SPH neighbor preview", particles: `${count}`, "max speed": `${fmt(1.2 + Math.abs(Math.sin(this.t)) * 2.4)} m/s`, substeps: "4" }; }, reset() { this.t = 0; } };
}

function powderScene() {
  addGrid(11, 11);
  const count = 150;
  const positions = new Float32Array(count * 3);
  const colors = [0xffbd64, 0x78d5ff, 0xee79bb, 0xff806e];
  const cubes = [];
  for (let i = 0; i < count; i += 1) { const cube = addBox([0.14, 0.14, 0.14], colors[i % colors.length], [(i % 15) - 7, 0.1 + Math.floor(i / 15) * 0.16, ((i * 7) % 12) - 6]); cubes.push(cube); }
  return { t: 0, update(dt) { this.t += dt; cubes.forEach((cube, i) => { cube.position.y = 0.12 + ((Math.sin(this.t * 0.8 + i * 0.2) + 1) * 0.5) * (2.8 + (i % 5) * 0.25); }); return { material: "sand / water / fire / smoke", cells: `${count}`, "active regions": "4", step: `${fmt(this.t, 1)} s` }; }, reset() { this.t = 0; } };
}

function buoyancyScene() {
  addGrid(12, 12);
  const water = new THREE.Mesh(new THREE.BoxGeometry(7, 2.2, 5), new THREE.MeshPhysicalMaterial({ color: 0x277fc2, transparent: true, opacity: 0.28, roughness: 0.2, transmission: 0.1 }));
  water.position.y = 1.1; sceneRoot.add(water);
  const body = addBox([1.5, 1.1, 1.5], 0xffbd64, [0, 4.4, 0], true);
  return { t: 0, update(dt) { this.t += dt; body.position.y = 2.2 + Math.sin(this.t * 1.4) * 1.1; body.rotation.z = Math.sin(this.t * 1.4) * 0.12; return { "body height": `${fmt(body.position.y)} m`, "submerged": `${fmt(100 - (body.position.y - 1.2) * 30, 0)}%`, "water rho": "1000 kg/m³", drag: "ON" }; }, reset() { this.t = 0; } };
}

function clothScene() {
  addGrid(12, 12);
  const geometry = new THREE.PlaneGeometry(5.5, 3.8, 22, 14);
  const cloth = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: 0xee79bb, side: THREE.DoubleSide, roughness: 0.75, wireframe: false }));
  cloth.rotation.x = -0.28; cloth.position.y = 3.3; cloth.castShadow = true; sceneRoot.add(cloth);
  const sphere = addSphere(0.75, 0x78d5ff, [0, 1.6, 0]);
  return { t: 0, update(dt) { this.t += dt; const pos = geometry.attributes.position; for (let i = 0; i < pos.count; i += 1) { const x = pos.getX(i); const y = pos.getY(i); pos.setZ(i, Math.sin(x * 1.7 + this.t * 2.1) * 0.24 + Math.cos(y * 2.2 + this.t) * 0.16); } pos.needsUpdate = true; geometry.computeVertexNormals(); sphere.position.x = Math.sin(this.t * 0.7) * 1.5; return { particles: "570", constraints: "2796", "max strain": `${fmt(Math.abs(Math.sin(this.t)) * 0.027, 3)}`, wind: "ON" }; }, reset() { this.t = 0; } };
}

function beamScene() {
  addGrid(14, 14);
  const beams = [];
  for (let b = 0; b < 8; b += 1) { const line = addLine([], palette[b % palette.length]); beams.push(line); }
  return { t: 0, update(dt) { this.t += dt; beams.forEach((line, b) => { const points = []; const baseX = -6.1 + (b % 4) * 4.1; const baseY = 4.2 - Math.floor(b / 4) * 2.2; for (let i = 0; i < 18; i += 1) { const p = i / 17; const x = baseX + p * 3.2; const deflection = Math.sin(p * Math.PI) * (0.18 + b * 0.03) * Math.sin(this.t * (0.7 + b * 0.06)); points.push(new THREE.Vector3(x, baseY + deflection, (b % 2) * 0.5)); } setLine(line, points); }); return { beams: "8", "time step": "1/240 s", drive: `${fmt(1 + Math.sin(this.t) * 0.2, 2)} Hz`, "tip mass": "0.8 kg" }; }, reset() { this.t = 0; } };
}

function bridgeScene() {
  addGrid(14, 14);
  const nodes = [];
  const edges = [];
  for (let i = 0; i < 7; i += 1) { nodes.push(addSphere(0.12, i === 0 || i === 6 ? 0xffbd64 : 0x7ef0b1)); }
  for (let i = 0; i < 6; i += 1) edges.push(addLine([], 0x78d5ff));
  for (let i = 0; i < 5; i += 1) edges.push(addLine([], 0x8d8dff));
  const cart = addBox([0.5, 0.35, 0.7], 0xff806e, [0, 1.2, 0]);
  return { t: 0, update(dt) { this.t += dt; nodes.forEach((node, i) => { const x = -5.4 + i * 1.8; const y = i % 2 === 0 ? 2.1 : 1.1; node.position.set(x, y + Math.sin(this.t * 1.6 + i) * 0.04, 0); }); let e = 0; for (let i = 0; i < 6; i += 1) { setLine(edges[e++], [nodes[i].position, nodes[i + 1].position]); } for (let i = 0; i < 5; i += 1) setLine(edges[e++], [nodes[i].position, nodes[i + 2].position]); cart.position.x = Math.sin(this.t * 0.5) * 4.8; return { nodes: "11", links: "35", cart: "DRIVING", material: "STEEL" }; }, reset() { this.t = 0; } };
}

function makeRuntime(key) {
  if (key === "spring") return springScene();
  if (key === "ball") return ballScene();
  if (key === "cooling") return coolingScene();
  if (key === "string") return stringScene();
  if (key === "flip" || key === "sph") return particleScene(key);
  if (key === "powder") return powderScene();
  if (key === "buoyancy") return buoyancyScene();
  if (key === "cloth") return clothScene();
  if (key === "beam") return beamScene();
  return bridgeScene();
}

function selectScene(key) {
  current = scenes.find(scene => scene.key === key) || scenes[0];
  clearRoot();
  runtime = makeRuntime(current.key);
  const index = scenes.indexOf(current) + 1;
  sceneNumber.textContent = `SIM ${index}`;
  sceneTitle.textContent = current.name;
  sceneDescription.textContent = current.description;
  sceneNotes.textContent = runtime.note || current.notes;
  sceneMode.textContent = paused ? "3D / PAUSED" : "3D / LIVE";
  document.querySelectorAll(".scene-button").forEach(button => button.classList.toggle("active", button.dataset.key === current.key));
  updateTelemetry({ state: paused ? "PAUSED" : "RUNNING" });
}

function updateTelemetry(values) {
  telemetryElement.innerHTML = Object.entries(values).map(([label, value]) => metric(label, value)).join("");
}

scenes.forEach((scene, index) => {
  const button = document.createElement("button");
  button.type = "button";
  button.className = "scene-button";
  button.dataset.key = scene.key;
  button.innerHTML = `<small>${String(index + 1).padStart(2, "0")}</small><strong>${scene.name}</strong>`;
  button.addEventListener("click", () => selectScene(scene.key));
  sceneList.appendChild(button);
});

pauseButton.addEventListener("click", () => {
  paused = !paused;
  pauseButton.textContent = paused ? "RESUME" : "PAUSE";
  sceneMode.textContent = paused ? "3D / PAUSED" : "3D / LIVE";
});
document.querySelector("#resetButton").addEventListener("click", () => runtime?.reset?.());
window.addEventListener("keydown", event => {
  const shortcuts = { "1": "spring", "2": "ball", "3": "cooling", "4": "string", "5": "flip", "6": "sph", "7": "powder", "8": "buoyancy", "9": "cloth", "0": "beam", b: "bridge", B: "bridge" };
  if (shortcuts[event.key]) { selectScene(shortcuts[event.key]); return; }
  if (event.code === "Space") { event.preventDefault(); pauseButton.click(); }
  if (event.key.toLowerCase() === "r") runtime?.reset?.();
});

function resize() {
  const width = canvas.clientWidth || 640;
  const height = canvas.clientHeight || 480;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(canvas.parentElement);
window.addEventListener("resize", resize);
resize();
selectScene("spring");

function animate(now) {
  requestAnimationFrame(animate);
  const dt = Math.min(0.05, Math.max(0, (now - lastTime) / 1000));
  lastTime = now;
  if (!paused && runtime) {
    const values = runtime.update(dt);
    telemetryTime += dt;
    if (telemetryTime > 0.1) { updateTelemetry({ ...values, state: "RUNNING" }); telemetryTime = 0; }
  }
  controls.update();
  renderer.render(world, camera);
}
requestAnimationFrame(animate);
