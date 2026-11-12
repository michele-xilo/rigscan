import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';

const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

// Pose of a joint frame in its parent. URDF rpy is fixed-axis roll, pitch, yaw: R = Rz * Ry * Rx.
function local(origin) {
  const [x, y, z] = origin.xyz;
  const [r, p, w] = origin.rpy;
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(r, p, w, 'ZYX'));
  return new THREE.Matrix4().compose(new THREE.Vector3(x, y, z), q, new THREE.Vector3(1, 1, 1));
}

// Cylinders and capsules are along Z in URDF and MJCF, along Y in three.js.
function primitive(s) {
  switch (s.type) {
    case 'box': return new THREE.BoxGeometry(...s.size);
    case 'sphere': return new THREE.SphereGeometry(s.radius, 24, 16);
    case 'cylinder': return new THREE.CylinderGeometry(s.radius, s.radius, s.length, 24).rotateX(Math.PI / 2);
    case 'capsule': return new THREE.CapsuleGeometry(s.radius, s.length, 8, 16).rotateX(Math.PI / 2);
    default: return null;
  }
}

const text = (bytes) => new TextDecoder().decode(bytes);
const wait = () => new Promise((done) => setTimeout(done));

// The geometries of an STL or OBJ file, or null for a format that is not drawn.
function parseMesh(path, bytes) {
  if (/\.stl$/i.test(path)) {
    const g = new STLLoader().parse(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
    return [g];
  }
  if (/\.obj$/i.test(path)) {
    const out = [];
    new OBJLoader().parse(text(bytes)).traverse((o) => {
      if (!o.isMesh) return;
      if (!o.geometry.attributes.normal) o.geometry.computeVertexNormals();
      out.push(o.geometry);
    });
    return out;
  }
  return null;
}

// Link frames in the world, by link name. A parent that is never a child is a root at the origin.
function frames(result) {
  const byParent = new Map();
  const children = new Set();
  for (const j of result.joints) {
    if (!byParent.has(j.parent)) byParent.set(j.parent, []);
    byParent.get(j.parent).push(j);
    children.add(j.child);
  }
  const world = new Map();
  const queue = [];
  const names = [...result.links.map((l) => l.name), ...byParent.keys()];
  for (const n of names) {
    if (!children.has(n) && !world.has(n)) {
      world.set(n, new THREE.Matrix4());
      queue.push(n);
    }
  }
  while (queue.length) {
    const parent = queue.shift();
    for (const j of byParent.get(parent) ?? []) {
      if (world.has(j.child)) continue;
      world.set(j.child, world.get(parent).clone().multiply(local(j.origin)));
      queue.push(j.child);
    }
  }
  return world;
}

export function createRig(canvas, onPick, onMeshes) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.001, 1000);
  camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, canvas);
  const ray = new THREE.Raycaster();
  scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 2.8));
  const sun = new THREE.DirectionalLight(0xffffff, 1.8);
  sun.position.set(1, -1, 2);
  scene.add(sun);
  const ring = new THREE.Mesh(new THREE.SphereGeometry(1, 12, 8), new THREE.MeshBasicMaterial({ wireframe: true }));
  ring.visible = false;
  scene.add(ring);

  let content = null;
  let pickable = [];
  let anchors = new Map(); // "link name" or "joint name" -> world position
  let radius = 1;
  let linkMaterials = new Map(); // link name -> { material, base }
  let touched = false;
  let token = 0;
  let accent = null;

  const render = () => renderer.render(scene, camera);
  controls.addEventListener('change', render);
  controls.addEventListener('start', () => { touched = true; });

  function resize() {
    const { clientWidth: w, clientHeight: h } = canvas;
    if (!w || !h) return;
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    render();
  }
  new ResizeObserver(resize).observe(canvas);

  function fit() {
    const box = new THREE.Box3().setFromObject(content);
    if (box.isEmpty()) return;
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 0.1);
    const centre = box.getCenter(new THREE.Vector3());
    camera.near = size / 100;
    camera.far = size * 100;
    camera.updateProjectionMatrix();
    camera.position.copy(centre).addScaledVector(new THREE.Vector3(1, -1.2, 0.8).normalize(), size * 1.6);
    controls.target.copy(centre);
    controls.update();
  }

  function clear() {
    token++;
    if (!content) return;
    scene.remove(content);
    content.traverse((o) => {
      o.geometry?.dispose();
      o.material?.dispose();
    });
    content = null;
    pickable = [];
    anchors = new Map();
    linkMaterials = new Map();
  }

  // One mesh per task, so a large robot does not freeze the page, drawn as each one arrives.
  async function loadMeshes(jobs, files, mine) {
    const count = { drawn: 0, missing: 0, unsupported: 0, failed: 0 };
    const cache = new Map();
    for (const job of jobs) {
      await wait();
      if (mine !== token) return;
      const path = job.shape.resolved;
      if (!path) { count.missing++; continue; }
      try {
        if (!cache.has(path)) cache.set(path, parseMesh(path, files.get(path) ?? new Uint8Array(0)));
        const geometries = cache.get(path);
        if (!geometries) { count.unsupported++; continue; }
        for (const g of geometries) {
          const mesh = new THREE.Mesh(g, job.material);
          mesh.userData = job.holder.userData;
          job.holder.add(mesh);
        }
        count.drawn++;
        render();
      } catch {
        count.failed++;
      }
    }
    if (mine !== token) return;
    if (!touched) fit();
    render();
    onMeshes?.(count);
  }

  function show(result, files) {
    touched = false;
    clear();
    const mine = token;
    accent = css('--accent');
    const colour = { healthy: css('--healthy'), warning: css('--warning'), critical: css('--critical') };
    const world = frames(result);
    const pos = new Map([...world].map(([n, m]) => [n, new THREE.Vector3().setFromMatrixPosition(m)]));
    const box = new THREE.Box3().setFromPoints([...pos.values()]);
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 0.1);
    radius = size * 0.025;
    content = new THREE.Group();
    const skeleton = new THREE.Group();
    content.add(skeleton);

    const linkStatus = new Map(result.links.map((l) => [l.name, l.status]));
    for (const [name, p] of pos) {
      const status = linkStatus.get(name) ?? 'critical';
      const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(status === 'healthy' ? radius : radius * 1.5, 16, 12),
        new THREE.MeshBasicMaterial({ color: colour[status], transparent: status === 'healthy', opacity: 0.6 }),
      );
      sphere.position.copy(p);
      sphere.userData = { kind: 'link', name };
      skeleton.add(sphere);
      pickable.push(sphere);
      anchors.set(`link ${name}`, p);
      const triad = new THREE.AxesHelper(size * 0.06);
      triad.applyMatrix4(world.get(name));
      skeleton.add(triad);
    }

    for (const j of result.joints) {
      const from = pos.get(j.parent);
      const to = pos.get(j.child);
      if (!from || !to) continue;
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([from, to]),
        new THREE.LineBasicMaterial({ color: colour[j.status], transparent: j.status === 'healthy', opacity: 0.6 }),
      );
      line.userData = { kind: 'joint', name: j.name };
      skeleton.add(line);
      pickable.push(line);
      anchors.set(`joint ${j.name}`, from.clone().add(to).multiplyScalar(0.5));
      const axis = j.axis && new THREE.Vector3(...j.axis);
      if (axis && axis.length() > 0) {
        const q = new THREE.Quaternion().setFromRotationMatrix(world.get(j.child));
        axis.normalize().applyQuaternion(q);
        skeleton.add(new THREE.ArrowHelper(axis, to, size * 0.12, colour[j.status], radius * 2, radius));
      }
    }

    const jobs = [];
    for (const l of result.links) {
      const frame = world.get(l.name);
      const shapes = l.shapes.filter((sh) => sh.role !== 'collision');
      if (!frame || !shapes.length) continue;
      const status = l.status;
      const base = status === 'healthy' ? css('--muted') : colour[status];
      const material = new THREE.MeshLambertMaterial({ color: base, side: THREE.DoubleSide });
      linkMaterials.set(l.name, { material, base });
      for (const sh of shapes) {
        const holder = new THREE.Group();
        holder.matrixAutoUpdate = false;
        holder.matrix.copy(frame).multiply(local(sh.origin));
        holder.userData = { kind: 'link', name: l.name };
        const geometry = primitive(sh);
        if (geometry) {
          const mesh = new THREE.Mesh(geometry, material);
          mesh.userData = holder.userData;
          holder.add(mesh);
        } else if (sh.type === 'mesh') {
          holder.matrix.scale(new THREE.Vector3(...sh.scale));
          jobs.push({ holder, shape: sh, material });
        }
        content.add(holder);
        pickable.push(holder);
      }
    }
    // The skeleton is drawn over the geometry, so a joint inside a mesh can still be seen.
    skeleton.traverse((o) => {
      o.renderOrder = 1;
      if (o.material) o.material.depthTest = false;
    });
    scene.add(content);
    fit();
    ring.material.color.set(accent);
    highlight(null);
    resize();
    if (jobs.length) loadMeshes(jobs, files, mine);
    else onMeshes?.({ drawn: 0, missing: 0, unsupported: 0, failed: 0 });
  }

  function highlight(sel) {
    for (const [name, m] of linkMaterials) {
      m.material.color.set(sel?.kind === 'link' && sel.name === name ? accent : m.base);
    }
    const p = sel && anchors.get(`${sel.kind} ${sel.name}`);
    ring.visible = !!p;
    if (p) {
      ring.position.copy(p);
      ring.scale.setScalar(radius * 2.6);
    }
    render();
  }

  let down = null;
  canvas.addEventListener('pointerdown', (e) => { down = [e.clientX, e.clientY]; });
  canvas.addEventListener('pointerup', (e) => {
    if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) return;
    const r = canvas.getBoundingClientRect();
    ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
    ray.params.Line.threshold = radius;
    const hit = ray.intersectObjects(pickable, true)[0];
    onPick(hit ? hit.object.userData : null);
  });

  return { show, highlight };
}
