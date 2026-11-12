import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

// Pose of a joint frame in its parent. URDF rpy is fixed-axis roll, pitch, yaw: R = Rz * Ry * Rx.
function local(origin) {
  const [x, y, z] = origin.xyz;
  const [r, p, w] = origin.rpy;
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(r, p, w, 'ZYX'));
  return new THREE.Matrix4().compose(new THREE.Vector3(x, y, z), q, new THREE.Vector3(1, 1, 1));
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

export function createRig(canvas, onPick) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.001, 1000);
  camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, canvas);
  const ray = new THREE.Raycaster();
  const ring = new THREE.Mesh(new THREE.SphereGeometry(1, 12, 8), new THREE.MeshBasicMaterial({ wireframe: true }));
  ring.visible = false;
  scene.add(ring);

  let content = null;
  let pickable = [];
  let anchors = new Map(); // "link name" or "joint name" -> world position
  let radius = 1;
  let last = null;

  const render = () => renderer.render(scene, camera);
  controls.addEventListener('change', render);

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

  function clear() {
    if (!content) return;
    scene.remove(content);
    content.traverse((o) => {
      o.geometry?.dispose();
      o.material?.dispose();
    });
    content = null;
    pickable = [];
    anchors = new Map();
  }

  function show(result) {
    last = result;
    clear();
    const colour = { healthy: css('--healthy'), warning: css('--warning'), critical: css('--critical') };
    const world = frames(result);
    const pos = new Map([...world].map(([n, m]) => [n, new THREE.Vector3().setFromMatrixPosition(m)]));
    const box = new THREE.Box3().setFromPoints([...pos.values()]);
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 0.1);
    radius = size * 0.025;
    content = new THREE.Group();

    const linkStatus = new Map(result.links.map((l) => [l.name, l.status]));
    for (const [name, p] of pos) {
      const status = linkStatus.get(name) ?? 'critical';
      const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(status === 'healthy' ? radius : radius * 1.5, 16, 12),
        new THREE.MeshBasicMaterial({ color: colour[status], transparent: status === 'healthy', opacity: 0.6 }),
      );
      sphere.position.copy(p);
      sphere.userData = { kind: 'link', name };
      content.add(sphere);
      pickable.push(sphere);
      anchors.set(`link ${name}`, p);
      const triad = new THREE.AxesHelper(size * 0.06);
      triad.applyMatrix4(world.get(name));
      content.add(triad);
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
      content.add(line);
      pickable.push(line);
      anchors.set(`joint ${j.name}`, from.clone().add(to).multiplyScalar(0.5));
      const axis = j.axis && new THREE.Vector3(...j.axis);
      if (axis && axis.length() > 0) {
        const q = new THREE.Quaternion().setFromRotationMatrix(world.get(j.child));
        axis.normalize().applyQuaternion(q);
        content.add(new THREE.ArrowHelper(axis, to, size * 0.12, colour[j.status], radius * 2, radius));
      }
    }
    scene.add(content);

    const centre = box.isEmpty() ? new THREE.Vector3() : box.getCenter(new THREE.Vector3());
    camera.near = size / 100;
    camera.far = size * 100;
    camera.updateProjectionMatrix();
    camera.position.copy(centre).addScaledVector(new THREE.Vector3(1, -1.2, 0.8).normalize(), size * 1.6);
    controls.target.copy(centre);
    controls.update();
    ring.material.color.set(css('--accent'));
    highlight(null);
    resize();
  }

  function highlight(sel) {
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
    const hit = ray.intersectObjects(pickable)[0];
    onPick(hit ? hit.object.userData : null);
  });

  // Colours come from the page theme, so a light/dark switch redraws.
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => last && show(last));

  return { show, highlight };
}
