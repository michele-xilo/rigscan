import init, { scan_files } from './pkg/rigscan_wasm.js';

const $ = (id) => document.getElementById(id);
const READ = /\.(urdf|xml|usd|usda|usdc)$/i;
const WORD = { healthy: 'healthy', warning: 'warning', critical: 'critical' };

let files = new Map(); // path -> Uint8Array
let result = null;
let ready = false;

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

function setStatus(text, error = false) {
  const s = $('status');
  s.textContent = text;
  s.classList.toggle('error', error);
}

async function start() {
  try {
    await init();
    ready = true;
    setStatus('Ready. Open a robot to scan it.');
  } catch (e) {
    const hint = location.protocol === 'file:' ? ' Serve the web folder over http, for example: python3 -m http.server -d web' : '';
    setStatus(`Could not load the WebAssembly module: ${e}.${hint}`, true);
  }
}

// ---- reading files ----------------------------------------------------------

async function fromList(list) {
  const out = new Map();
  for (const f of list) {
    const path = f.webkitRelativePath || f.name;
    if (READ.test(path)) out.set(path, new Uint8Array(await f.arrayBuffer()));
  }
  return out;
}

// Entries must be taken from the event before the first await.
async function fromDrop(dt) {
  const entries = [...dt.items].map((i) => i.webkitGetAsEntry?.()).filter(Boolean);
  if (!entries.length) return fromList(dt.files);
  const out = new Map();
  const walk = async (entry) => {
    if (entry.isFile) {
      const path = entry.fullPath.replace(/^\//, '');
      if (!READ.test(path)) return;
      const file = await new Promise((ok, fail) => entry.file(ok, fail));
      out.set(path, new Uint8Array(await file.arrayBuffer()));
      return;
    }
    const reader = entry.createReader();
    for (;;) {
      const batch = await new Promise((ok, fail) => reader.readEntries(ok, fail));
      if (!batch.length) break;
      for (const e of batch) await walk(e);
    }
  };
  for (const e of entries) await walk(e);
  return out;
}

function isMjcf(bytes) {
  return new TextDecoder().decode(bytes.subarray(0, 16384)).includes('<mujoco');
}

function candidates() {
  return [...files.keys()].filter((p) => !/\.xml$/i.test(p) || isMjcf(files.get(p)));
}

function load(map) {
  files = map;
  const list = candidates();
  if (!list.length) {
    $('result').hidden = true;
    $('main-pick').hidden = true;
    setStatus('No URDF, MJCF or USD file found in what was opened.', true);
    return;
  }
  const rank = (p) => (/\.urdf$/i.test(p) ? 0 : /\.xml$/i.test(p) ? 1 : 2);
  list.sort((a, b) => rank(a) - rank(b) || a.length - b.length || a.localeCompare(b));
  const select = $('main');
  select.replaceChildren(...list.map((p) => Object.assign(el('option', '', p), { value: p })));
  $('main-pick').hidden = list.length < 2;
  run(list[0]);
}

// ---- scanning and drawing ---------------------------------------------------

function run(main) {
  if (!ready) return setStatus('Rigscan is still loading.', true);
  const paths = [...files.keys()];
  const started = performance.now();
  const out = JSON.parse(scan_files(main, paths, paths.map((p) => files.get(p))));
  const ms = performance.now() - started;
  if (out.error) {
    result = null;
    $('result').hidden = true;
    return setStatus(out.error, true);
  }
  result = out;
  setStatus(`Scanned ${main} in ${ms < 10 ? ms.toFixed(1) : Math.round(ms)} ms.`);
  draw();
}

function pill(status, n) {
  return el('span', `pill ${status}`, `${n} ${WORD[status]}`);
}

function draw() {
  const r = result;
  $('result').hidden = false;
  const summary = $('summary');
  summary.replaceChildren(
    el('strong', '', `${r.robot ?? r.path} `),
    el('span', '', `(${r.format.toUpperCase()}) `),
    pill('critical', r.verdict.critical), document.createTextNode(' '),
    pill('warning', r.verdict.warnings), document.createTextNode(' '),
    pill('healthy', r.verdict.healthy),
  );
  drawTree();
  drawFindings();
  drawUnmapped();
  drawSource();
  select(null);
}

function badge(status) {
  return el('span', `badge ${status}`, WORD[status]);
}

function node(kind, name, status, detail) {
  const b = el('button', 'node');
  b.type = 'button';
  b.dataset.kind = kind;
  b.dataset.name = name;
  b.setAttribute('role', 'treeitem');
  b.setAttribute('aria-selected', 'false');
  b.append(el('span', 'kind', detail ? `${kind} (${detail})` : kind), el('span', 'name', name), badge(status));
  b.addEventListener('click', () => select({ kind, name }, 'tree'));
  return b;
}

function drawTree() {
  const r = result;
  const links = new Map(r.links.map((l) => [l.name, l]));
  const byParent = new Map();
  const children = new Set();
  for (const j of r.joints) {
    if (!byParent.has(j.parent)) byParent.set(j.parent, []);
    byParent.get(j.parent).push(j);
    children.add(j.child);
  }
  const seen = new Set();
  const jointItem = (j) => {
    const li = el('li');
    li.append(node('joint', j.name, j.status, j.kind));
    const sub = el('ul');
    sub.setAttribute('role', 'group');
    sub.append(linkItem(j.child));
    li.append(sub);
    return li;
  };
  const linkItem = (name) => {
    const li = el('li');
    const link = links.get(name);
    if (!link) {
      li.className = 'missing';
      li.append(el('span', 'node', ''));
      li.firstChild.append(el('span', 'kind', 'link'), el('span', 'name', name), el('span', 'badge critical', 'not defined'));
      return li;
    }
    if (seen.has(name)) {
      li.append(el('span', 'node', ''));
      li.firstChild.append(el('span', 'kind', 'link'), el('span', 'name', name), el('span', 'kind', 'shown above'));
      return li;
    }
    seen.add(name);
    li.append(node('link', name, link.status));
    const joints = byParent.get(name) ?? [];
    if (joints.length) {
      const sub = el('ul');
      sub.setAttribute('role', 'group');
      sub.append(...joints.map(jointItem));
      li.append(sub);
    }
    return li;
  };
  const tree = $('tree');
  const roots = [];
  for (const l of r.links) if (!children.has(l.name)) roots.push(linkItem(l.name));
  // A parent named by a joint but never defined (a world frame, or a typo) still roots its joints.
  for (const parent of byParent.keys()) if (!links.has(parent)) roots.push(linkItem(parent));
  for (const l of r.links) if (!seen.has(l.name)) roots.push(linkItem(l.name));
  tree.replaceChildren(...roots);
}

function where(f) {
  return f.line == null ? 'no position' : `line ${f.line}:${f.column}`;
}

function drawFindings() {
  const list = $('findings');
  list.replaceChildren(...result.findings.map((f, i) => {
    const li = el('li');
    const b = el('button', `finding ${f.severity}`);
    b.type = 'button';
    b.dataset.index = i;
    const head = el('div', 'head');
    head.append(badge(f.severity), el('span', 'rule', f.rule), el('span', 'where', where(f)));
    b.append(head, el('p', '', f.message), el('span', 'kind', `${f.element.kind} ${f.element.name ?? ''}`));
    b.addEventListener('click', () => select(f.element, 'finding', f));
    li.append(b);
    return li;
  }));
  $('no-findings').hidden = result.findings.length > 0;
}

function drawUnmapped() {
  const u = result.unmapped;
  $('unmapped-box').hidden = u.length === 0;
  $('unmapped-count').textContent = `(${u.length})`;
  $('unmapped').replaceChildren(...u.map((x) => el('li', '', `${x.what}${x.name ? ` ${x.name}` : ''}${x.line ? ` (line ${x.line})` : ''}`)));
}

function drawSource() {
  $('source-name').textContent = result.path;
  const box = $('source');
  if (!result.source) {
    box.replaceChildren(el('p', 'empty', 'USD has no source positions, so there is no source to show.'));
    return;
  }
  const worst = new Map();
  for (const f of result.findings) {
    if (f.line == null) continue;
    if (worst.get(f.line) !== 'critical') worst.set(f.line, f.severity);
  }
  const frag = document.createDocumentFragment();
  result.source.split('\n').forEach((text, i) => {
    const row = el('div', `row ${worst.get(i + 1) ?? ''}`.trim());
    row.id = `L${i + 1}`;
    row.append(el('span', 'num', String(i + 1)), el('span', 'text', text));
    frag.append(row);
  });
  box.replaceChildren(frag);
}

function lineOf(sel) {
  const pool = sel.kind === 'link' ? result.links : sel.kind === 'joint' ? result.joints : [];
  return pool.find((x) => x.name === sel.name)?.line ?? null;
}

function select(sel, from, finding) {
  for (const n of document.querySelectorAll('.node[data-kind]')) {
    n.setAttribute('aria-selected', String(!!sel && n.dataset.kind === sel.kind && n.dataset.name === sel.name));
  }
  for (const b of document.querySelectorAll('.finding')) {
    const f = result.findings[Number(b.dataset.index)];
    const match = !!sel && f.element.kind === sel.kind && f.element.name === sel.name;
    b.setAttribute('aria-current', String(from === 'finding' ? f === finding : match));
  }
  for (const r of document.querySelectorAll('#source .row.picked')) r.classList.remove('picked');
  const line = finding?.line ?? (sel ? lineOf(sel) : null);
  const row = line ? $(`L${line}`) : null;
  if (row) {
    row.classList.add('picked');
    const box = $('source');
    box.scrollTop = row.offsetTop - box.clientHeight / 3;
  }
}

// ---- wiring -----------------------------------------------------------------

$('pick-files').addEventListener('change', async (e) => load(await fromList(e.target.files)));
$('pick-folder').addEventListener('change', async (e) => load(await fromList(e.target.files)));
$('main').addEventListener('change', (e) => run(e.target.value));
$('sample').addEventListener('click', async () => {
  const res = await fetch('sample/broken_arm.urdf');
  load(new Map([['broken_arm.urdf', new Uint8Array(await res.arrayBuffer())]]));
});

const drop = $('drop');
document.addEventListener('dragover', (e) => { e.preventDefault(); drop.classList.add('over'); });
document.addEventListener('dragleave', (e) => { if (!e.relatedTarget) drop.classList.remove('over'); });
document.addEventListener('drop', async (e) => {
  e.preventDefault();
  drop.classList.remove('over');
  load(await fromDrop(e.dataTransfer));
});

start();
