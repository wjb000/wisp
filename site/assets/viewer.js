import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {RoomEnvironment} from 'three/addons/environments/RoomEnvironment.js';
import {toCreasedNormals} from 'three/addons/utils/BufferGeometryUtils.js';

const BUILD = 'v2';
const $ = id => document.getElementById(id);
const el = $('viewer'), Q = new URLSearchParams(location.search);
const scene = new THREE.Scene();
const cam = new THREE.PerspectiveCamera(28, el.clientWidth / el.clientHeight, 5, 8000);
const r = new THREE.WebGLRenderer({antialias: true, alpha: true, preserveDrawingBuffer: true});
r.setPixelRatio(Math.min(devicePixelRatio, 2)); r.setSize(el.clientWidth, el.clientHeight);
r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFSoftShadowMap;
r.toneMapping = THREE.ACESFilmicToneMapping; r.toneMappingExposure = 1.05; r.outputColorSpace = THREE.SRGBColorSpace;
el.prepend(r.domElement);
const pm = new THREE.PMREMGenerator(r); scene.environment = pm.fromScene(new RoomEnvironment(), 0.04).texture; scene.environmentIntensity = 0.55;
const ctl = new OrbitControls(cam, r.domElement); ctl.enableDamping = true; ctl.minDistance = 160; ctl.maxDistance = 2500;

// robot frame: x forward, y left, z up (mm)  ->  three: (x, z, -y)
const TGT = new THREE.Vector3(10, 100, 70);
const VIEWS = {front: [860, 160, 70], '34': [600, 360, -520], side: [10, 170, -880], back: [-860, 180, 70], top: [11, 980, 70]};
function setView(n) {
  const v = VIEWS[n] || VIEWS['34']; cam.position.set(...v); ctl.target.copy(TGT); ctl.update();
  document.querySelectorAll('[data-view]').forEach(b => b.classList.toggle('on', b.dataset.view === n));
}

scene.add(new THREE.HemisphereLight(0xfff6ea, 0x9a8f80, 0.9));
const key = new THREE.DirectionalLight(0xffffff, 2.1); key.position.set(380, 760, -420); key.castShadow = true;
key.shadow.mapSize.set(2048, 2048); Object.assign(key.shadow.camera, {left: -320, right: 320, top: 320, bottom: -320, near: 100, far: 2200});
key.shadow.bias = -0.0004; key.shadow.normalBias = 1.0; scene.add(key);
const rim = new THREE.DirectionalLight(0xffe0bf, 1.1); rim.position.set(-600, 380, 300); scene.add(rim);
const floor = new THREE.Mesh(new THREE.CircleGeometry(1200, 96), new THREE.ShadowMaterial({opacity: 0.16}));
floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
const root = new THREE.Group(); root.rotation.x = -Math.PI / 2; scene.add(root);

const GROUPS = {
  shell: {name: 'Shell', color: '#e9e1d3'}, structure: {name: 'Inner structure', color: '#2b2f36'},
  drivetrain: {name: 'Drivetrain', color: '#9aa1ab'}, power: {name: 'Power', color: '#3b6fb6'},
  electronics: {name: 'Electronics', color: '#2f8f4e'}, sensors: {name: 'Sensors', color: '#7a2fa0'},
  expression: {name: 'Face, light, sound, motion', color: '#ff8a3d'}, dock: {name: 'Charging dock', color: '#5b616b'}
};
const items = {}, links = {}, labels = [];
const shellMats = [];
let RI, xray = Q.get('xray') === '1', showLabels = Q.get('labels') !== '0', explodeT = +(Q.get('explode') || 0), demo = false, sel = null;
const visible = Object.fromEntries(Object.keys(GROUPS).map(g => [g, true]));
(Q.get('hide') || '').split(',').filter(Boolean).forEach(g => visible[g] = false);

function eyeTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 256; const g = c.getContext('2d');
  g.fillStyle = '#05080c'; g.fillRect(0, 0, 256, 256);
  const grd = g.createRadialGradient(128, 132, 10, 128, 132, 80); grd.addColorStop(0, '#fff6d8'); grd.addColorStop(0.35, '#ffc36b'); grd.addColorStop(0.8, '#ff8a3d'); grd.addColorStop(1, 'rgba(255,120,50,0)');
  g.fillStyle = grd; g.beginPath(); g.ellipse(128, 136, 58, 66, 0, 0, Math.PI * 2); g.fill();
  g.fillStyle = '#05080c'; g.beginPath(); g.ellipse(128, 140, 22, 26, 0, 0, Math.PI * 2); g.fill();
  g.fillStyle = 'rgba(255,255,255,.9)'; g.beginPath(); g.ellipse(108, 112, 10, 8, -0.5, 0, Math.PI * 2); g.fill();
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}

function makeMat(key, it) {
  const k = key.split(':')[1];
  const c = new THREE.Color(it.color);
  if (key.startsWith('part:')) {
    if (k === 'glow_ring') return new THREE.MeshPhysicalMaterial({color: 0xfff1dc, emissive: 0xffa24a, emissiveIntensity: 0.55, roughness: 0.35, transmission: 0.2, thickness: 2, transparent: true, opacity: 0.92});
    if (k === 'tail') return new THREE.MeshPhysicalMaterial({color: c, roughness: 0.55, sheen: 0.6, sheenColor: 0xffc0a0});
    if (k === 'dome' || k === 'mantle') { const m = new THREE.MeshPhysicalMaterial({color: c, roughness: 0.32, clearcoat: 0.6, clearcoatRoughness: 0.25}); shellMats.push(m); return m; }
    if (k === 'chassis') { const m = new THREE.MeshPhysicalMaterial({color: c, roughness: 0.5, clearcoat: 0.2}); shellMats.push(m); return m; }
    return new THREE.MeshStandardMaterial({color: c, roughness: 0.6});
  }
  if (k === 'ledstrip') return new THREE.MeshStandardMaterial({color: 0xffc27a, emissive: 0xff9a3d, emissiveIntensity: 1.2});
  if (k.startsWith('pogo') || k.startsWith('dock_pad')) return new THREE.MeshStandardMaterial({color: c, metalness: 0.9, roughness: 0.3});
  if (k.startsWith('motor')) return new THREE.MeshStandardMaterial({color: c, metalness: 0.75, roughness: 0.35});
  return new THREE.MeshStandardMaterial({color: c, roughness: 0.5, metalness: 0.05});
}

Promise.all([
  fetch('assets/robot_info.json?v=' + BUILD).then(x => x.json()),
  new Promise((ok, bad) => new GLTFLoader().load('assets/wisp.gltf?v=' + BUILD, ok, undefined, bad))
]).then(([ri, gltf]) => {
  RI = ri; const g = gltf.scene; root.add(g);
  const fix = s => s.replace(/^(link|part|elec)(?!:)/, '$1:');
  const meshes = [];
  g.traverse(o => { const n = fix(o.name); if (n.startsWith('link:') && !o.isMesh) links[n.slice(5)] = o; if (o.isMesh) meshes.push(o); });
  for (const o of meshes) {
    let n = fix(o.name); if (!ri.items[n]) n = fix(o.parent.name);
    const it = ri.items[n]; if (!it) continue;
    o.geometry = toCreasedNormals(o.geometry.index ? o.geometry.toNonIndexed() : o.geometry, THREE.MathUtils.degToRad(35));
    o.material = makeMat(n, it); o.castShadow = true; o.receiveShadow = true;
    const node = ri.items[fix(o.name)] ? o : o.parent;
    if (!items[n]) items[n] = Object.assign({key: n, nodes: [], p0: []}, it);
    if (!items[n].nodes.includes(node)) { items[n].nodes.push(node); items[n].p0.push(node.position.clone()); }
    o.userData.key = n;
  }
  for (const l of Object.values(links)) { l.userData.q0 = l.quaternion.clone(); }
  // the eye: a glowing iris drawn on the round LCD
  if (links.head) {
    const eye = new THREE.Mesh(new THREE.CircleGeometry(16.2, 64), new THREE.MeshBasicMaterial({map: eyeTexture(), toneMapped: false}));
    eye.rotation.y = Math.PI / 2; eye.rotation.z = Math.PI / 2; eye.position.set(50.8, 0, 36.55);
    const lcd = items['elec:lcd']; (lcd ? lcd.nodes[0] : links.head).add(eye);
    if (lcd) eye.position.sub(lcd.nodes[0].position);
    eye.userData.key = 'elec:lcd';
  }
  buildUI(); applyAll(); setView(Q.get('view') || '34');
  document.body.classList.add('loaded');
}).catch(e => { $('loading').textContent = 'Could not load the model: ' + e; console.error(e); });

function buildUI() {
  const counts = {};
  for (const it of Object.values(items)) counts[it.group] = (counts[it.group] || 0) + 1;
  $('layers').innerHTML = Object.entries(GROUPS).map(([g, G]) =>
    `<label class="layer"><input type="checkbox" data-g="${g}" ${visible[g] ? 'checked' : ''}><span class="sw" style="background:${G.color}"></span>${G.name}<span class="n">${counts[g] || 0}</span></label>`).join('');
  $('layers').querySelectorAll('input').forEach(i => i.onchange = () => { visible[i.dataset.g] = i.checked; applyAll(); });
  // component list: one row per distinct component name
  const rows = {};
  for (const it of Object.values(items)) { const k = it.group + '|' + it.title; (rows[k] = rows[k] || {it, keys: [], n: 0}).keys.push(it.key); rows[k].n++; }
  let html = '', ncomp = 0;
  for (const g of Object.keys(GROUPS)) {
    const rs = Object.values(rows).filter(r => r.it.group === g); if (!rs.length) continue;
    html += `<div class="grp"><span class="sw" style="background:${GROUPS[g].color}"></span>${GROUPS[g].name}</div>`;
    for (const rw of rs) {
      ncomp++;
      const short = (rw.it.desc || '').split(/(?<=\.)\s/)[0];
      const name = rw.it.title.replace(/ \(x\d\)$/, '') + (rw.n > 1 ? ` &times;${rw.n}` : '');
      html += `<div class="c" data-keys="${rw.keys.join(',')}" data-g="${g}"><b>${name}</b><span>${short}</span></div>`;
    }
  }
  $('clist').innerHTML = html; $('ncomp').textContent = '(' + ncomp + ')';
  $('clist').querySelectorAll('.c').forEach(d => {
    d.onclick = () => select(d.dataset.keys.split(','));
    d.onmouseenter = () => highlight(d.dataset.keys.split(','), true); d.onmouseleave = () => highlight(d.dataset.keys.split(','), false);
  });
  const L = $('labels');
  for (const lb of RI.labels) {
    const it = items[lb.key]; if (!it) continue;
    const a = new THREE.Object3D(); a.position.fromArray(lb.local).sub(it.p0[0]); it.nodes[0].add(a);   // rides with the part when exploded
    const d = document.createElement('div'); d.className = 'lbl'; d.innerHTML = `<i></i><span>${it.title.replace(/ \(x\d\)$/, '')}</span>`;
    d.onclick = () => select([lb.key]); L.appendChild(d);
    const ext = lb.key.startsWith('part:') || ['elec:lcd', 'elec:camera'].includes(lb.key);
    labels.push({a, d, it, ext, key: lb.key});
  }
  const s = RI.summary;
  $('stats').innerHTML = `<div><b>${s.height_mm} mm</b>tall</div><div><b>${s.diameter_mm} mm</b>wide</div><div><b>${(s.total_mass_g / 1000).toFixed(2)} kg</b>est. mass</div><div><b>${s.printed_parts}</b>printed parts</div>`;
}

function highlight(keys, on) {
  for (const k of keys) { const it = items[k]; if (!it) continue;
    for (const n of it.nodes) n.traverse(o => { if (o.isMesh && o.material.emissive && !['part:glow_ring', 'elec:ledstrip'].includes(k)) { o.material.emissive.set(on ? 0xff6a1a : 0x000000); o.material.emissiveIntensity = on ? 0.45 : 1; } }); }
}
function select(keys) {
  if (sel) highlight(sel, false);
  sel = keys; highlight(keys, true);
  const it = items[keys[0]];
  $('info').classList.remove('hidden'); $('info-t').textContent = it.title.replace(/ \(x\d\)$/, '') + (keys.length > 1 ? ` (x${keys.length})` : '');
  $('info-k').textContent = (GROUPS[it.group] || {}).name + (it.kind === 'vendor' ? ' · maker CAD' : it.kind === 'printed' ? ' · printed part' : ' · modelled from datasheet');
  $('info-d').textContent = it.desc + (it.material ? ' Material: ' + it.material + '.' : '');
  document.querySelectorAll('#clist .c').forEach(c => c.classList.toggle('sel', c.dataset.keys.split(',').includes(keys[0])));
  labels.forEach(l => l.d.classList.toggle('sel', keys.includes(l.key)));
}

function applyAll() {
  for (const it of Object.values(items)) {
    const vis = visible[it.group];
    it.nodes.forEach((n, i) => { n.visible = vis; n.position.copy(it.p0[i]).add(new THREE.Vector3(...it.explode).multiplyScalar(explodeT)); });
  }
  for (const m of shellMats) { m.transparent = xray; m.opacity = xray ? 0.14 : 1; m.depthWrite = !xray; m.needsUpdate = true; }
  $('xray').classList.toggle('on', xray); $('lbls').classList.toggle('on', showLabels); $('demo').classList.toggle('on', demo);
  $('explode').value = explodeT;
  document.querySelectorAll('#clist .c').forEach(c => c.classList.toggle('off', !visible[c.dataset.g]));
}

document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => setView(b.dataset.view));
$('xray').onclick = () => { xray = !xray; applyAll(); };
$('lbls').onclick = () => { showLabels = !showLabels; applyAll(); };
$('demo').onclick = () => { demo = !demo; if (!demo) for (const l of Object.values(links)) l.quaternion.copy(l.userData.q0); applyAll(); };
$('explode').oninput = e => { explodeT = +e.target.value; applyAll(); };

const ray = new THREE.Raycaster(), mp = new THREE.Vector2(); let down = null;
r.domElement.addEventListener('pointerdown', e => down = [e.clientX, e.clientY]);
r.domElement.addEventListener('pointerup', e => {
  if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) return;
  const b = r.domElement.getBoundingClientRect(); mp.set((e.clientX - b.left) / b.width * 2 - 1, -(e.clientY - b.top) / b.height * 2 + 1);
  ray.setFromCamera(mp, cam);
  const hits = ray.intersectObject(root, true).filter(h => h.object.visible && h.object.userData.key && isShown(h.object) && !(xray && shellMats.includes(h.object.material)));
  if (hits.length) select([hits[0].object.userData.key]); else { if (sel) highlight(sel, false); sel = null; $('info').classList.add('hidden'); }
});
function isShown(o) { while (o) { if (!o.visible) return false; o = o.parent; } return true; }

const v = new THREE.Vector3(), q = new THREE.Quaternion(), AX = {head_tilt: [0, 1, 0], tail_wag: [0, 0, 1], wheel_l: [0, 1, 0], wheel_r: [0, 1, 0]};
function stepDemo(t) {
  if (!demo) return;
  const ang = {tail_wag: 0.55 * Math.sin(t * 7) * (0.6 + 0.4 * Math.sin(t * 0.7)), head_tilt: 0.14 * Math.sin(t * 1.3), wheel_l: t * 3, wheel_r: t * 3};
  for (const [j, a] of Object.entries(ang)) {
    const jj = RI.joints[j]; const l = jj && links[jj.node.slice(5)]; if (!l) continue;
    q.setFromAxisAngle(new THREE.Vector3(...AX[j]), a); l.quaternion.copy(l.userData.q0).multiply(q);
  }
}
function placeLabels() {
  const w = el.clientWidth, h = el.clientHeight;
  for (const L of labels) {
    const vis = showLabels && visible[L.it.group] && (L.ext || xray || !visible.shell || explodeT > 0.35);
    if (!vis) { L.d.style.display = 'none'; continue; }
    L.a.getWorldPosition(v); v.project(cam);
    if (v.z > 1) { L.d.style.display = 'none'; continue; }
    L.d.style.display = ''; L.d.style.transform = `translate(${(v.x * .5 + .5) * w}px,${(-v.y * .5 + .5) * h}px)`;
  }
}
new ResizeObserver(() => { cam.aspect = el.clientWidth / el.clientHeight; cam.updateProjectionMatrix(); r.setSize(el.clientWidth, el.clientHeight); }).observe(el);
(function loop(t) { requestAnimationFrame(loop); stepDemo((t || 0) / 1000); ctl.update(); placeLabels(); r.render(scene, cam); })();
