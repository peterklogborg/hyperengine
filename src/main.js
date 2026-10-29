// ---------------------------------------------------------------------------
// main.js — glues the 4D world, the renderer and the control panel together.
//
// The simulation runs on a fixed 1/60 s timestep with a bounded accumulator,
// so physics is deterministic regardless of display refresh rate; rendering
// just draws whatever the latest state is.
// ---------------------------------------------------------------------------

import { World, populateDemo, CAGE } from './world.js';
import { Renderer4 } from './render4.js';

const DT = 1 / 60;
const MAX_STEPS = 6;
const MAX_DYNAMIC = 64;

const $ = (id) => document.getElementById(id);

const canvas = $('view');
const renderer = new Renderer4(canvas);
const world = new World();
populateDemo(world);

const ui = {
  panel: $('panel'),
  reveal: $('reveal'),
  hidePanel: $('hidePanel'),
  status: $('status'),
  statusText: $('statusText'),
  statFps: $('statFps'),
  statBodies: $('statBodies'),
  statContacts: $('statContacts'),
  statMs: $('statMs'),
  btnPause: $('btnPause'),
  btnStep: $('btnStep'),
  btnReset: $('btnReset'),
  btnCube: $('btnCube'),
  btnGlome: $('btnGlome'),
  btnShock: $('btnShock'),
  gravity: $('gravity'),
  valGravity: $('valGravity'),
  rest: $('rest'),
  valRest: $('valRest'),
  fric: $('fric'),
  valFric: $('valFric'),
  time: $('time'),
  valTime: $('valTime'),
  spin: $('spin'),
  valSpin: $('valSpin'),
  tgSpin: $('tgSpin'),
  tgTrails: $('tgTrails'),
  tgCage: $('tgCage'),
  tgAxes: $('tgAxes'),
  tgSlice: $('tgSlice'),
  slice: $('slice'),
  valSlice: $('valSlice'),
  toast: $('toast'),
};

const sliders = [
  { el: ui.gravity, out: ui.valGravity, fmt: (v) => v.toFixed(1) },
  { el: ui.rest, out: ui.valRest, fmt: (v) => v.toFixed(2) },
  { el: ui.fric, out: ui.valFric, fmt: (v) => v.toFixed(2) },
  { el: ui.time, out: ui.valTime, fmt: (v) => `${v.toFixed(2)}×` },
  { el: ui.spin, out: ui.valSpin, fmt: (v) => `${v.toFixed(2)}×` },
  { el: ui.slice, out: ui.valSlice, fmt: (v) => `${v >= 0 ? '+' : ''}${v.toFixed(2)}` },
];
const sliceSlider = sliders[sliders.length - 1];

let paused = false;
let accumulator = 0;
let timeScale = 1;
let lastGravity = 7.6;
let physicsMs = 0;
let toastTimer = 0;

// --- settings ---------------------------------------------------------------

function syncSlider(s) {
  const min = Number(s.el.min);
  const max = Number(s.el.max);
  const v = Number(s.el.value);
  const pct = max > min ? ((v - min) / (max - min)) * 100 : 0;
  s.el.style.setProperty('--fill', `${pct.toFixed(2)}%`);
  s.out.textContent = s.fmt(v);
}

function applySettings() {
  const g = Number(ui.gravity.value);
  world.gravity = [0, -g, 0, 0];
  world.setRestitution(Number(ui.rest.value));
  world.setFriction(Number(ui.fric.value));
  timeScale = Number(ui.time.value);
  renderer.spinScale = Number(ui.spin.value);
  renderer.sliceW = Number(ui.slice.value);
  for (const s of sliders) syncSlider(s);
}

// --- transport --------------------------------------------------------------

function setPaused(p) {
  paused = p;
  accumulator = 0;
  ui.btnPause.textContent = paused ? 'Resume' : 'Pause';
  ui.btnPause.classList.toggle('paused', paused);
  ui.status.classList.toggle('paused', paused);
  ui.statusText.textContent = paused ? 'paused' : 'running';
}

function stepOnce() {
  if (!paused) setPaused(true);
  world.step(DT);
  accumulator = 0;
  updateStats();
}

function resetScene() {
  populateDemo(world);
  applySettings();
  accumulator = 0;
  updateStats();
  toast('scene reset');
}

// --- actions ----------------------------------------------------------------

function spawn(kind) {
  const dynamic = world.bodies.filter((b) => !b.isStatic);
  if (dynamic.length >= MAX_DYNAMIC) {
    const idx = world.bodies.indexOf(dynamic[0]);
    if (idx >= 0) world.bodies.splice(idx, 1);
  }
  world.spawn(kind);
  updateStats();
  toast(kind === 'glome' ? 'glome added' : 'tesseract added');
}

function shockwave() {
  world.shockwave([0, 0, 0, 0], 22);
  toast('4D shockwave');
}

function toggleGravity() {
  const on = Number(ui.gravity.value) > 0.05;
  if (on) {
    lastGravity = Number(ui.gravity.value);
    ui.gravity.value = '0';
  } else {
    ui.gravity.value = String(lastGravity || 7.6);
  }
  applySettings();
  toast(on ? 'gravity off' : 'gravity on');
}

// --- display toggles --------------------------------------------------------

function setChip(btn, on) {
  btn.classList.toggle('on', on);
  btn.setAttribute('aria-pressed', on ? 'true' : 'false');
}

function toggleSpin() {
  renderer.spinOn = !renderer.spinOn;
  setChip(ui.tgSpin, renderer.spinOn);
  toast(renderer.spinOn ? '4D drift on' : '4D drift off');
}

function toggleTrails() {
  renderer.trails = !renderer.trails;
  setChip(ui.tgTrails, renderer.trails);
}

function toggleCage() {
  renderer.showCage = !renderer.showCage;
  setChip(ui.tgCage, renderer.showCage);
}

function toggleAxes() {
  renderer.showAxes = !renderer.showAxes;
  setChip(ui.tgAxes, renderer.showAxes);
}

function setSlice(v) {
  const w = Math.min(CAGE, Math.max(-CAGE, v));
  ui.slice.value = String(w);
  renderer.sliceW = w;
  syncSlider(sliceSlider);
}

function toggleSlice() {
  renderer.sliceMode = !renderer.sliceMode;
  setChip(ui.tgSlice, renderer.sliceMode);
  ui.slice.disabled = !renderer.sliceMode;
  if (renderer.sliceMode) toast(`3D slice at w = ${renderer.sliceW.toFixed(2)}`);
  else toast('4D projection');
}

function togglePanel() {
  const hide = !ui.panel.classList.contains('hidden');
  ui.panel.classList.toggle('hidden', hide);
  ui.reveal.classList.toggle('show', hide);
}

// --- toast ------------------------------------------------------------------

function toast(msg) {
  ui.toast.textContent = msg;
  ui.toast.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => ui.toast.classList.remove('show'), 1500);
}

// --- HUD --------------------------------------------------------------------

function dynamicCount() {
  let n = 0;
  for (const b of world.bodies) if (!b.isStatic) n++;
  return n;
}

function updateStats() {
  ui.statBodies.textContent = String(dynamicCount());
  ui.statContacts.textContent = String(world.stats.contactCount);
}

// --- input: panel -----------------------------------------------------------

// Keep the slice slider range tied to the room size.
ui.slice.min = String(-CAGE);
ui.slice.max = String(CAGE);

for (const s of sliders) {
  syncSlider(s);
  s.el.addEventListener('input', applySettings);
}

ui.btnPause.addEventListener('click', () => {
  setPaused(!paused);
  ui.btnPause.blur();
});
ui.btnStep.addEventListener('click', () => {
  stepOnce();
  ui.btnStep.blur();
});
ui.btnReset.addEventListener('click', () => {
  resetScene();
  ui.btnReset.blur();
});
ui.btnCube.addEventListener('click', () => {
  spawn('hyperbox');
  ui.btnCube.blur();
});
ui.btnGlome.addEventListener('click', () => {
  spawn('glome');
  ui.btnGlome.blur();
});
ui.btnShock.addEventListener('click', () => {
  shockwave();
  ui.btnShock.blur();
});

ui.hidePanel.addEventListener('click', togglePanel);
ui.reveal.addEventListener('click', togglePanel);

ui.tgSpin.addEventListener('click', () => {
  toggleSpin();
  ui.tgSpin.blur();
});
ui.tgTrails.addEventListener('click', () => {
  toggleTrails();
  ui.tgTrails.blur();
});
ui.tgCage.addEventListener('click', () => {
  toggleCage();
  ui.tgCage.blur();
});
ui.tgAxes.addEventListener('click', () => {
  toggleAxes();
  ui.tgAxes.blur();
});
ui.tgSlice.addEventListener('click', () => {
  toggleSlice();
  ui.tgSlice.blur();
});

// --- input: keyboard --------------------------------------------------------

window.addEventListener('keydown', (e) => {
  const tag = e.target && e.target.tagName;
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;

  if (e.code === 'Space') {
    e.preventDefault();
    if (!e.repeat) setPaused(!paused);
    return;
  }

  switch (e.code) {
    case 'KeyR':
      resetScene();
      break;
    case 'KeyG':
      toggleGravity();
      break;
    case 'Digit1':
      if (!e.repeat) spawn('hyperbox');
      break;
    case 'Digit2':
      if (!e.repeat) spawn('glome');
      break;
    case 'KeyF':
      if (!e.repeat) shockwave();
      break;
    case 'KeyH':
      if (!e.repeat) togglePanel();
      break;
    case 'KeyT':
      if (!e.repeat) toggleTrails();
      break;
    case 'KeyC':
      if (!e.repeat) toggleCage();
      break;
    case 'KeyA':
      if (!e.repeat) toggleAxes();
      break;
    case 'KeyX':
      if (!e.repeat) toggleSlice();
      break;
    case 'KeyS':
      if (!e.repeat) toggleSpin();
      break;
  }
});

// --- input: 4D orbit + depth ------------------------------------------------

let drag = null;

canvas.addEventListener('pointerdown', (e) => {
  if (e.pointerType === 'mouse' && e.button !== 0) return;
  if (!e.isPrimary) return;
  drag = { id: e.pointerId, x: e.clientX, y: e.clientY };
  try {
    canvas.setPointerCapture(e.pointerId);
  } catch {
    // Synthetic/expired pointers cannot be captured; dragging still works.
  }
  canvas.classList.add('dragging');
});

canvas.addEventListener('pointermove', (e) => {
  if (!drag || e.pointerId !== drag.id) return;
  const dx = e.clientX - drag.x;
  const dy = e.clientY - drag.y;
  if (dx === 0 && dy === 0) return;
  drag.x = e.clientX;
  drag.y = e.clientY;

  const k = 0.0055;
  const comp = [0, 0, 0, 0, 0, 0];
  if (renderer.sliceMode) {
    // The slice view is an ordinary 3D camera: drag orbits yaw/pitch directly.
    renderer.yaw += dx * k;
    renderer.pitch = Math.max(-1.35, Math.min(1.35, renderer.pitch + dy * k));
    return;
  }
  if (e.shiftKey) {
    comp[0] = dx * k; // xy
    comp[3] = dy * k; // yz
  } else {
    comp[2] = dx * k; // xw
    comp[4] = dy * k; // yw
  }
  renderer.rotate(comp);
});

function endDrag(e) {
  if (!drag || e.pointerId !== drag.id) return;
  drag = null;
  canvas.classList.remove('dragging');
}

canvas.addEventListener('pointerup', endDrag);
canvas.addEventListener('pointercancel', endDrag);
canvas.addEventListener('lostpointercapture', endDrag);

canvas.addEventListener(
  'wheel',
  (e) => {
    e.preventDefault();
    if (renderer.sliceMode) {
      setSlice(renderer.sliceW - e.deltaY * 0.004);
      return;
    }
    renderer.dist4 = Math.min(9.5, Math.max(2.5, renderer.dist4 + e.deltaY * 0.0016));
  },
  { passive: false }
);

// --- resize -----------------------------------------------------------------

window.addEventListener('resize', () => renderer.resize());

// --- main loop --------------------------------------------------------------

let last = performance.now();
let frames = 0;
let frameWindow = 0;
let fps = 60;
let hudWindow = 0;

function frame(now) {
  requestAnimationFrame(frame);

  const dtReal = Math.max(0, (now - last) / 1000);
  last = now;
  const dt = Math.min(dtReal, 0.05);

  // FPS, measured over a rolling window.
  frames++;
  frameWindow += dtReal;
  if (frameWindow >= 0.4) {
    fps = frames / frameWindow;
    frames = 0;
    frameWindow = 0;
  }

  if (!paused) {
    accumulator += dt * timeScale;
    let steps = 0;
    const t0 = performance.now();
    while (accumulator >= DT && steps < MAX_STEPS) {
      world.step(DT);
      accumulator -= DT;
      steps++;
    }
    if (accumulator > DT * MAX_STEPS) accumulator = 0;
    if (steps > 0) {
      const per = (performance.now() - t0) / steps;
      physicsMs = physicsMs > 0 ? physicsMs * 0.85 + per * 0.15 : per;
    }
  } else {
    accumulator = 0;
  }

  renderer.update(dt);
  renderer.draw(world);

  hudWindow += dtReal;
  if (hudWindow >= 0.2) {
    hudWindow = 0;
    ui.statFps.textContent = String(Math.round(fps));
    ui.statMs.textContent = physicsMs.toFixed(2);
    updateStats();
  }
}

// Keep the first frame after a hidden tab from fast-forwarding the world.
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) {
    last = performance.now();
    accumulator = 0;
  }
});

document.documentElement.classList.remove('file-mode');
applySettings();
setPaused(false);
updateStats();
requestAnimationFrame(frame);

// Handy for poking at the simulation from the devtools console (and for the
// headless browser checks in tests/).
window.hyperengine = {
  world,
  renderer,
  spawn,
  resetScene,
  shockwave,
  get paused() {
    return paused;
  },
};
