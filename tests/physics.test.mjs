// Headless checks for the 4D engine. Run:  node tests/physics.test.mjs
import * as M from '../src/math4.js';
import { Body } from '../src/body.js';
import { World, populateDemo, CAGE } from '../src/world.js';
import { sliceBoxSegments, sliceGlome } from '../src/render4.js';

let failures = 0;
function ok(name, cond, extra = '') {
  if (cond) console.log(`  pass  ${name}`);
  else {
    failures++;
    console.log(`  FAIL  ${name} ${extra}`);
  }
}
const near = (a, b, eps) => Math.abs(a - b) <= eps;
const finite = (x) => Number.isFinite(x);

// --- 1. rotations ----------------------------------------------------------
console.log('\nmath');
for (let trial = 0; trial < 50; trial++) {
  const comp = Array.from({ length: 6 }, () => (Math.random() * 2 - 1) * 6);
  const R = M.expSkew(comp, 0.37);
  const RtR = M.mul(M.transpose(R), R);
  let err = 0;
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) err = Math.max(err, Math.abs(RtR[i][j] - (i === j ? 1 : 0)));
  const det = det4(R);
  if (err > 1e-9 || Math.abs(det - 1) > 1e-9) {
    ok('expSkew is a proper rotation', false, `err=${err} det=${det}`);
    break;
  }
  if (trial === 49) ok('expSkew is a proper rotation (50 random bivectors)', true);
}

// small-angle check: exp(dt*Omega) r == r + dt*(Omega r) + O(dt^2)
{
  const comp = [0.4, -0.2, 0.9, 0.3, -0.7, 0.15];
  const r = [1.3, -0.4, 0.7, -1.1];
  const dt = 1e-6;
  const R = M.expSkew(comp, dt);
  const rotated = M.mulVec(R, r);
  const dash = M.madd(r, M.omegaApply(comp, r), dt);
  let err = 0;
  for (let i = 0; i < 4; i++) err = Math.max(err, Math.abs(rotated[i] - dash[i]));
  ok('angular velocity matches the rotation derivative', err < 1e-9, `err=${err}`);
}

// bivector round-trip through the body frame
{
  const R = M.randomRotation();
  const b = [0.3, -1.2, 0.8, 0.5, 0.1, -0.6];
  const back = M.bivectorToWorld(R, M.bivectorToBody(R, b));
  ok('bivector frame round-trip', back.every((x, i) => near(x, b[i], 1e-9)));
}

// inertia operator is a positive quadratic form (energy must never go negative)
{
  const R = M.randomRotation();
  const e = [1, 2, 3, 4, 5, 6].map((k) => k / 4).map((x) => (x = 1 / x));
  let worst = Infinity;
  for (let t = 0; t < 40; t++) {
    const w = Array.from({ length: 6 }, () => Math.random() * 2 - 1);
    const L = M.inertiaMul(R, e.map((x) => 1 / x), w);
    let q = 0;
    for (let k = 0; k < 6; k++) q += w[k] * L[k];
    worst = Math.min(worst, q);
  }
  ok('I is positive definite', worst > 0, `min=${worst}`);
}

// --- 2. free rotation conserves energy and |L| -----------------------------
console.log('\nfree rigid body (anisotropic hyper-box)');
{
  const b = new Body({ shape: { type: 'hyperbox', half: [1.3, 0.5, 0.8, 0.6] }, pos: [0, 0, 0, 0] });
  b.setOmega([1.7, -0.9, 0.4, 2.1, 0.3, -1.2]);
  const e0 = b.kineticEnergy();
  const L0 = M.len(b.L);
  const dt = 1 / 240;
  let maxE = e0, maxL = 0;
  for (let i = 0; i < 240 * 30; i++) {
    b.integrate(dt);
    maxE = Math.max(maxE, b.kineticEnergy());
    maxL = Math.max(maxL, Math.abs(M.len(b.L) - L0));
  }
  const drift = Math.abs(b.kineticEnergy() - e0) / e0;
  ok('energy conserved in torque-free tumble', drift < 0.01, `drift=${(drift * 100).toFixed(3)}%`);
  ok('angular momentum conserved', maxL < 1e-6, `maxL=${maxL}`);
  ok('orientation stays orthonormal', (() => {
    const A = M.mul(M.transpose(b.R), b.R);
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) if (Math.abs(A[i][j] - (i === j ? 1 : 0)) > 1e-6) return false;
    return true;
  })());
}

// --- 3. head-on elastic collision -----------------------------------------
console.log('\nelastic collision');
{
  const w = new World();
  w.gravity = [0, 0, 0, 0];
  w.cfg.restThreshold = 0.05;
  const a = new Body({ shape: { type: 'glome', radius: 0.4 }, pos: [-1.2, 0, 0, 0], vel: [2, 0, 0, 0], restitution: 1, friction: 0 });
  const b = new Body({ shape: { type: 'glome', radius: 0.4 }, pos: [1.2, 0, 0, 0], vel: [-2, 0, 0, 0], restitution: 1, friction: 0 });
  w.add(a); w.add(b);
  for (let i = 0; i < 240; i++) w.step(1 / 120);
  ok('equal masses exchange velocity', near(a.vel[0], -2, 0.15) && near(b.vel[0], 2, 0.15), `a=${a.vel[0].toFixed(3)} b=${b.vel[0].toFixed(3)}`);
  ok('no spurious 4D drift', Math.abs(a.pos[3]) < 1e-9 && Math.abs(b.pos[2]) < 1e-9);
}

// --- 4. a glome settles on the floor --------------------------------------
console.log('\nresting contact');
{
  const w = new World();
  w.gravity = [0, -7.6, 0, 0];
  const floor = new Body({ shape: { type: 'hyperbox', half: [10, 0.5, 10, 10] }, pos: [0, -1.5, 0, 0], isStatic: true });
  const ball = new Body({ shape: { type: 'glome', radius: 0.5 }, pos: [0, 3, 0, 0], restitution: 0.1, friction: 0.4 });
  w.add(floor); w.add(ball);
  for (let i = 0; i < 600; i++) w.step(1 / 120);
  const expected = -1.0 + 0.5;
  ok('ball rests on the surface', near(ball.pos[1], expected, 0.03), `y=${ball.pos[1].toFixed(4)} expected=${expected}`);
  ok('ball stops', Math.abs(ball.vel[1]) < 0.08, `vy=${ball.vel[1].toFixed(4)}`);
  ok('ball stays centred', Math.abs(ball.pos[0]) < 0.01 && Math.abs(ball.pos[3]) < 0.01);
}

// --- 5. the whole demo scene survives, and stays in the room ---------------
console.log('\ndemo scene, 20 simulated seconds');
{
  const w = new World();
  populateDemo(w);
  const n = w.bodies.length;
  let maxSpeed = 0, bad = 0;
  const t0 = Date.now();
  for (let i = 0; i < 120 * 20; i++) {
    w.step(1 / 120);
    for (const b of w.bodies) {
      if (b.isStatic) continue;
      if (!b.pos.every(finite) || !b.vel.every(finite) || !b.omega.every(finite)) bad++;
      maxSpeed = Math.max(maxSpeed, M.len(b.vel));
      for (let k = 0; k < 4; k++) if (Math.abs(b.pos[k]) > CAGE + 0.6) bad++;
    }
  }
  const ms = Date.now() - t0;
  ok(`${n} bodies stay finite and inside the room`, bad === 0, `bad=${bad}`);
  ok('speeds stay bounded', maxSpeed < 25, `max=${maxSpeed.toFixed(2)}`);
  ok('nothing explodes', w.totalEnergy() < 500, `E=${w.totalEnergy().toFixed(1)}`);
  ok('runtime is sane', ms < 20000, `${ms}ms total for 2400 steps (${(ms / 2400).toFixed(2)}ms/step)`);
  const resting = w.bodies.filter((b) => !b.isStatic && M.len(b.vel) < 0.25).length;
  console.log(`  info  ${w.stats.contactCount} live contacts, ${resting}/${n - 12} bodies settled, E=${w.totalEnergy().toFixed(1)}`);
}

// --- 6. stacking ----------------------------------------------------------
console.log('\nstacking');
{
  const w = new World();
  w.gravity = [0, -7.6, 0, 0];
  w.add(new Body({ shape: { type: 'hyperbox', half: [8, 0.5, 8, 8] }, pos: [0, -1.5, 0, 0], isStatic: true, friction: 0.6 }));
  const boxes = [];
  for (let i = 0; i < 4; i++) {
    const b = new Body({
      shape: { type: 'hyperbox', half: [0.5, 0.5, 0.5, 0.5] },
      pos: [0, -0.5 + i * 1.02, 0, 0],
      friction: 0.6,
      restitution: 0.05,
    });
    w.add(b);
    boxes.push(b);
  }
  for (let i = 0; i < 900; i++) w.step(1 / 120);
  let drift = 0;
  for (let i = 0; i < boxes.length; i++) {
    drift = Math.max(drift, Math.abs(boxes[i].pos[0]), Math.abs(boxes[i].pos[2]), Math.abs(boxes[i].pos[3]));
  }
  const top = boxes[3].pos[1];
  ok('stack does not slide apart', drift < 0.12, `drift=${drift.toFixed(3)}`);
  ok('stack does not sink', top > 2.2, `top y=${top.toFixed(3)}`);
  ok('stack comes to rest', boxes.every((b) => M.len(b.vel) < 0.4));
}

// --- 8. 3D slice -----------------------------------------------------------
console.log('\n4D slice');
{
  const cube = new Body({ shape: { type: 'hyperbox', half: [1, 1, 1, 1] }, pos: [0, 0, 0, 0] });
  const segs = sliceBoxSegments(cube, 0);
  ok('axis-aligned hyper-cube slices to a cube wireframe', segs.length === 12, `segments=${segs.length}`);
  ok(
    'slice points lie inside the hyper-cube',
    segs.every((s) => s.every((p) => p.every((x) => Math.abs(x) <= 1.0001)))
  );
  ok('slice outside the body is empty', sliceBoxSegments(cube, 1.5).length === 0);

  const rotated = new Body({
    shape: { type: 'hyperbox', half: [1.4, 1.1, 0.9, 1.2] },
    pos: [0.3, -0.2, 0.1, 0],
    R: M.expSkew([0.7, -0.4, 0.9, 0.35, -0.6, 0.2], 1),
  });
  const rsegs = sliceBoxSegments(rotated, 0.15);
  ok('rotated tesseract has a non-empty slice', rsegs.length > 0, `segments=${rsegs.length}`);
  ok(
    'rotated slice stays inside the body hull',
    rsegs.every((s) => s.every((p) => p.every((x) => Math.abs(x) <= 3.2)))
  );

  const ball = new Body({ shape: { type: 'glome', radius: 2 }, pos: [0, 0, 0, 1.5] });
  const cut = sliceGlome(ball, 0);
  ok('glome slice is a 2-sphere', !!cut && near(cut.radius, Math.sqrt(4 - 2.25), 1e-9), cut && `r=${cut.radius}`);
  ok('glome slice misses outside its radius', sliceGlome(ball, 4) === null);

  const demo = populateDemo(new World());
  const hit = demo.bodies.filter((b) => !b.noRender).filter((b) => b.shape.type === 'glome' ? sliceGlome(b, 0) : sliceBoxSegments(b, 0).length > 0);
  ok('demo scene has geometry in the w=0 slice', hit.length > 0, `bodies=${hit.length}`);
}

function det4(m) {
  const det3 = (a) =>
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
    a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
    a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
  let d = 0;
  for (let j = 0; j < 4; j++) {
    const minor = [];
    for (let i = 1; i < 4; i++) {
      const row = [];
      for (let k = 0; k < 4; k++) if (k !== j) row.push(m[i][k]);
      minor.push(row);
    }
    d += (j % 2 === 0 ? 1 : -1) * m[0][j] * det3(minor);
  }
  return d;
}

console.log(failures === 0 ? '\nall checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
