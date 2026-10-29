// ---------------------------------------------------------------------------
// world.js — simulation loop, gravity, sleeping-free damping, and the demo
// scene: a walled hyper-room with tesseracts and glomes bouncing inside.
// ---------------------------------------------------------------------------

import * as M from './math4.js';
import { Body } from './body.js';
import { collide } from './collide.js';
import { buildContacts, solveContacts, correctPositions } from './solver.js';

export const CAGE = 6.4;
export const CAGE_HALF = [CAGE, CAGE, CAGE, CAGE];

const PALETTE = [188, 202, 268, 292, 318, 44, 96, 152];
const HUES_SOFT = [206, 258, 300];

export class World {
  constructor() {
    this.bodies = [];
    this.contacts = [];
    this.gravity = [0, -7.6, 0, 0];
    this.iterations = 10;
    this.substeps = 2;
    this.time = 0;
    this.restitution = 0.32;
    this.friction = 0.5;
    this.cfg = {
      beta: 0.16,
      slop: 0.004,
      maxBias: 4.0,
      restThreshold: 0.9,
      positionBeta: 0.22,
    };
    this.stats = { contactCount: 0, solveMs: 0 };
  }

  add(body) {
    this.bodies.push(body);
    return body;
  }

  clear() {
    this.bodies.length = 0;
    this.contacts.length = 0;
    this.time = 0;
    this.stats.contactCount = 0;
    this.stats.solveMs = 0;
  }

  setRestitution(v) {
    this.restitution = v;
    for (const b of this.bodies) b.restitution = b.isStatic ? v * 1.1 : v;
  }

  setFriction(v) {
    this.friction = v;
    for (const b of this.bodies) b.friction = v;
  }

  step(dt) {
    const h = dt / this.substeps;
    for (let s = 0; s < this.substeps; s++) this._substep(h);
    this.time += dt;
  }

  _substep(h) {
    const g = this.gravity;
    const linDamp = Math.exp(-0.012 * h);
    const angDamp = Math.exp(-0.03 * h);

    for (const b of this.bodies) {
      if (b.isStatic) continue;
      b.vel[0] += g[0] * h;
      b.vel[1] += g[1] * h;
      b.vel[2] += g[2] * h;
      b.vel[3] += g[3] * h;
      for (let i = 0; i < 4; i++) b.vel[i] *= linDamp;
      for (let k = 0; k < 6; k++) b.L[k] *= angDamp;
      b.refreshOmega();
      const sp = M.len(b.vel);
      if (sp > 55) {
        const s = 55 / sp;
        for (let i = 0; i < 4; i++) b.vel[i] *= s;
      }
    }

    const raw = collide(this.bodies);
    const contacts = buildContacts(raw, h, this.cfg);
    solveContacts(contacts, this.iterations);

    for (const b of this.bodies) b.integrate(h);
    correctPositions(contacts, this.cfg);
    for (const b of this.bodies) b.updateAABB();

    // Anything that escapes the hyper-room (should be impossible, but 4D is
    // treacherous) gets teleported back above the floor.
    for (const b of this.bodies) {
      if (b.isStatic) continue;
      let out = false;
      for (let i = 0; i < 4; i++) if (Math.abs(b.pos[i]) > CAGE + 40) out = true;
      if (out) {
        b.pos = [0, CAGE * 0.6, 0, 0];
        b.vel.fill(0);
        b.L.fill(0);
        b.refreshOmega();
      }
    }

    this.contacts = contacts;
    this.stats.contactCount = contacts.length;
  }

  /** A 4D shockwave: radial impulse falling off with distance. */
  shockwave(center, strength = 16) {
    for (const b of this.bodies) {
      if (b.isStatic) continue;
      const d = M.sub(b.pos, center);
      const r2 = M.len2(d) + 0.6;
      const dir = M.normalize(d);
      const k = strength / r2;
      for (let i = 0; i < 4; i++) b.vel[i] += dir[i] * k;
      const w = [];
      for (let j = 0; j < 6; j++) w.push((Math.random() * 2 - 1) * k * 0.55);
      b.L = M.inertiaMul(b.R, b.I, w);
      b.refreshOmega();
    }
  }

  totalEnergy() {
    let e = 0;
    for (const b of this.bodies) e += b.kineticEnergy();
    return e;
  }

  /** Add one randomly-sized body somewhere in the upper half of the room. */
  spawn(kind = 'hyperbox', hue = null) {
    const pos = this._findSpot();
    const R = M.randomRotation();
    const h = hue ?? PALETTE[(Math.random() * PALETTE.length) | 0];
    let shape;
    if (kind === 'glome') {
      shape = { type: 'glome', radius: 0.38 + Math.random() * 0.34 };
    } else {
      const s = 0.32 + Math.random() * 0.34;
      // A quarter of the tesseracts are stretched: their inertia is
      // anisotropic, so they tumble in that unmistakably 4D way.
      const stretch = Math.random() < 0.3;
      shape = {
        type: 'hyperbox',
        half: [
          s * (stretch ? 0.6 + Math.random() * 0.9 : 1),
          s * (stretch ? 0.6 + Math.random() * 0.9 : 1),
          s,
          s,
        ],
      };
    }
    const b = new Body({
      shape,
      pos,
      R,
      hue: h,
      restitution: this.restitution,
      friction: this.friction,
    });
    const w = [];
    for (let k = 0; k < 6; k++) w.push((Math.random() * 2 - 1) * 3.4);
    b.L = M.inertiaMul(b.R, b.I, w);
    b.refreshOmega();
    return this.add(b);
  }

  _findSpot() {
    for (let attempt = 0; attempt < 60; attempt++) {
      const p = [
        (Math.random() * 2 - 1) * (CAGE - 1.6),
        -1 + Math.random() * (CAGE - 1.2),
        (Math.random() * 2 - 1) * (CAGE - 1.6),
        (Math.random() * 2 - 1) * (CAGE - 1.6),
      ];
      let ok = true;
      for (const b of this.bodies) {
        if (b.isStatic) continue;
        if (M.dist2(p, b.pos) < 1.44) {
          ok = false;
          break;
        }
      }
      if (ok) return p;
    }
    return [0, CAGE * 0.7, 0, 0];
  }
}

/** The starting scene: hyper-room, a few pillars, and a pile of 4D bodies. */
export function populateDemo(world) {
  world.clear();
  world.gravity = [0, -7.6, 0, 0];
  world.restitution = 0.32;
  world.friction = 0.5;

  const T = 0.9;
  for (let axis = 0; axis < 4; axis++) {
    for (const sign of [-1, 1]) {
      const half = [CAGE + T, CAGE + T, CAGE + T, CAGE + T];
      half[axis] = T;
      const pos = [0, 0, 0, 0];
      pos[axis] = sign * (CAGE + T);
      world.add(
        new Body({
          shape: { type: 'hyperbox', half },
          pos,
          isStatic: true,
          noRender: true,
          restitution: 0.4,
          friction: 0.55,
        })
      );
    }
  }

  // Static blocks floating in the room — 4D obstacles to bounce off.
  const blocks = [
    { pos: [-2.6, -4.9, 0.4, 0.2], half: [1.5, 0.55, 1.1, 1.1] },
    { pos: [2.9, -4.2, -1.6, 1.8], half: [1.0, 0.7, 1.4, 1.0] },
    { pos: [0.2, -2.1, 3.1, -2.4], half: [0.9, 0.9, 0.9, 0.9] },
    { pos: [-3.8, -2.2, -2.8, 1.0], half: [0.8, 0.8, 0.8, 0.8] },
  ];
  for (let i = 0; i < blocks.length; i++) {
    const d = blocks[i];
    world.add(
      new Body({
        shape: { type: 'hyperbox', half: d.half },
        pos: d.pos,
        R: M.expSkew([0.3 * i, 0.2, 0.5 * i, 0.1, 0.4, 0.25 * i], 1),
        isStatic: true,
        hue: HUES_SOFT[i % HUES_SOFT.length],
        restitution: 0.45,
        friction: 0.55,
      })
    );
  }

  for (let i = 0; i < 14; i++) world.spawn('hyperbox');
  for (let i = 0; i < 3; i++) world.spawn('glome');

  return world;
}
