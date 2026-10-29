// ---------------------------------------------------------------------------
// solver.js — sequential impulses on a 4D contact manifold.
//
// Each contact lives on a 3-dimensional hyperplane (the plane orthogonal to
// the normal), so friction is solved with a 3x3 coupled block — three tangent
// directions instead of the familiar two. Impulses are accumulated and clamped
// per iteration, which is what lets stacks converge without exploding.
// ---------------------------------------------------------------------------

import * as M from './math4.js';

export class Contact {
  constructor(raw, dt, cfg) {
    this.a = raw.a;
    this.b = raw.b;
    this.n = M.normalize(raw.n);
    this.p = raw.p;
    this.depth = raw.depth;
    this.count = raw.count ?? 1;
    this.ra = M.sub(this.p, this.a.pos);
    this.rb = M.sub(this.p, this.b.pos);
    this.jn = 0;
    this.jt = [0, 0, 0];
    this._v = [0, 0, 0, 0];

    const a = this.a, b = this.b, n = this.n;
    this.imSum = a.invMass + b.invMass;
    this.t = tangentBasis(n);

    // Effective mass along the normal, including the rotational response.
    const kn = this.imSum + angTerm(a, this.ra, n, n) + angTerm(b, this.rb, n, n);
    this.mn = kn > 1e-9 ? 1 / kn : 0;

    // Coupled 3x3 friction mass matrix.
    const K = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        K[i][j] =
          (i === j ? this.imSum : 0) +
          angTerm(a, this.ra, this.t[i], this.t[j]) +
          angTerm(b, this.rb, this.t[i], this.t[j]);
      }
    }
    this.mt = M.inv3(K);

    this.restitution = 0.5 * (a.restitution + b.restitution);
    this.friction = Math.sqrt(a.friction * b.friction);

    // Penetration is removed with a capped Baumgarte velocity...
    this.bias = Math.min(
      (cfg.beta / dt) * Math.max(0, this.depth - cfg.slop),
      cfg.maxBias
    );
    // ...and restitution only kicks in above a threshold, so resting contacts
    // cannot buzz.
    const vn = this.vn();
    this.bounce = vn < -cfg.restThreshold ? -this.restitution * vn : 0;
  }

  /** Relative velocity at the contact, written into `out` (no allocation). */
  relVel(out) {
    const a = this.a, b = this.b, ra = this.ra, rb = this.rb;
    const wa = a.omega, wb = b.omega;
    out[0] = b.vel[0] + wb[0] * rb[1] + wb[1] * rb[2] + wb[2] * rb[3] -
      (a.vel[0] + wa[0] * ra[1] + wa[1] * ra[2] + wa[2] * ra[3]);
    out[1] = b.vel[1] - wb[0] * rb[0] + wb[3] * rb[2] + wb[4] * rb[3] -
      (a.vel[1] - wa[0] * ra[0] + wa[3] * ra[2] + wa[4] * ra[3]);
    out[2] = b.vel[2] - wb[1] * rb[0] - wb[3] * rb[1] + wb[5] * rb[3] -
      (a.vel[2] - wa[1] * ra[0] - wa[3] * ra[1] + wa[5] * ra[3]);
    out[3] = b.vel[3] - wb[2] * rb[0] - wb[4] * rb[1] - wb[5] * rb[2] -
      (a.vel[3] - wa[2] * ra[0] - wa[4] * ra[1] - wa[5] * ra[2]);
    return out;
  }

  vn() {
    const v = this.relVel(this._v);
    const n = this.n;
    return v[0] * n[0] + v[1] * n[1] + v[2] * n[2] + v[3] * n[3];
  }

  apply(J) {
    this.a.applyImpulse(J, this.ra, -1);
    this.b.applyImpulse(J, this.rb, 1);
  }

  solveNormal() {
    const vn = this.vn();
    const target = this.bounce + this.bias;
    const dj = this.mn * (target - vn);
    const prev = this.jn;
    this.jn = Math.max(0, prev + dj);
    const d = this.jn - prev;
    if (d !== 0) {
      const n = this.n;
      this.apply([n[0] * d, n[1] * d, n[2] * d, n[3] * d]);
    }
  }

  solveFriction() {
    if (this.jn <= 1e-9) return;
    const v = this.relVel(this._v);
    const t = this.t;
    const v0 = M.dot(v, t[0]);
    const v1 = M.dot(v, t[1]);
    const v2 = M.dot(v, t[2]);
    const m = this.mt;
    const d0 = -(m[0][0] * v0 + m[0][1] * v1 + m[0][2] * v2);
    const d1 = -(m[1][0] * v0 + m[1][1] * v1 + m[1][2] * v2);
    const d2 = -(m[2][0] * v0 + m[2][1] * v1 + m[2][2] * v2);

    const prev = this.jt;
    let acc0 = prev[0] + d0;
    let acc1 = prev[1] + d1;
    let acc2 = prev[2] + d2;

    const maxF = this.friction * this.jn;
    const mag = Math.hypot(acc0, acc1, acc2);
    if (mag > maxF && mag > 1e-12) {
      const s = maxF / mag;
      acc0 *= s;
      acc1 *= s;
      acc2 *= s;
    }

    const j0 = acc0 - prev[0];
    const j1 = acc1 - prev[1];
    const j2 = acc2 - prev[2];
    prev[0] = acc0;
    prev[1] = acc1;
    prev[2] = acc2;

    if (j0 === 0 && j1 === 0 && j2 === 0) return;
    const J = [
      t[0][0] * j0 + t[1][0] * j1 + t[2][0] * j2,
      t[0][1] * j0 + t[1][1] * j1 + t[2][1] * j2,
      t[0][2] * j0 + t[1][2] * j1 + t[2][2] * j2,
      t[0][3] * j0 + t[1][3] * j1 + t[2][3] * j2,
    ];
    this.apply(J);
  }
}

/** How much velocity along `velDir` a unit impulse along `impDir` creates. */
function angTerm(body, r, impDir, velDir) {
  if (body.isStatic) return 0;
  const u = M.wedge(impDir, r);
  const M6 = body.invIw;
  const dw = [
    M6[0][0] * u[0] + M6[0][1] * u[1] + M6[0][2] * u[2] + M6[0][3] * u[3] + M6[0][4] * u[4] + M6[0][5] * u[5],
    M6[1][0] * u[0] + M6[1][1] * u[1] + M6[1][2] * u[2] + M6[1][3] * u[3] + M6[1][4] * u[4] + M6[1][5] * u[5],
    M6[2][0] * u[0] + M6[2][1] * u[1] + M6[2][2] * u[2] + M6[2][3] * u[3] + M6[2][4] * u[4] + M6[2][5] * u[5],
    M6[3][0] * u[0] + M6[3][1] * u[1] + M6[3][2] * u[2] + M6[3][3] * u[3] + M6[3][4] * u[4] + M6[3][5] * u[5],
    M6[4][0] * u[0] + M6[4][1] * u[1] + M6[4][2] * u[2] + M6[4][3] * u[3] + M6[4][4] * u[4] + M6[4][5] * u[5],
    M6[5][0] * u[0] + M6[5][1] * u[1] + M6[5][2] * u[2] + M6[5][3] * u[3] + M6[5][4] * u[4] + M6[5][5] * u[5],
  ];
  const v = M.omegaApply(dw, r);
  return v[0] * velDir[0] + v[1] * velDir[1] + v[2] * velDir[2] + v[3] * velDir[3];
}

/** Three orthonormal directions spanning the hyperplane orthogonal to n. */
function tangentBasis(n) {
  const out = [];
  for (let i = 0; i < 4 && out.length < 3; i++) {
    let t = [0, 0, 0, 0];
    t[i] = 1;
    const d = n[i];
    for (let k = 0; k < 4; k++) t[k] -= d * n[k];
    for (const b of out) {
      const p = M.dot(t, b);
      for (let k = 0; k < 4; k++) t[k] -= p * b[k];
    }
    const l = M.len(t);
    if (l > 1e-6) out.push(M.scale(t, 1 / l));
  }
  while (out.length < 3) out.push([0, 0, 0, 0]);
  return out;
}

export function buildContacts(rawContacts, dt, cfg) {
  const out = [];
  for (const raw of rawContacts) {
    if (raw.depth < -cfg.slop) continue;
    out.push(new Contact(raw, dt, cfg));
  }
  return out;
}

export function solveContacts(contacts, iterations) {
  for (let it = 0; it < iterations; it++) {
    for (let i = 0; i < contacts.length; i++) contacts[i].solveNormal();
    for (let i = 0; i < contacts.length; i++) contacts[i].solveFriction();
  }
}

/**
 * A small positional nudge on top of the velocity bias. This is what keeps
 * deep piles from visibly sinking into each other over long runs.
 */
export function correctPositions(contacts, cfg) {
  for (const c of contacts) {
    const pen = c.depth - cfg.slop;
    if (pen <= 0) continue;
    const im = c.a.invMass + c.b.invMass;
    if (im <= 0) continue;
    const corr = (pen * cfg.positionBeta) / im;
    if (c.a.invMass) c.a.pos = M.madd(c.a.pos, c.n, -corr * c.a.invMass);
    if (c.b.invMass) c.b.pos = M.madd(c.b.pos, c.n, corr * c.b.invMass);
  }
}
