// ---------------------------------------------------------------------------
// body.js — a rigid body in four spatial dimensions.
//
// State: position (4), orientation (SO(4) matrix), linear velocity (4) and
// angular momentum (6 bivector components). Storing angular *momentum* rather
// than angular velocity is what makes the 4D free rotation interesting: the
// world-space inertia tensor turns as the body turns, so an asymmetric
// hyper-box visibly precesses — the 4D counterpart of the Dzhanibekov effect.
// ---------------------------------------------------------------------------

import * as M from './math4.js';

let NEXT_ID = 1;

// Corners of the unit hyper-cube, and its 32 edges.
export const UNIT_VERTS = [];
for (let i = 0; i < 16; i++) {
  UNIT_VERTS.push([
    i & 1 ? 1 : -1,
    i & 2 ? 1 : -1,
    i & 4 ? 1 : -1,
    i & 8 ? 1 : -1,
  ]);
}

export const UNIT_EDGES = [];
for (let i = 0; i < 16; i++) {
  for (let j = i + 1; j < 16; j++) {
    const d = i ^ j;
    if ((d & (d - 1)) === 0) UNIT_EDGES.push([i, j]);
  }
}

export const GLOME_PLANES = [[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]];

const ZERO6 = [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]];

export class Body {
  constructor(opts = {}) {
    this.id = NEXT_ID++;
    this.shape = opts.shape ?? { type: 'hyperbox', half: [0.5, 0.5, 0.5, 0.5] };
    this.pos = opts.pos ? M.clone(opts.pos) : [0, 0, 0, 0];
    this.R = opts.R ? M.mat(opts.R) : M.identity();
    this.Rt = M.transpose(this.R);
    this.vel = opts.vel ? M.clone(opts.vel) : [0, 0, 0, 0];
    this.L = opts.L ? opts.L.slice() : [0, 0, 0, 0, 0, 0];
    this.omega = [0, 0, 0, 0, 0, 0];
    this.isStatic = !!opts.isStatic || !!opts.static;

    this.restitution = opts.restitution ?? 0.3;
    this.friction = opts.friction ?? 0.45;
    this.hue = opts.hue ?? 190;
    this.noRender = !!opts.noRender;

    const shape = this.shape;
    let volume;
    if (shape.type === 'glome') {
      volume = ((Math.PI * Math.PI) / 2) * Math.pow(shape.radius, 4);
    } else {
      const h = shape.half;
      volume = 16 * h[0] * h[1] * h[2] * h[3];
    }
    const density = opts.density ?? 0.7;
    this.mass = this.isStatic ? 0 : opts.mass ?? Math.max(0.05, volume * density);
    this.invMass = this.isStatic ? 0 : 1 / this.mass;

    // Principal moments, indexed by the six coordinate planes.
    //   hyper-box:  I_ij = m/3 (h_i^2 + h_j^2)      (uniform density)
    //   glome:      I_ij = m r^2 / 3               (uniform 4-ball)
    const I = new Array(6).fill(0);
    if (!this.isStatic) {
      for (let k = 0; k < 6; k++) {
        const [i, j] = M.PAIRS[k];
        if (shape.type === 'glome') I[k] = (this.mass * shape.radius * shape.radius) / 3;
        else I[k] = (this.mass / 3) * (shape.half[i] ** 2 + shape.half[j] ** 2);
      }
    }
    this.I = I;
    this.invI = I.map((x) => (x > 0 ? 1 / x : 0));

    this.aabb = [M.vec(), M.vec()];
    this.stepCount = 0;
    this.refreshFrames();
    this.refreshOmega();
    this.updateAABB();
  }

  get isSphere() {
    return this.shape.type === 'glome';
  }

  /**
   * Cache the world-space inverse inertia operator as a 6x6 matrix.
   * It only changes when the orientation does, so the solver can reuse it for
   * every impulse in a substep and skip six 4x4 multiplies each time.
   */
  refreshFrames() {
    if (this.isStatic) {
      this.invIw = ZERO6;
      return;
    }
    // World inverse inertia = A · diag(I⁻¹) · Aᵀ, where A is the 6x6 action
    // of the orientation on bivectors:
    //     A[(ij),(kl)] = R_ik R_jl − R_il R_jk
    // Building it directly is several times cheaper than rotating six basis
    // bivectors through two 4x4 multiplies each.
    const R = this.R;
    const A = this._A || (this._A = [[], [], [], [], [], []]);
    for (let p = 0; p < 6; p++) {
      const [i, j] = M.PAIRS[p];
      const row = A[p];
      for (let q = 0; q < 6; q++) {
        const [k, l] = M.PAIRS[q];
        row[q] = R[i][k] * R[j][l] - R[i][l] * R[j][k];
      }
    }
    const out = this.invIw && this.invIw !== ZERO6 ? this.invIw : (this.invIw = [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]);
    const invI = this.invI;
    for (let p = 0; p < 6; p++) {
      const ap = A[p];
      for (let q = 0; q <= p; q++) {
        const aq = A[q];
        let sum = 0;
        for (let m = 0; m < 6; m++) sum += ap[m] * invI[m] * aq[m];
        out[p][q] = sum;
        out[q][p] = sum;
      }
    }
  }

  /** Angular velocity is derived from angular momentum + orientation. */
  refreshOmega() {
    if (this.isStatic) {
      this.omega.fill(0);
      return;
    }
    const L = this.L;
    const M6 = this.invIw;
    let w0 = M6[0][0] * L[0] + M6[0][1] * L[1] + M6[0][2] * L[2] + M6[0][3] * L[3] + M6[0][4] * L[4] + M6[0][5] * L[5];
    let w1 = M6[1][0] * L[0] + M6[1][1] * L[1] + M6[1][2] * L[2] + M6[1][3] * L[3] + M6[1][4] * L[4] + M6[1][5] * L[5];
    let w2 = M6[2][0] * L[0] + M6[2][1] * L[1] + M6[2][2] * L[2] + M6[2][3] * L[3] + M6[2][4] * L[4] + M6[2][5] * L[5];
    let w3 = M6[3][0] * L[0] + M6[3][1] * L[1] + M6[3][2] * L[2] + M6[3][3] * L[3] + M6[3][4] * L[4] + M6[3][5] * L[5];
    let w4 = M6[4][0] * L[0] + M6[4][1] * L[1] + M6[4][2] * L[2] + M6[4][3] * L[3] + M6[4][4] * L[4] + M6[4][5] * L[5];
    let w5 = M6[5][0] * L[0] + M6[5][1] * L[1] + M6[5][2] * L[2] + M6[5][3] * L[3] + M6[5][4] * L[4] + M6[5][5] * L[5];
    const w = Math.hypot(w0, w1, w2, w3, w4, w5);
    const MAX = 42;
    if (w > MAX) {
      const s = MAX / w;
      w0 *= s; w1 *= s; w2 *= s; w3 *= s; w4 *= s; w5 *= s;
      for (let k = 0; k < 6; k++) this.L[k] *= s;
    }
    this.omega[0] = w0; this.omega[1] = w1; this.omega[2] = w2;
    this.omega[3] = w3; this.omega[4] = w4; this.omega[5] = w5;
  }

  /** Set the world-space angular velocity, deriving the stored momentum. */
  setOmega(w) {
    this.L = M.inertiaMul(this.R, this.I, w);
    this.omega = w.slice();
  }

  /** Velocity of the material point at lever arm r from the centre of mass. */
  pointVel(r) {
    const w = this.omega;
    return [
      this.vel[0] + w[0] * r[1] + w[1] * r[2] + w[2] * r[3],
      this.vel[1] - w[0] * r[0] + w[3] * r[2] + w[4] * r[3],
      this.vel[2] - w[1] * r[0] - w[3] * r[1] + w[5] * r[3],
      this.vel[3] - w[2] * r[0] - w[4] * r[1] - w[5] * r[2],
    ];
  }

  applyImpulse(J, r, sign = 1) {
    if (this.isStatic) return;
    this.vel[0] += sign * this.invMass * J[0];
    this.vel[1] += sign * this.invMass * J[1];
    this.vel[2] += sign * this.invMass * J[2];
    this.vel[3] += sign * this.invMass * J[3];
    const J0 = J[0], J1 = J[1], J2 = J[2], J3 = J[3];
    const L = this.L;
    L[0] += sign * (J0 * r[1] - J1 * r[0]);
    L[1] += sign * (J0 * r[2] - J2 * r[0]);
    L[2] += sign * (J0 * r[3] - J3 * r[0]);
    L[3] += sign * (J1 * r[2] - J2 * r[1]);
    L[4] += sign * (J1 * r[3] - J3 * r[1]);
    L[5] += sign * (J2 * r[3] - J3 * r[2]);
    this.refreshOmega();
  }

  /** Half-width of the body along a world-space direction. */
  radiusAlong(axis) {
    if (this.shape.type === 'glome') return this.shape.radius;
    let r = 0;
    for (let i = 0; i < 4; i++) {
      r += this.shape.half[i] * Math.abs(axis[0] * this.R[0][i] + axis[1] * this.R[1][i] + axis[2] * this.R[2][i] + axis[3] * this.R[3][i]);
    }
    return r;
  }

  updateAABB() {
    const min = this.aabb[0];
    const max = this.aabb[1];
    if (this.shape.type === 'glome') {
      const r = this.shape.radius;
      for (let i = 0; i < 4; i++) {
        min[i] = this.pos[i] - r;
        max[i] = this.pos[i] + r;
      }
    } else {
      for (let i = 0; i < 4; i++) {
        const e =
          this.shape.half[0] * Math.abs(this.R[i][0]) +
          this.shape.half[1] * Math.abs(this.R[i][1]) +
          this.shape.half[2] * Math.abs(this.R[i][2]) +
          this.shape.half[3] * Math.abs(this.R[i][3]);
        min[i] = this.pos[i] - e;
        max[i] = this.pos[i] + e;
      }
    }
  }

  integrate(dt) {
    if (this.isStatic) return;
    this.pos = M.madd(this.pos, this.vel, dt);
    // Midpoint (RK2) step on SO(4): evaluate the angular velocity halfway
    // through the turn. One evaluation is enough to make the free tumble
    // conserve energy to well under a percent over a minute of simulation.
    const half = M.mul(M.expSkew(this.omega, dt * 0.5), this.R);
    const wMid = M.invInertiaMul(half, this.invI, this.L);
    this.R = M.mul(M.expSkew(wMid, dt), this.R);
    if ((this.stepCount++ & 63) === 0) this.R = M.orthonormalize(this.R);
    this.Rt = M.transpose(this.R);
    this.refreshFrames();
    // Angular momentum is constant during free flight; omega follows the body.
    this.refreshOmega();
    this.updateAABB();
  }

  worldVertices() {
    const out = [];
    const h = this.shape.half;
    for (let i = 0; i < 16; i++) {
      const v = UNIT_VERTS[i];
      const local = [v[0] * h[0], v[1] * h[1], v[2] * h[2], v[3] * h[3]];
      out.push(M.add(this.pos, M.mulVec(this.R, local)));
    }
    return out;
  }

  kineticEnergy() {
    if (this.isStatic) return 0;
    let e = 0.5 * this.mass * M.len2(this.vel);
    // ½ ω·L is frame-independent; ½Σ ω²I would only be valid in the body frame.
    for (let k = 0; k < 6; k++) e += 0.5 * this.omega[k] * this.L[k];
    return e;
  }

  /** Soft, purely visual glow radius used by the renderer. */
  get boundRadius() {
    if (this.shape.type === 'glome') return this.shape.radius;
    return Math.hypot(...this.shape.half);
  }
}
