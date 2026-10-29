// ---------------------------------------------------------------------------
// math4.js — the tiny linear-algebra core the engine sits on.
//
//   vectors .... [x, y, z, w]
//   matrices ... 4 rows of 4, row-major:  m[row][col]
//   rotation ... a 4x4 orthogonal matrix (a point of SO(4))
//
// In 3D a rotation has one axis and one angle. In 4D a rotation is generated
// by a *bivector*: six independent planes of rotation, stored here as
//
//     comp = [w01, w02, w03, w12, w13, w23]
//
// which are the upper-triangle entries of the skew-symmetric matrix Omega.
// The velocity of a point r relative to the centre of mass is  v = Omega r.
// ---------------------------------------------------------------------------

export const PAIRS = [[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]];

// --- vectors ---------------------------------------------------------------

export const vec = (x = 0, y = 0, z = 0, w = 0) => [x, y, z, w];
export const clone = (a) => [a[0], a[1], a[2], a[3]];
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s, a[3] * s];
export const madd = (a, b, s) => [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s, a[3] + b[3] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
export const len2 = (a) => dot(a, a);
export const len = (a) => Math.sqrt(dot(a, a));
export const neg = (a) => [-a[0], -a[1], -a[2], -a[3]];
export const dist2 = (a, b) => {
  const x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2], w = a[3] - b[3];
  return x * x + y * y + z * z + w * w;
};
export const normalize = (a) => {
  const l = len(a);
  return l > 1e-12 ? scale(a, 1 / l) : [0, 0, 0, 0];
};
export const lerp = (a, b, t) => [
  a[0] + (b[0] - a[0]) * t,
  a[1] + (b[1] - a[1]) * t,
  a[2] + (b[2] - a[2]) * t,
  a[3] + (b[3] - a[3]) * t,
];

/** The unique direction (up to sign) orthogonal to three 4-vectors. */
export function cross4(a, b, c) {
  const det3 = (m, skip) => {
    const r = [];
    for (let i = 0; i < 4; i++) if (i !== skip) r.push(i);
    const [i0, i1, i2] = r;
    return (
      m[0][i0] * (m[1][i1] * m[2][i2] - m[1][i2] * m[2][i1]) -
      m[0][i1] * (m[1][i0] * m[2][i2] - m[1][i2] * m[2][i0]) +
      m[0][i2] * (m[1][i0] * m[2][i1] - m[1][i1] * m[2][i0])
    );
  };
  const m = [a, b, c];
  return [0, 1, 2, 3].map((i) => (i % 2 === 0 ? 1 : -1) * det3(m, i));
}

export const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);

// --- 4x4 matrices ----------------------------------------------------------

export const identity = () => [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
];

export function mat(rows) {
  return [rows[0].slice(), rows[1].slice(), rows[2].slice(), rows[3].slice()];
}

/** Column j as a vector — for a rotation matrix this is the image of e_j. */
export const col = (m, j) => [m[0][j], m[1][j], m[2][j], m[3][j]];

export function mul(a, b) {
  const out = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let i = 0; i < 4; i++) {
    const ai0 = a[i][0], ai1 = a[i][1], ai2 = a[i][2], ai3 = a[i][3];
    for (let j = 0; j < 4; j++) {
      out[i][j] =
        ai0 * b[0][j] + ai1 * b[1][j] + ai2 * b[2][j] + ai3 * b[3][j];
    }
  }
  return out;
}

export function mulVec(m, v) {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2] + m[0][3] * v[3],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2] + m[1][3] * v[3],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2] + m[2][3] * v[3],
    m[3][0] * v[0] + m[3][1] * v[1] + m[3][2] * v[2] + m[3][3] * v[3],
  ];
}

export function transpose(m) {
  return [
    [m[0][0], m[1][0], m[2][0], m[3][0]],
    [m[0][1], m[1][1], m[2][1], m[3][1]],
    [m[0][2], m[1][2], m[2][2], m[3][2]],
    [m[0][3], m[1][3], m[2][3], m[3][3]],
  ];
}

export function matScale(m, s) {
  return m.map((r) => r.map((x) => x * s));
}

/** Skew matrix of a bivector:  Omega[i][j] = comp, Omega[j][i] = -comp. */
export function skew(comp) {
  const m = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let k = 0; k < 6; k++) {
    const [i, j] = PAIRS[k];
    m[i][j] = comp[k];
    m[j][i] = -comp[k];
  }
  return m;
}

/** Upper triangle of a skew matrix, back to the 6-vector form. */
export function compOf(m) {
  const out = new Array(6);
  for (let k = 0; k < 6; k++) {
    const [i, j] = PAIRS[k];
    out[k] = 0.5 * (m[i][j] - m[j][i]);
  }
  return out;
}

// --- bivector plumbing -----------------------------------------------------

/** Angular velocity applied to a lever arm: v = Omega * r. */
export function omegaApply(comp, r) {
  const [w01, w02, w03, w12, w13, w23] = comp;
  return [
    w01 * r[1] + w02 * r[2] + w03 * r[3],
    -w01 * r[0] + w12 * r[2] + w13 * r[3],
    -w02 * r[0] - w12 * r[1] + w23 * r[3],
    -w03 * r[0] - w13 * r[1] - w23 * r[2],
  ];
}

/** Torque bivector produced by an impulse J applied at lever arm r. */
export function wedge(J, r) {
  const out = new Array(6);
  for (let k = 0; k < 6; k++) {
    const [i, j] = PAIRS[k];
    out[k] = J[i] * r[j] - J[j] * r[i];
  }
  return out;
}

export const bivectorToWorld = (R, b) => compOf(mul(mul(R, skew(b)), transpose(R)));
export const bivectorToBody = (R, b) => compOf(mul(mul(transpose(R), skew(b)), R));

/**
 * Apply a body-space inverse inertia tensor to a world-space bivector.
 * The tensor is diagonal in the body frame (our shapes are hyper-boxes and
 * hyperspheres, whose principal planes are the coordinate planes).
 */
export function invInertiaMul(R, invI, b) {
  const local = bivectorToBody(R, b);
  for (let k = 0; k < 6; k++) local[k] *= invI[k];
  return bivectorToWorld(R, local);
}

/** The forward tensor: angular velocity -> angular momentum, in world space. */
export function inertiaMul(R, I, b) {
  const local = bivectorToBody(R, b);
  for (let k = 0; k < 6; k++) local[k] *= I[k];
  return bivectorToWorld(R, local);
}

/**
 * exp(Omega * dt) — the rotation matrix you travel through by spinning at a
 * constant bivector rate for dt. Scaling-and-squaring keeps it exact enough
 * for any angular velocity.
 */
export function expSkew(comp, dt) {
  let A = matScale(skew(comp), dt);
  let norm = 0;
  for (let i = 0; i < 4; i++) {
    let s = 0;
    for (let j = 0; j < 4; j++) s += Math.abs(A[i][j]);
    if (s > norm) norm = s;
  }
  if (norm < 1e-12) return identity();
  let squarings = 0;
  while (norm > 0.25) {
    norm *= 0.5;
    squarings++;
  }
  A = matScale(A, Math.pow(2, -squarings));

  const A2 = mul(A, A);
  const A3 = mul(A2, A);
  if (norm < 0.05) {
    // Small-angle fast path: the omitted terms are below 1e-9.
    const E0 = identity();
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        E0[i][j] += A[i][j] + A2[i][j] / 2 + A3[i][j] / 6;
      }
    }
    return E0;
  }
  const A4 = mul(A3, A);
  const A5 = mul(A4, A);
  const A6 = mul(A5, A);
  const A7 = mul(A6, A);
  const A8 = mul(A7, A);
  const E = identity();
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      E[i][j] +=
        A[i][j] +
        A2[i][j] / 2 +
        A3[i][j] / 6 +
        A4[i][j] / 24 +
        A5[i][j] / 120 +
        A6[i][j] / 720 +
        A7[i][j] / 5040 +
        A8[i][j] / 40320;
    }
  }
  for (let k = 0; k < squarings; k++) {
    const sq = mul(E, E);
    for (let i = 0; i < 4; i++) E[i] = sq[i];
  }
  return E;
}

/** Modified Gram–Schmidt. Keeps a long simulation from slowly shearing. */
export function orthonormalize(m) {
  const rows = [m[0].slice(), m[1].slice(), m[2].slice(), m[3].slice()];
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < i; j++) {
      const d = dot(rows[i], rows[j]);
      for (let k = 0; k < 4; k++) rows[i][k] -= d * rows[j][k];
    }
    const l = len(rows[i]);
    if (l < 1e-9) {
      rows[i] = [0, 0, 0, 0];
      rows[i][i] = 1;
    } else {
      const inv = 1 / l;
      for (let k = 0; k < 4; k++) rows[i][k] *= inv;
    }
  }
  return rows;
}

/** A random orientation, by exponentiating six random plane angles. */
export function randomRotation(rnd = Math.random) {
  const comp = [];
  for (let k = 0; k < 6; k++) comp.push((rnd() * 2 - 1) * Math.PI);
  return expSkew(comp, 1);
}

/** A 3x3 inverse, used by the 3-D friction block. */
export function inv3(m) {
  const [a, b, c] = m;
  const A = a[0], B = a[1], C = a[2];
  const D = b[0], E = b[1], F = b[2];
  const G = c[0], H = c[1], I = c[2];
  const det = A * (E * I - F * H) - B * (D * I - F * G) + C * (D * H - E * G);
  const id = Math.abs(det) > 1e-9 ? 1 / det : 0;
  return [
    [(E * I - F * H) * id, (C * H - B * I) * id, (B * F - C * E) * id],
    [(F * G - D * I) * id, (A * I - C * G) * id, (C * D - A * F) * id],
    [(D * H - E * G) * id, (B * G - A * H) * id, (A * E - B * D) * id],
  ];
}
