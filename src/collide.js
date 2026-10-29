// ---------------------------------------------------------------------------
// collide.js — broadphase + narrowphase in R^4.
//
// Shapes:   glome      — a hypersphere, the exact 4D analogue of a ball
//           hyperbox   — an oriented box, i.e. a product of four intervals
//
// Every contact normal points from body A to body B. Contact points are in
// world space, on the surface of A (or of B for the reversed test).
// ---------------------------------------------------------------------------

import * as M from './math4.js';
import { UNIT_VERTS } from './body.js';

const maxDepth = 0.003; // tolerance for "just touching"
const vertexTol = 0.006; // a corner this close to a face still counts as contact

export function collide(bodies) {
  const contacts = [];
  for (const b of bodies) b.updateAABB();

  for (let i = 0; i < bodies.length; i++) {
    const a = bodies[i];
    for (let j = i + 1; j < bodies.length; j++) {
      const b = bodies[j];
      if (a.isStatic && b.isStatic) continue;
      if (a.noRender && b.noRender) continue; // cage walls never touch each other
      if (!overlapAABB(a, b)) continue;
      const found = pairContacts(a, b);
      for (let k = 0; k < found.length; k++) contacts.push(found[k]);
    }
  }
  return contacts;
}

function overlapAABB(a, b) {
  const am = a.aabb, bm = b.aabb;
  for (let i = 0; i < 4; i++) {
    if (am[1][i] + maxDepth < bm[0][i]) return false;
    if (bm[1][i] + maxDepth < am[0][i]) return false;
  }
  return true;
}

function pairContacts(a, b) {
  const ta = a.shape.type;
  const tb = b.shape.type;

  if (ta === 'glome' && tb === 'glome') return glomeGlome(a, b);

  if (ta === 'glome' && tb === 'hyperbox') return glomeBox(a, b);
  if (ta === 'hyperbox' && tb === 'glome') {
    // Build it as (glome, box) then mirror the normal into a -> b order.
    return glomeBox(b, a).map((c) => ({
      a, b, n: M.neg(c.n), p: c.p, depth: c.depth, count: c.count,
    }));
  }
  return boxBox(a, b);
}

// --- glome / glome ---------------------------------------------------------

function glomeGlome(a, b) {
  const d = M.sub(b.pos, a.pos);
  const rsum = a.shape.radius + b.shape.radius;
  const d2 = M.len2(d);
  if (d2 >= rsum * rsum) return [];
  const dist = Math.sqrt(d2);
  const n = dist > 1e-6 ? M.scale(d, 1 / dist) : [1, 0, 0, 0];
  const depth = rsum - dist;
  const p = M.madd(a.pos, n, a.shape.radius - depth * 0.5);
  return [{ a, b, n, p, depth, count: 1 }];
}

// --- glome / hyperbox ------------------------------------------------------

/**
 * Closest point on an oriented hyper-box: clamp the centre into the box in the
 * box's own frame. That is exact in any dimension because a box is a product
 * of intervals.
 */
function glomeBox(g, box) {
  const R = box.R;
  const local = M.mulVec(box.Rt, M.sub(g.pos, box.pos));
  const h = box.shape.half;
  const c = [
    M.clamp(local[0], -h[0], h[0]),
    M.clamp(local[1], -h[1], h[1]),
    M.clamp(local[2], -h[2], h[2]),
    M.clamp(local[3], -h[3], h[3]),
  ];
  const delta = M.sub(local, c);
  const dist = M.len(delta);
  const r = g.shape.radius;

  let nLocal, depth, pLocal;
  if (dist > 1e-7) {
    if (dist >= r) return [];
    nLocal = M.scale(delta, 1 / dist); // box surface -> sphere centre
    depth = r - dist;
    pLocal = c;
  } else {
    // The centre is inside the box: escape through the nearest cell.
    let best = Infinity, bi = 0, sgn = 1;
    for (let i = 0; i < 4; i++) {
      const fd = h[i] - Math.abs(local[i]);
      if (fd < best) {
        best = fd;
        bi = i;
        sgn = local[i] >= 0 ? 1 : -1;
      }
    }
    nLocal = [0, 0, 0, 0];
    nLocal[bi] = sgn;
    depth = best + r;
    pLocal = local.slice();
    pLocal[bi] = sgn * h[bi];
  }

  const nBoxToSphere = M.mulVec(R, nLocal);
  const p = M.add(box.pos, M.mulVec(R, pLocal));
  // A = glome, B = box, so the normal must point from the glome to the box.
  return [{ a: g, b: box, n: M.neg(nBoxToSphere), p, depth, count: 1 }];
}

// --- hyperbox / hyperbox ---------------------------------------------------

const scratch = { depth: 0, axis: 0, sign: 1 };

/**
 * Box-box uses a hybrid manifold:
 *   1. SAT over the eight cell normals of both boxes picks the contact plane.
 *   2. Every corner of either box that lies on/inside the other becomes a
 *      contact point *on that plane*. Sharing one normal is what keeps a
 *      resting stack from shaking itself apart: corner normals flip
 *      arbitrarily when two faces are parallel.
 */
function boxBox(a, b) {
  let minOverlap = Infinity;
  let satN = null;
  const delta = M.sub(b.pos, a.pos);
  for (let pass = 0; pass < 2; pass++) {
    const A = pass === 0 ? a : b;
    for (let i = 0; i < 4; i++) {
      const axis = M.col(A.R, i);
      // Always measure the centre offset in the same direction (a -> b) and
      // orient the axis that way, so the resulting normal means the same
      // thing whichever body contributed the axis.
      const d = M.dot(delta, axis);
      const overlap = a.radiusAlong(axis) + b.radiusAlong(axis) - Math.abs(d);
      if (overlap < -maxDepth) return [];
      if (overlap < minOverlap) {
        minOverlap = overlap;
        satN = d >= 0 ? axis : M.neg(axis);
      }
    }
  }
  if (!satN) return [];

  const depth = Math.max(0, minOverlap);
  const points = [];

  // Corners of b inside a, then corners of a inside b. Keeping both sets is
  // fine now that they share a normal; the dedupe below merges the pairs that
  // describe the same spot.
  for (const v of b.worldVertices()) if (inside(a, v, scratch)) points.push(v);
  if (points.length === 0) {
    for (const v of a.worldVertices()) if (inside(b, v, scratch)) points.push(v);
  }

  if (points.length === 0) {
    const p = M.madd(a.pos, satN, a.radiusAlong(satN) - depth * 0.5);
    return [{ a, b, n: satN, p, depth, count: 1 }];
  }

  const out = [];
  for (const p of points) {
    if (out.length >= 8) break;
    let redundant = false;
    for (const o of out) {
      if (M.dist2(o.p, p) < 0.03) {
        redundant = true;
        break;
      }
    }
    if (!redundant) out.push({ a, b, n: satN, p, depth, count: 0 });
  }
  if (out.length === 0) {
    const p = M.madd(a.pos, satN, a.radiusAlong(satN) - depth * 0.5);
    return [{ a, b, n: satN, p, depth, count: 1 }];
  }
  for (const c of out) c.count = out.length;
  return out;
}

function inside(box, point, out) {
  const local = M.mulVec(box.Rt, M.sub(point, box.pos));
  const h = box.shape.half;
  let best = Infinity;
  let bi = -1;
  for (let i = 0; i < 4; i++) {
    const fd = h[i] - Math.abs(local[i]);
    if (fd < -vertexTol) return false;
    if (fd < best) {
      best = fd;
      bi = i;
    }
  }
  out.depth = Math.max(0, best);
  out.axis = bi;
  out.sign = local[bi] >= 0 ? 1 : -1;
  return true;
}
