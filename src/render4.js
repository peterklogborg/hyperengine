// ---------------------------------------------------------------------------
// render4.js — an R^4 -> R^3 -> R^2 pipeline.
//
// Two nested perspective projections. The first one treats w as a depth axis:
// things with a larger w are nearer to the 4D viewer and therefore project
// into 3D *bigger*. The second is an ordinary 3D camera. The result is a
// picture in which the fourth dimension reads as scale + parallax, and a
// tesseract tumbling in the xw/yw planes folds itself inside out on screen.
// ---------------------------------------------------------------------------

import * as M from './math4.js';
import { UNIT_VERTS, UNIT_EDGES, GLOME_PLANES } from './body.js';
import { CAGE } from './world.js';

const BG = '#05070f';
const RING_SEGS = 26;

const clamp01 = (x) => (x < 0 ? 0 : x > 1 ? 1 : x);

export class Renderer4 {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d', { alpha: false });

    this.view = M.identity(); // 4D orientation of the world
    this.dist4 = 4.4;
    this.focal4 = 1.12;
    this.dist3 = 4.6;
    this.focal3 = 4.6;
    this.yaw = 0.42;
    this.pitch = 0.2;
    this.zoom = 220;

    this.spinOn = true;
    this.spinScale = 1;
    this.spinPlanes = [
      [2, 0.26], // x-w
      [4, 0.17], // y-w
      [5, 0.05], // z-w
    ];
    this.trails = true;
    this.showCage = true;
    this.showAxes = true;

    this.W = 1;
    this.H = 1;
    this.dpr = 1;
    this.cx = 0;
    this.cy = 0;
    this.items = [];
    this._tmpComp = [0, 0, 0, 0, 0, 0];
    this.resize();
  }

  resize() {
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const w = Math.max(1, this.canvas.clientWidth);
    const h = Math.max(1, this.canvas.clientHeight);
    this.W = w;
    this.H = h;
    this.dpr = dpr;
    this.canvas.width = Math.round(w * dpr);
    this.canvas.height = Math.round(h * dpr);
    this.cx = w * 0.5;
    this.cy = h * 0.52;
    this.zoom = Math.min(w, h) * 0.3;
  }

  /** Rotate the whole world in a set of 4D planes. */
  rotate(comp) {
    this.view = M.mul(M.expSkew(comp, 1), this.view);
  }

  update(dt) {
    if (this.spinOn) {
      const c = this._tmpComp;
      c.fill(0);
      for (const [k, rate] of this.spinPlanes) c[k] += rate * dt * this.spinScale;
      this.view = M.mul(M.expSkew(c, 1), this.view);
    }
    // Very slow orthonormalisation: 4D rotations have no gimbal lock, but the
    // product of thousands of exponentials still drifts in the last bits.
    this._spinAcc = (this._spinAcc || 0) + dt;
    if (this._spinAcc > 2) {
      this._spinAcc = 0;
      this.view = M.orthonormalize(this.view);
    }
  }

  // --- projection ----------------------------------------------------------

  _prepCamera() {
    this._cy = Math.cos(this.yaw);
    this._sy = Math.sin(this.yaw);
    this._cp = Math.cos(this.pitch);
    this._sp = Math.sin(this.pitch);
  }

  /** World point -> screen, or null if it is behind one of the cameras. */
  project(p) {
    const R = this.view;
    const x0 = R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2] + R[0][3] * p[3];
    const y0 = R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2] + R[1][3] * p[3];
    const z0 = R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2] + R[2][3] * p[3];
    const w0 = R[3][0] * p[0] + R[3][1] * p[1] + R[3][2] * p[2] + R[3][3] * p[3];

    const d4 = this.dist4 - w0;
    if (d4 < 0.22) return null;
    const s4 = this.focal4 / d4;

    const X = x0 * s4;
    const Y = y0 * s4;
    const Z = z0 * s4;

    const x1 = this._cy * X + this._sy * Z;
    const z1 = -this._sy * X + this._cy * Z;
    const y1 = this._cp * Y - this._sp * z1;
    const z2 = this._sp * Y + this._cp * z1;

    const d3 = this.dist3 - z2;
    if (d3 < 0.25) return null;
    const s3 = (this.focal3 / d3) * this.zoom;

    return {
      x: this.cx + x1 * s3,
      y: this.cy - y1 * s3,
      d4,
      s: s4 * s3,
      a: clamp01(1.18 - d4 / (this.dist4 * 1.9)),
    };
  }

  // --- frame ---------------------------------------------------------------

  draw(world) {
    const ctx = this.ctx;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.globalCompositeOperation = 'source-over';
    ctx.globalAlpha = 1;

    if (this.trails) {
      ctx.fillStyle = 'rgba(5, 7, 15, 0.42)';
    } else {
      ctx.fillStyle = BG;
    }
    ctx.fillRect(0, 0, this.W, this.H);

    this._prepCamera();
    ctx.globalCompositeOperation = 'lighter';
    ctx.lineCap = 'round';

    if (this.showCage) this.drawCage();
    if (this.showAxes) this.drawAxes();

    const items = this.items;
    items.length = 0;
    for (const b of world.bodies) {
      if (b.noRender) continue;
      const c = this.project(b.pos);
      if (!c) continue;
      items.push({ b, c });
    }
    items.sort((p, q) => q.c.d4 - p.c.d4);

    for (const it of items) {
      if (it.b.shape.type === 'glome') this.drawGlome(it.b, it.c);
      else this.drawBox(it.b, it.c);
    }

    ctx.globalCompositeOperation = 'source-over';
    ctx.globalAlpha = 1;
  }

  // --- pieces --------------------------------------------------------------

  drawBox(b, center) {
    const ctx = this.ctx;
    const h = b.shape.half;
    const R = b.R;
    const pos = b.pos;
    const pts = this._boxPts || (this._boxPts = new Array(16));
    let visible = 0;

    for (let i = 0; i < 16; i++) {
      const v = UNIT_VERTS[i];
      const lx = v[0] * h[0];
      const ly = v[1] * h[1];
      const lz = v[2] * h[2];
      const lw = v[3] * h[3];
      const p = [
        pos[0] + R[0][0] * lx + R[0][1] * ly + R[0][2] * lz + R[0][3] * lw,
        pos[1] + R[1][0] * lx + R[1][1] * ly + R[1][2] * lz + R[1][3] * lw,
        pos[2] + R[2][0] * lx + R[2][1] * ly + R[2][2] * lz + R[2][3] * lw,
        pos[3] + R[3][0] * lx + R[3][1] * ly + R[3][2] * lz + R[3][3] * lw,
      ];
      pts[i] = this.project(p);
      if (pts[i]) visible++;
    }
    if (visible < 2) return;

    const path = new Path2D();
    for (let e = 0; e < 32; e++) {
      const A = pts[UNIT_EDGES[e][0]];
      const B = pts[UNIT_EDGES[e][1]];
      if (!A || !B) continue;
      path.moveTo(A.x, A.y);
      path.lineTo(B.x, B.y);
    }

    const hue = b.hue;
    const dim = b.isStatic ? 0.55 : 1;
    const a = center.a * dim;
    const lw = Math.min(3.2, Math.max(0.55, center.s * 0.013));
    const sat = b.isStatic ? 55 : 92;

    ctx.strokeStyle = `hsla(${hue}, ${sat}%, 62%, ${0.09 * a})`;
    ctx.lineWidth = lw * 4.2;
    ctx.stroke(path);
    ctx.strokeStyle = `hsla(${hue}, ${sat}%, 74%, ${0.82 * a})`;
    ctx.lineWidth = lw;
    ctx.stroke(path);

    // vertices as tiny sparks
    ctx.fillStyle = `hsla(${hue}, 100%, 82%, ${0.6 * a})`;
    const r = Math.max(0.7, lw * 0.85);
    for (let i = 0; i < 16; i++) {
      const p = pts[i];
      if (p) ctx.fillRect(p.x - r * 0.5, p.y - r * 0.5, r, r);
    }
  }

  drawGlome(b, center) {
    const ctx = this.ctx;
    const r = b.shape.radius;
    const R = b.R;
    const pos = b.pos;
    const hue = b.hue;
    const a = center.a;

    // six great circles, one per coordinate plane
    ctx.strokeStyle = `hsla(${hue}, 95%, 72%, ${0.5 * a})`;
    ctx.lineWidth = Math.max(0.6, center.s * 0.009);
    const path = new Path2D();
    for (const [i, j] of GLOME_PLANES) {
      let started = false;
      for (let k = 0; k <= RING_SEGS; k++) {
        const t = (k / RING_SEGS) * Math.PI * 2;
        const lx = Math.cos(t) * r;
        const ly = Math.sin(t) * r;
        const p = [
          pos[0] + R[0][i] * lx + R[0][j] * ly,
          pos[1] + R[1][i] * lx + R[1][j] * ly,
          pos[2] + R[2][i] * lx + R[2][j] * ly,
          pos[3] + R[3][i] * lx + R[3][j] * ly,
        ];
        const q = this.project(p);
        if (!q) {
          started = false;
          continue;
        }
        if (started) path.lineTo(q.x, q.y);
        else {
          path.moveTo(q.x, q.y);
          started = true;
        }
      }
    }
    ctx.stroke(path);

    // soft core
    const rad = Math.max(2, r * center.s);
    const g = ctx.createRadialGradient(center.x, center.y, rad * 0.05, center.x, center.y, rad);
    g.addColorStop(0, `hsla(${hue}, 100%, 88%, ${0.5 * a})`);
    g.addColorStop(0.45, `hsla(${hue}, 100%, 65%, ${0.14 * a})`);
    g.addColorStop(1, `hsla(${hue}, 100%, 55%, 0)`);
    ctx.fillStyle = g;
    ctx.beginPath();
    ctx.arc(center.x, center.y, rad, 0, Math.PI * 2);
    ctx.fill();
  }

  drawCage() {
    const ctx = this.ctx;
    const C = CAGE;
    const pts = new Array(16);
    for (let i = 0; i < 16; i++) {
      const v = UNIT_VERTS[i];
      pts[i] = this.project([v[0] * C, v[1] * C, v[2] * C, v[3] * C]);
    }
    const path = new Path2D();
    for (let e = 0; e < 32; e++) {
      const A = pts[UNIT_EDGES[e][0]];
      const B = pts[UNIT_EDGES[e][1]];
      if (!A || !B) continue;
      path.moveTo(A.x, A.y);
      path.lineTo(B.x, B.y);
    }
    ctx.strokeStyle = 'hsla(205, 80%, 60%, 0.085)';
    ctx.lineWidth = 4;
    ctx.stroke(path);
    ctx.strokeStyle = 'hsla(205, 90%, 72%, 0.20)';
    ctx.lineWidth = 1;
    ctx.stroke(path);

    // floor grid (a 3D slice of the 4D floor, at w = 0)
    const g = new Path2D();
    const step = C / 5;
    for (let i = -5; i <= 5; i++) {
      const t = i * step;
      g.moveTo(...this._screen([t, -C, -C, 0]));
      g.lineTo(...this._screen([t, -C, C, 0]));
      g.moveTo(...this._screen([-C, -C, t, 0]));
      g.lineTo(...this._screen([C, -C, t, 0]));
    }
    ctx.strokeStyle = 'hsla(200, 85%, 65%, 0.13)';
    ctx.lineWidth = 0.8;
    ctx.stroke(g);
  }

  /** Project to an [x, y] pair, or a far-away point when clipped. */
  _screen(p) {
    const q = this.project(p);
    return q ? [q.x, q.y] : [-9999, -9999];
  }

  drawAxes() {
    const ctx = this.ctx;
    const o = [-CAGE * 0.82, -CAGE * 0.82, -CAGE * 0.82, -CAGE * 0.82];
    const len = 2.6;
    const axes = [
      { dir: [1, 0, 0, 0], hue: 0, label: 'x' },
      { dir: [0, 1, 0, 0], hue: 130, label: 'y' },
      { dir: [0, 0, 1, 0], hue: 215, label: 'z' },
      { dir: [0, 0, 0, 1], hue: 300, label: 'w' },
    ];
    const O = this.project(o);
    if (!O) return;
    ctx.font = '600 11px ui-monospace, SFMono-Regular, Menlo, monospace';
    for (const ax of axes) {
      const p = [o[0] + ax.dir[0] * len, o[1] + ax.dir[1] * len, o[2] + ax.dir[2] * len, o[3] + ax.dir[3] * len];
      const P = this.project(p);
      if (!P) continue;
      ctx.strokeStyle = `hsla(${ax.hue}, 95%, 65%, 0.55)`;
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(O.x, O.y);
      ctx.lineTo(P.x, P.y);
      ctx.stroke();
      ctx.fillStyle = `hsla(${ax.hue}, 95%, 78%, 0.8)`;
      ctx.fillText(ax.label, P.x + 4, P.y + 3);
    }
  }
}
