# Hyperengine

A four-dimensional rigid-body physics sandbox that runs in a browser tab. No
dependencies, no build step, no WebGL — just ES modules, a canvas, and a small
amount of linear algebra.

Bodies live in **R⁴**, have full **SO(4)** orientations, tumble around six
independent planes, and collide with real contact manifolds. The picture on
screen is a nested projection R⁴ → R³ → R², so the fourth axis shows up as
depth and scale: tesseracts fold inside out, glomes breathe, and stretched
hyper-boxes precess in a way that has no 3D analogue.

## Run it

ES modules are blocked on `file://`, so serve the folder:

```sh
cd hyperengine
python3 -m http.server 8000
```

Then open <http://localhost:8000>. Any static server works (`npx serve`,
`caddy file-server`, etc.); there is nothing to install or compile.

## Controls

| Input | Action |
| --- | --- |
| drag | rotate the world in the **xw** / **yw** planes |
| shift + drag | rotate in **xy** / **yz** |
| wheel | move the 4D camera in depth |
| `space` | pause / resume |
| `R` | reset the demo scene |
| `G` | toggle gravity |
| `1` / `2` | spawn a tesseract / glome |
| `F` | fire a 4D shockwave |
| `H` | hide / show the panel |
| `T` / `C` / `A` / `S` | trails / cage / axes / 4D drift |
| `X` | toggle the 3D slice view |

The panel exposes gravity, bounciness, friction, time scale and drift speed,
plus pause, single-step and reset. Every spawned body is a new random rigid
body with its own inertia tensor.

Turning on **3D slice** swaps the nested projection for a true cross-section:
the `slice w` slider (or the wheel) moves the hyperplane `w = const` through the
room, and you see only the matter that plane cuts through. Drag orbits the
ordinary 3D camera in that mode.

## What is actually 4D

- **State.** Position and linear velocity are 4-vectors. Orientation is a 4×4
  rotation matrix. Angular velocity and angular momentum are **bivectors** with
  six components, one per coordinate plane:

  ```
  [ w01, w02, w03, w12, w13, w23 ]
  ```

  In 3D a rotation has an axis; in 4D it has a plane — actually up to two
  orthogonal invariant planes at once. Storing the six components is the only
  lossless option.

- **Dynamics.** Angular *momentum* `L` is the integrated state, not `ω`.
  `ω = I_world⁻¹ L`, and `I_world` turns with the body, so an asymmetric
  tesseract visibly precesses. The world-space inverse inertia is cached as a
  6×6 matrix per body and rebuilt only when the orientation changes.

- **Integration.** Linear motion is semi-implicit Euler; orientation uses a
  midpoint (RK2) step on SO(4) via `exp(Ω dt)`, evaluated with scaling and
  squaring. Drift is corrected by periodic modified Gram–Schmidt. A free
  anisotropic body conserves energy to well under 1% over 30 simulated seconds.

- **Shapes.** Two primitives:
  - **hyperbox** (tesseract) — 16 vertices, 32 edges, anisotropic inertia
  - **glome** (3-sphere) — analytic inertia, rendered as six great circles

- **Collision.** Sphere–sphere and sphere–box are analytic. Box–box uses a
  face-normal SAT over the eight 4D axis directions and builds a shared-normal
  vertex manifold (deduplicated, capped at eight points). Contacts are solved
  with sequential impulses, a three-direction friction block, Baumgarte bias,
  restitution and positional correction, at 120 Hz internally (two substeps per
  rendered 1/60 s frame).

- **Rendering.** Points go through two perspective projections: first 4D
  (with `w` as depth, which produces the characteristic scale change), then an
  ordinary 3D camera. Everything is drawn with additive strokes, so overlapping
  hyper-boxes glow through each other as their fourth-coordinate ordering
  changes.

- **Slice view.** The second camera mode drops the R⁴ → R³ step entirely and
  intersects the world with the hyperplane `w = sliceW` instead. Hyper-boxes
  are cut face by face (the 24 square faces of the tesseract are clipped to the
  plane, then de-duplicated into 3D segments); a glome of radius `r` becomes the
  2-sphere `|x - p|² = r² - (p_w - w)²`, drawn as three great circles. Move the
  slider while the sim runs and bodies appear from nothing at one `w`, swell,
  then vanish at another: the 4D analogue of a 3D scene sliced by a plane.

## Project layout

```
index.html          canvas + control panel
style.css           dark instrument-panel styling
src/math4.js        4-vectors, 4×4 matrices, bivectors, exp(Ω), inertia
src/body.js         rigid-body state, mass properties, SO(4) integration
src/collide.js      broadphase + narrowphase, contact manifolds
src/solver.js       sequential impulses, friction, position correction
src/world.js        simulation loop, shockwaves, demo scene
src/render4.js      R⁴ → R³ → R² projection and drawing
src/main.js         fixed-timestep loop, input, HUD
tests/physics.test.mjs
```

## Tests

```sh
node tests/physics.test.mjs
```

The suite covers rotation validity, the angular-velocity/rotation derivative
relation, bivector round-trips, positive-definite inertia, torque-free energy
and momentum conservation, elastic collisions, resting contact, 3D slice
cross-sections, a 20-second run of the full demo scene, and stacking stability.

On a laptop-class CPU the demo scene (29 bodies, roughly 100 live contacts)
steps in about a millisecond per 1/60 s frame.

## Poking at it

With the page open, `window.hyperengine` exposes the live `world` and
`renderer`, plus `spawn()`, `resetScene()` and `shockwave()`, so you can drive
the simulation from the devtools console or attach your own experiments.

## Known limits

- No continuous collision detection; very fast, very thin hyper-boxes can
  tunnel.
- Contact manifolds are vertex/face based, so box–box edge–edge cases can be
  soft.
- There is no sound, no persistence and no mobile pinch gesture — the 4D camera
  is drag + wheel (or drag on a touchscreen).

The interesting part is not the polygons. It is that the rotation group has six
generators, and you can watch all of them at once.
