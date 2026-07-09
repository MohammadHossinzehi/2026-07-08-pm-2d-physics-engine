# 2D Physics Engine

A from-scratch 2D rigid body physics engine in plain JavaScript: broad-phase
spatial hashing, narrow-phase collision detection for circles and convex
polygons (SAT with full two-point clipped manifolds), a sequential-impulse
solver with restitution and Coulomb friction, and a canvas demo you can drop
shapes into.

No physics library, no build step. Everything under `src/` is dependency-free
ES modules that run directly in Node or in a browser.

## Why this exists

Most "physics engine from scratch" tutorials stop at a single ball bouncing
on a floor. The interesting (and hard) part is what happens once you have
more than one moving body touching another moving body: stacking, resting
contact, and the numerical instabilities that show up the moment a solver
takes shortcuts. This project pushes past the single-ball demo into that
territory, including a real bug that showed up during development (see
"Design decisions" below) and how it was diagnosed and fixed.

## What it does

- **Shapes**: circles and arbitrary convex polygons (boxes are a helper on
  top of the general polygon).
- **Broad phase**: a uniform spatial hash grid, so collision checks scale
  with how many bodies are actually near each other instead of every pair.
- **Narrow phase**: circle-circle, circle-polygon, and polygon-polygon (SAT)
  collision detection. Polygon-polygon contacts are computed with full
  Sutherland-Hodgman clipping of the incident edge against the reference
  face, producing up to two contact points -- this is what lets a box rest
  flat on a surface instead of teetering on a single point.
- **Solver**: a sequential-impulse solver (several velocity iterations per
  step, Ã  la Box2D-lite) with restitution (bounciness) and Coulomb friction,
  plus a separate positional-correction pass to stop bodies sinking into
  each other.
- **Demo**: an HTML canvas page where you can click to drop boxes, shift-click
  to drop circles, and watch them collide, stack, and settle.

## How to run

**Tests** (Node's built-in test runner, no dependencies to install):

```
node --test test/
```

**Demo**: serve the repo root with any static file server and open
`demo/index.html` in a browser. For example:

```
npx serve .
# then open http://localhost:3000/demo/index.html
```

(It has to be served over http:// rather than opened as a `file://` URL,
because the demo uses ES module imports, which browsers block for local files.)

## Project structure

```
src/
  vector.js      Vec2: the only math primitive everything else is built on
  shapes.js      Circle and Polygon shape definitions
  rigidbody.js   RigidBody: position, velocity, mass, inertia, world-space geometry
  broadphase.js  SpatialHashGrid: cheap candidate-pair generation
  collision.js   narrow-phase detection (circle-circle, circle-polygon, polygon-polygon)
  solver.js      sequential-impulse velocity resolution + positional correction
  world.js       World: owns the bodies and runs one fixed-timestep simulation step
  engine.js      barrel file re-exporting the public API
demo/
  index.html, demo.js   canvas demo
test/
  test.js        node:test suite covering math, collision detection, and full-step scenarios
```

## Design decisions (and a real bug this surfaced)

**Single vs. multi-point manifolds.** Circle contacts always have one contact
point. Polygon-polygon contacts use two clipped contact points rather than
one. An earlier version of this engine used a single "deepest point" contact
for box-on-box collisions, which is simpler to write but physically wrong for
resting contact: a single point cannot resist a moment, so a box balanced on
one contact point behaves like it is standing on a pin and slowly (or not so
slowly) rotates off. Switching to a two-point clipped manifold fixed that.

**Velocity iterations vs. positional correction, and a real instability bug.**
While testing stacks of boxes, bodies would occasionally gain enormous
velocity out of nowhere and shoot off-screen after a few seconds -- a classic
"physics engine explosion." Instrumented logging of contact manifolds frame
by frame traced it to the solver: positional correction (the small nudge that
separates two overlapping bodies) was being recomputed and reapplied inside
every one of the solver's velocity iterations, using the same stale
penetration depth each time. Four iterations meant up to four times the
intended correction was applied in a single step, which could shove a body
far enough that the next frame's collision detection picked the wrong
reference face entirely, compounding the error. The fix was to separate the
two concerns: `resolveVelocity()` runs for several iterations per step using
live, updated velocities (as it should), while `correctPositions()` runs
exactly once per step, after velocities have settled. The positional
correction is also clamped to a maximum distance per step as a second line of
defense, since any simple discrete (non-continuous) collision system can in
principle be handed a frame with a deeper-than-expected penetration.

**Known limitation: tall, exactly-aligned stacks.** A stack of several boxes
placed perfectly edge-to-edge is a well-known worst case for simple iterative
solvers -- it is a stable equilibrium in theory but a knife's-edge one
numerically, and even production engines rely on more machinery (warm
starting, block solving, more iterations) to keep it picture-perfect. This
engine keeps such stacks upright and bounded (verified in the tests) but does
not guarantee pixel-perfect stillness for tall towers; two or three stacked
boxes settle cleanly, which you can see in the demo.

## Testing

`test/test.js` uses Node's built-in `node:test` runner (no dependencies to
install). It covers:

- Vec2 arithmetic.
- Each pairwise narrow-phase collision case (circle-circle, circle-polygon,
  polygon-polygon), both the "no collision" and "collision detected with
  correct normal/penetration" branches.
- Full simulation scenarios run through `World.step()`: a static body never
  moving under gravity, a ball falling and settling to rest on a floor
  without sinking through it, a stack of boxes remaining upright and bounded
  rather than diverging, spatial hash grid pair generation, and approximate
  momentum conservation in an elastic collision.

Run them with `node --test test/`.
