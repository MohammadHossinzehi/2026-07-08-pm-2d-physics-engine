import { Vec2 } from './vector.js';
import { SpatialHashGrid } from './broadphase.js';
import { detectCollision } from './collision.js';
import { resolveVelocity, correctPositions } from './solver.js';

const SOLVER_ITERATIONS = 8;

export class World {
  constructor({ gravity = new Vec2(0, 500), cellSize = 32 } = {}) {
    this.gravity = gravity;
    this.bodies = [];
    this.grid = new SpatialHashGrid(cellSize);
  }

  addBody(body) {
    this.bodies.push(body);
    return body;
  }

  removeBody(body) {
    const idx = this.bodies.indexOf(body);
    if (idx !== -1) this.bodies.splice(idx, 1);
  }

  // Advances the simulation by a fixed timestep dt (seconds).
  // Order: integrate velocities/positions -> broad phase -> narrow phase ->
  // several velocity-only solver iterations -> a single positional correction pass.
  step(dt) {
    for (const body of this.bodies) {
      if (body.isStatic) continue;
      body.velocity = body.velocity.add(this.gravity.scale(dt));
      body.position = body.position.add(body.velocity.scale(dt));
      body.angle += body.angularVelocity * dt;
    }

    this.grid.clear();
    for (const body of this.bodies) this.grid.insert(body);
    const candidatePairs = this.grid.possiblePairs();

    const manifolds = [];
    for (const [a, b] of candidatePairs) {
      if (a.isStatic && b.isStatic) continue;
      const manifold = detectCollision(a, b);
      if (manifold) manifolds.push(manifold);
    }

    // Resolving the same set of contacts multiple times per step (with
    // updated velocities each pass) improves stacking stability, since each
    // pass propagates impulses a bit further through a stack of resting
    // bodies. Position is deliberately left untouched during these passes.
    for (let iteration = 0; iteration < SOLVER_ITERATIONS; iteration++) {
      for (const manifold of manifolds) {
        resolveVelocity(manifold);
      }
    }

    // Positional correction runs exactly once, using the penetration
    // computed at the start of the step.
    for (const manifold of manifolds) {
      correctPositions(manifold);
    }
  }
}
