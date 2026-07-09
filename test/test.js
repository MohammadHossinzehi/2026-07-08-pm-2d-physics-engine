import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Vec2, Circle, Polygon, RigidBody, World, detectCollision } from '../src/engine.js';
import { SpatialHashGrid } from '../src/broadphase.js';

test('Vec2 basic operations', () => {
  const a = new Vec2(3, 4);
  const b = new Vec2(1, 2);
  assert.equal(a.length(), 5);
  assert.deepEqual(a.add(b), new Vec2(4, 6));
  assert.deepEqual(a.sub(b), new Vec2(2, 2));
  assert.equal(a.dot(b), 11);
  assert.equal(a.cross(b), 3 * 2 - 4 * 1);
  const n = a.normalize();
  assert.ok(Math.abs(n.length() - 1) < 1e-9);
});

test('circle vs circle: no collision when far apart', () => {
  const a = new RigidBody({ shape: new Circle(10), position: new Vec2(0, 0) });
  const b = new RigidBody({ shape: new Circle(10), position: new Vec2(100, 0) });
  assert.equal(detectCollision(a, b), null);
});

test('circle vs circle: collision when overlapping', () => {
  const a = new RigidBody({ shape: new Circle(10), position: new Vec2(0, 0) });
  const b = new RigidBody({ shape: new Circle(10), position: new Vec2(15, 0) });
  const m = detectCollision(a, b);
  assert.ok(m);
  assert.equal(m.contacts.length, 1);
  assert.ok(Math.abs(m.contacts[0].penetration - 5) < 1e-9);
  assert.ok(Math.abs(m.normal.x - 1) < 1e-9);
});

test('box vs box collision detects overlap', () => {
  const a = new RigidBody({ shape: Polygon.box(20, 20), position: new Vec2(0, 0) });
  const b = new RigidBody({ shape: Polygon.box(20, 20), position: new Vec2(15, 0) });
  const m = detectCollision(a, b);
  assert.ok(m);
  assert.ok(m.contacts.length >= 1);
  assert.ok(m.contacts.every((c) => c.penetration > 0));
  assert.ok(m.normal.x > 0);
});

test('box vs box: no collision when separated', () => {
  const a = new RigidBody({ shape: Polygon.box(20, 20), position: new Vec2(0, 0) });
  const b = new RigidBody({ shape: Polygon.box(20, 20), position: new Vec2(100, 0) });
  assert.equal(detectCollision(a, b), null);
});

test('circle vs polygon collision', () => {
  const circle = new RigidBody({ shape: new Circle(10), position: new Vec2(0, 0) });
  const box = new RigidBody({ shape: Polygon.box(20, 20), position: new Vec2(15, 0) });
  const m = detectCollision(circle, box);
  assert.ok(m);
  assert.equal(m.contacts.length, 1);
  assert.ok(m.contacts[0].penetration > 0);
});

test('circle vs polygon: no collision when separated', () => {
  const circle = new RigidBody({ shape: new Circle(10), position: new Vec2(0, 0) });
  const box = new RigidBody({ shape: Polygon.box(20, 20), position: new Vec2(100, 0) });
  assert.equal(detectCollision(circle, box), null);
});

test('static body has infinite mass and never moves under gravity', () => {
  const ground = new RigidBody({ shape: Polygon.box(100, 20), position: new Vec2(0, 100), isStatic: true });
  assert.equal(ground.invMass, 0);
  const world = new World();
  world.addBody(ground);
  for (let i = 0; i < 60; i++) world.step(1 / 60);
  assert.equal(ground.position.y, 100);
});

test('a falling circle comes to rest on a static ground without sinking through it', () => {
  const world = new World({ gravity: new Vec2(0, 500) });
  const ground = new RigidBody({
    shape: Polygon.box(400, 40),
    position: new Vec2(0, 100),
    isStatic: true,
    friction: 0.5,
  });
  world.addBody(ground);
  const ball = new RigidBody({ shape: new Circle(10), position: new Vec2(0, 0), restitution: 0, friction: 0.5 });
  world.addBody(ball);

  for (let i = 0; i < 240; i++) world.step(1 / 60);

  // Sample a settling window instead of a single instant: a simple
  // sequential-impulse solver has small resting jitter frame to frame, so we
  // check the ball stays close to its resting height rather than requiring
  // an exactly zero velocity at one sampled instant.
  const groundTop = 100 - 20; // ground center y=100, half-height 20
  const expectedY = groundTop - 10; // resting circle center = ground top - radius
  let maxDeviation = 0;
  for (let i = 0; i < 60; i++) {
    world.step(1 / 60);
    maxDeviation = Math.max(maxDeviation, Math.abs(ball.position.y - expectedY));
  }
  assert.ok(maxDeviation < 2, `expected to settle near y=${expectedY}, max deviation was ${maxDeviation}`);
});

test('a stack of boxes remains standing on a static ground', () => {
  // Three boxes, exactly edge-aligned above a static floor: a classic worst
  // case for a simple sequential-impulse solver (any tiny numerical
  // asymmetry can nudge a perfectly balanced tower). This checks the stack
  // stays upright and bounded rather than requiring pixel-perfect stillness.
  const world = new World({ gravity: new Vec2(0, 500) });
  const ground = new RigidBody({
    shape: Polygon.box(400, 40),
    position: new Vec2(0, 200),
    isStatic: true,
    friction: 0.6,
  });
  world.addBody(ground);

  const boxes = [];
  for (let i = 0; i < 3; i++) {
    const box = new RigidBody({
      shape: Polygon.box(40, 40),
      position: new Vec2(0, 160 - i * 40),
      restitution: 0.05,
      friction: 0.6,
    });
    world.addBody(box);
    boxes.push(box);
  }

  for (let i = 0; i < 400; i++) world.step(1 / 60);

  // the stack should not have collapsed sideways
  for (const box of boxes) {
    assert.ok(Math.abs(box.position.x) < 15, `box drifted sideways to x=${box.position.x}`);
  }
  // boxes should remain roughly stacked in order (increasing height => decreasing y)
  for (let i = 1; i < boxes.length; i++) {
    assert.ok(boxes[i - 1].position.y > boxes[i].position.y + 20, 'stack order should be preserved');
  }
  // no body should have picked up runaway velocity (a sign of a tunneling/instability bug)
  for (const box of boxes) {
    assert.ok(box.velocity.length() < 50, `box velocity did not settle: ${box.velocity.length()}`);
  }
});

test('spatial hash grid proposes nearby pairs and skips far apart bodies', () => {
  const grid = new SpatialHashGrid(10);
  const a = new RigidBody({ shape: new Circle(2), position: new Vec2(0, 0) });
  const b = new RigidBody({ shape: new Circle(2), position: new Vec2(3, 0) });
  const c = new RigidBody({ shape: new Circle(2), position: new Vec2(500, 500) });
  grid.insert(a);
  grid.insert(b);
  grid.insert(c);
  const pairs = grid.possiblePairs();
  const pairKeys = pairs.map(([x, y]) => `${x.id}_${y.id}`);
  assert.ok(pairKeys.includes(`${a.id}_${b.id}`));
  assert.ok(!pairKeys.some((k) => k.includes(`${c.id}`)));
});

test('elastic circle-circle collision approximately conserves momentum', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  const a = new RigidBody({ shape: new Circle(10), position: new Vec2(-15, 0), restitution: 1, friction: 0 });
  const b = new RigidBody({ shape: new Circle(10), position: new Vec2(15, 0), restitution: 1, friction: 0 });
  a.velocity = new Vec2(100, 0);
  world.addBody(a);
  world.addBody(b);

  const momentumBefore = a.mass * a.velocity.x + b.mass * b.velocity.x;

  for (let i = 0; i < 30; i++) world.step(1 / 60);

  const momentumAfter = a.mass * a.velocity.x + b.mass * b.velocity.x;
  assert.ok(Math.abs(momentumBefore - momentumAfter) < 2, 'momentum should be approximately conserved');
  assert.ok(b.velocity.x > 1, 'body b should have picked up velocity from the impact');
});
