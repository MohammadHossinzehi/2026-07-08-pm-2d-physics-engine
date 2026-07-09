import { Vec2, Circle, Polygon, RigidBody, World } from '../src/engine.js';

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

let world;

function setup() {
  world = new World({ gravity: new Vec2(0, 600) });

  const ground = new RigidBody({
    shape: Polygon.box(860, 40),
    position: new Vec2(450, 580),
    isStatic: true,
    friction: 0.5,
  });
  world.addBody(ground);

  const leftWall = new RigidBody({
    shape: Polygon.box(40, 600),
    position: new Vec2(20, 300),
    isStatic: true,
  });
  world.addBody(leftWall);

  const rightWall = new RigidBody({
    shape: Polygon.box(40, 600),
    position: new Vec2(880, 300),
    isStatic: true,
  });
  world.addBody(rightWall);

  // a small starter stack so there is something to see immediately
  for (let i = 0; i < 3; i++) {
    world.addBody(
      new RigidBody({
        shape: Polygon.box(50, 50),
        position: new Vec2(450, 530 - i * 55),
        mass: 1,
        restitution: 0.1,
        friction: 0.4,
      }),
    );
  }

  world.addBody(
    new RigidBody({
      shape: new Circle(28),
      position: new Vec2(650, 400),
      mass: 1,
      restitution: 0.6,
      friction: 0.3,
    }),
  );
}

canvas.addEventListener('mousedown', (event) => {
  const rect = canvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;

  if (event.shiftKey) {
    world.addBody(
      new RigidBody({
        shape: new Circle(15 + Math.random() * 15),
        position: new Vec2(x, y),
        mass: 1,
        restitution: 0.6,
        friction: 0.3,
      }),
    );
  } else {
    const size = 30 + Math.random() * 30;
    world.addBody(
      new RigidBody({
        shape: Polygon.box(size, size),
        position: new Vec2(x, y),
        mass: 1,
        restitution: 0.2,
        friction: 0.4,
      }),
    );
  }
});

window.addEventListener('keydown', (event) => {
  if (event.key === 'r' || event.key === 'R') setup();
});

function render() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  for (const body of world.bodies) {
    ctx.save();
    ctx.translate(body.position.x, body.position.y);
    ctx.rotate(body.angle);
    ctx.strokeStyle = body.isStatic ? '#5a5f70' : '#7cc7ff';
    ctx.lineWidth = 2;

    if (body.shape.type === 'circle') {
      ctx.beginPath();
      ctx.arc(0, 0, body.shape.radius, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(body.shape.radius, 0);
      ctx.stroke();
    } else {
      const verts = body.shape.vertices;
      ctx.beginPath();
      ctx.moveTo(verts[0].x, verts[0].y);
      for (let i = 1; i < verts.length; i++) ctx.lineTo(verts[i].x, verts[i].y);
      ctx.closePath();
      ctx.stroke();
    }

    ctx.restore();
  }
}

const FIXED_DT = 1 / 60;
let accumulator = 0;
let lastTime = performance.now();

function loop(now) {
  const frameTime = Math.min((now - lastTime) / 1000, 0.25);
  lastTime = now;
  accumulator += frameTime;

  while (accumulator >= FIXED_DT) {
    world.step(FIXED_DT);
    accumulator -= FIXED_DT;
  }

  render();
  requestAnimationFrame(loop);
}

setup();
requestAnimationFrame(loop);
