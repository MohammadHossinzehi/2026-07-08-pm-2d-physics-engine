import { Vec2 } from './vector.js';
import { ShapeType } from './shapes.js';

let idCounter = 0;

export class RigidBody {
  constructor({
    shape,
    position,
    mass = 1,
    restitution = 0.3,
    friction = 0.3,
    isStatic = false,
  }) {
    this.id = idCounter++;
    this.shape = shape;
    this.position = position ? position.clone() : Vec2.zero();
    this.velocity = Vec2.zero();
    this.angle = 0;
    this.angularVelocity = 0;
    this.restitution = restitution;
    this.friction = friction;
    this.isStatic = isStatic;

    if (isStatic) {
      this.mass = Infinity;
      this.invMass = 0;
      this.inertia = Infinity;
      this.invInertia = 0;
    } else {
      this.mass = mass;
      this.invMass = 1 / mass;
      this.inertia = shape.momentOfInertia(mass);
      this.invInertia = this.inertia > 0 ? 1 / this.inertia : 0;
    }
  }

  // apply an impulse at a point given as an offset from the body's center of mass
  applyImpulse(impulse, contactVector) {
    if (this.isStatic) return;
    this.velocity = this.velocity.add(impulse.scale(this.invMass));
    this.angularVelocity += this.invInertia * contactVector.cross(impulse);
  }

  getWorldVertices() {
    if (this.shape.type !== ShapeType.POLYGON) return [];
    const cos = Math.cos(this.angle);
    const sin = Math.sin(this.angle);
    return this.shape.vertices.map((v) => {
      const rx = v.x * cos - v.y * sin;
      const ry = v.x * sin + v.y * cos;
      return new Vec2(this.position.x + rx, this.position.y + ry);
    });
  }

  getWorldNormals() {
    if (this.shape.type !== ShapeType.POLYGON) return [];
    const cos = Math.cos(this.angle);
    const sin = Math.sin(this.angle);
    return this.shape.normals.map((n) => {
      const rx = n.x * cos - n.y * sin;
      const ry = n.x * sin + n.y * cos;
      return new Vec2(rx, ry);
    });
  }

  aabb() {
    if (this.shape.type === ShapeType.CIRCLE) {
      const r = this.shape.radius;
      return {
        minX: this.position.x - r,
        minY: this.position.y - r,
        maxX: this.position.x + r,
        maxY: this.position.y + r,
      };
    }
    const verts = this.getWorldVertices();
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const v of verts) {
      if (v.x < minX) minX = v.x;
      if (v.y < minY) minY = v.y;
      if (v.x > maxX) maxX = v.x;
      if (v.y > maxY) maxY = v.y;
    }
    return { minX, minY, maxX, maxY };
  }
}
