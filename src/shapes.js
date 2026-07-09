import { Vec2 } from './vector.js';

export const ShapeType = { CIRCLE: 'circle', POLYGON: 'polygon' };

export class Circle {
  constructor(radius) {
    this.type = ShapeType.CIRCLE;
    this.radius = radius;
  }

  momentOfInertia(mass) {
    return (mass * this.radius * this.radius) / 2;
  }
}

export class Polygon {
  // vertices must be given in local space, counter-clockwise winding
  constructor(vertices) {
    this.type = ShapeType.POLYGON;
    this.vertices = vertices;
    this.normals = computeNormals(vertices);
  }

  static box(width, height) {
    const w = width / 2;
    const h = height / 2;
    return new Polygon([
      new Vec2(-w, -h),
      new Vec2(w, -h),
      new Vec2(w, h),
      new Vec2(-w, h),
    ]);
  }

  // moment of inertia of a polygon about its centroid, assuming uniform density
  momentOfInertia(mass) {
    let numerator = 0;
    let denominator = 0;
    const verts = this.vertices;
    for (let i = 0; i < verts.length; i++) {
      const p1 = verts[i];
      const p2 = verts[(i + 1) % verts.length];
      const crossMag = Math.abs(p1.cross(p2));
      const term = p2.dot(p2) + p2.dot(p1) + p1.dot(p1);
      numerator += crossMag * term;
      denominator += crossMag;
    }
    if (denominator === 0) return mass;
    return (mass / 6) * (numerator / denominator);
  }
}

function computeNormals(vertices) {
  const normals = [];
  for (let i = 0; i < vertices.length; i++) {
    const p1 = vertices[i];
    const p2 = vertices[(i + 1) % vertices.length];
    const edge = p2.sub(p1);
    // outward normal for a CCW polygon is the edge rotated -90 degrees
    normals.push(new Vec2(edge.y, -edge.x).normalize());
  }
  return normals;
}
