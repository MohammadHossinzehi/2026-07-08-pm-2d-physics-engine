import { Vec2 } from './vector.js';
import { ShapeType } from './shapes.js';

// A manifold describes the contact between two bodies:
//   {
//     bodyA, bodyB,
//     normal: unit vector pointing from A to B,
//     contacts: [{ point: Vec2 (world space), penetration: number }, ...]
//   }
//
// Circle-circle and circle-polygon contacts always produce a single contact
// point. Polygon-polygon contacts use full Sutherland-Hodgman clipping of the
// incident edge against the reference face's side planes, producing up to two
// contact points -- this is what keeps stacked boxes from spinning out under
// a single-point contact (a single point cannot resist a moment, so a box
// resting on one contact point behaves like it is balanced on a pin).

export function detectCollision(a, b) {
  if (a.shape.type === ShapeType.CIRCLE && b.shape.type === ShapeType.CIRCLE) {
    return circleVsCircle(a, b);
  }
  if (a.shape.type === ShapeType.CIRCLE && b.shape.type === ShapeType.POLYGON) {
    return circleVsPolygon(a, b);
  }
  if (a.shape.type === ShapeType.POLYGON && b.shape.type === ShapeType.CIRCLE) {
    const res = circleVsPolygon(b, a);
    if (!res) return null;
    return { bodyA: a, bodyB: b, normal: res.normal.negate(), contacts: res.contacts };
  }
  return polygonVsPolygon(a, b);
}

function circleVsCircle(a, b) {
  const delta = b.position.sub(a.position);
  const dist = delta.length();
  const radiusSum = a.shape.radius + b.shape.radius;
  if (dist >= radiusSum) return null;
  const normal = dist > 1e-9 ? delta.scale(1 / dist) : new Vec2(1, 0);
  const penetration = radiusSum - dist;
  const point = a.position.add(normal.scale(a.shape.radius));
  return { bodyA: a, bodyB: b, normal, contacts: [{ point, penetration }] };
}

// circle is a RigidBody with a Circle shape, poly is a RigidBody with a Polygon shape.
// Returned normal points from circle (A) to poly (B).
function circleVsPolygon(circle, poly) {
  const verts = poly.getWorldVertices();
  const normals = poly.getWorldNormals();

  let maxSep = -Infinity;
  let faceIndex = 0;
  for (let i = 0; i < verts.length; i++) {
    const sep = normals[i].dot(circle.position.sub(verts[i]));
    if (sep > circle.shape.radius) return null;
    if (sep > maxSep) {
      maxSep = sep;
      faceIndex = i;
    }
  }

  const v1 = verts[faceIndex];
  const v2 = verts[(faceIndex + 1) % verts.length];
  const faceNormal = normals[faceIndex];

  if (maxSep < 1e-9) {
    // circle center is inside the polygon: push out along the reference face normal.
    // faceNormal points away from the polygon (poly -> circle); the manifold
    // convention here is normal points circle -> poly, so it is negated.
    const penetration = circle.shape.radius - maxSep;
    const point = circle.position.sub(faceNormal.scale(maxSep));
    return { bodyA: circle, bodyB: poly, normal: faceNormal.negate(), contacts: [{ point, penetration }] };
  }

  const edge = v2.sub(v1);
  const d1 = circle.position.sub(v1).dot(edge);
  const d2 = circle.position.sub(v2).dot(edge.negate());

  let normal;
  let point;
  let dist;

  if (d1 <= 0) {
    dist = circle.position.sub(v1).length();
    if (dist > circle.shape.radius) return null;
    normal = v1.sub(circle.position).normalize();
    point = v1;
  } else if (d2 <= 0) {
    dist = circle.position.sub(v2).length();
    if (dist > circle.shape.radius) return null;
    normal = v2.sub(circle.position).normalize();
    point = v2;
  } else {
    dist = maxSep;
    if (dist > circle.shape.radius) return null;
    normal = faceNormal.negate();
    point = circle.position.sub(faceNormal.scale(dist));
  }

  const penetration = circle.shape.radius - dist;
  return { bodyA: circle, bodyB: poly, normal, contacts: [{ point, penetration }] };
}

// Finds the axis (face normal of polyA) with the least penetration into polyB.
// A positive separation means the polygons are not overlapping on that axis.
function findAxisLeastPenetration(polyA, polyB) {
  const vertsA = polyA.getWorldVertices();
  const normalsA = polyA.getWorldNormals();
  const vertsB = polyB.getWorldVertices();

  let bestSep = -Infinity;
  let bestIndex = 0;

  for (let i = 0; i < normalsA.length; i++) {
    const n = normalsA[i];
    const v = vertsA[i];
    let minProj = Infinity;
    for (const vb of vertsB) {
      const proj = n.dot(vb.sub(v));
      if (proj < minProj) minProj = proj;
    }
    if (minProj > bestSep) {
      bestSep = minProj;
      bestIndex = i;
    }
  }

  return { separation: bestSep, faceIndex: bestIndex };
}

// Clips a segment [p0, p1] to the half-plane { p : tangent.dot(p) <= offset },
// returning 0, 1, or 2 points (linearly interpolating a new point at the
// plane crossing when the segment straddles it).
function clipSegment(points, tangent, offset) {
  const out = [];
  const d0 = tangent.dot(points[0]) - offset;
  const d1 = tangent.dot(points[1]) - offset;

  if (d0 <= 0) out.push(points[0]);
  if (d1 <= 0) out.push(points[1]);

  if (d0 * d1 < 0) {
    const t = d0 / (d0 - d1);
    out.push(points[0].add(points[1].sub(points[0]).scale(t)));
  }

  return out;
}

function polygonVsPolygon(a, b) {
  const resA = findAxisLeastPenetration(a, b);
  if (resA.separation > 0) return null;

  const resB = findAxisLeastPenetration(b, a);
  if (resB.separation > 0) return null;

  let refPoly;
  let incPoly;
  let refIndex;
  let flip;

  if (resB.separation > resA.separation + 1e-9) {
    refPoly = b;
    incPoly = a;
    refIndex = resB.faceIndex;
    flip = true;
  } else {
    refPoly = a;
    incPoly = b;
    refIndex = resA.faceIndex;
    flip = false;
  }

  const refVerts = refPoly.getWorldVertices();
  const refNormals = refPoly.getWorldNormals();
  const refNormal = refNormals[refIndex];
  const v1 = refVerts[refIndex];
  const v2 = refVerts[(refIndex + 1) % refVerts.length];

  // incident edge: the edge on incPoly whose normal is most anti-parallel to refNormal
  const incNormals = incPoly.getWorldNormals();
  const incVerts = incPoly.getWorldVertices();
  let incIndex = 0;
  let minDot = Infinity;
  for (let i = 0; i < incNormals.length; i++) {
    const d = refNormal.dot(incNormals[i]);
    if (d < minDot) {
      minDot = d;
      incIndex = i;
    }
  }
  let incidentEdge = [incVerts[incIndex], incVerts[(incIndex + 1) % incVerts.length]];

  const tangent = v2.sub(v1).normalize();

  incidentEdge = clipSegment(incidentEdge, tangent.negate(), -tangent.dot(v1));
  if (incidentEdge.length < 2) return null;
  incidentEdge = clipSegment(incidentEdge, tangent, tangent.dot(v2));
  if (incidentEdge.length < 2) return null;

  const contacts = [];
  for (const p of incidentEdge) {
    const separation = refNormal.dot(p.sub(v1));
    if (separation <= 0) {
      contacts.push({ point: p, penetration: -separation });
    }
  }
  if (contacts.length === 0) return null;

  const normal = flip ? refNormal.negate() : refNormal;
  return { bodyA: a, bodyB: b, normal, contacts };
}
