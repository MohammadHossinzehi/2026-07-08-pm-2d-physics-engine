import { Vec2 } from './vector.js';

// Fraction of penetration corrected per step (Baumgarte-style stabilization).
const CORRECTION_PERCENT = 0.2;
// Penetration allowed before positional correction kicks in, to avoid jitter.
const PENETRATION_SLOP = 0.02;
// Positional correction is capped per contact per step. Without this, a
// single frame with an unusually deep (or momentarily miscomputed) contact
// can shove a body a huge distance in one go, which is what causes visible
// "teleporting" / explosion bugs in simple solvers.
const MAX_CORRECTION = 4;

function velocityAtPoint(body, r) {
  // v + omega x r, where omega is a scalar in 2D and (omega x r) = omega * perp(r)
  return body.velocity.add(new Vec2(-body.angularVelocity * r.y, body.angularVelocity * r.x));
}

// Applies the normal (restitution) and tangential (Coulomb friction) impulses
// for every contact point in a manifold. Meant to be called multiple times
// per step (sequential impulse iterations) to let impulses propagate through
// stacks of resting bodies. Does NOT touch position -- see correctPositions.
export function resolveVelocity(manifold) {
  const { bodyA: a, bodyB: b, normal, contacts } = manifold;
  if (a.isStatic && b.isStatic) return;

  for (const { point } of contacts) {
    resolveContactVelocity(a, b, normal, point);
  }
}

function resolveContactVelocity(a, b, normal, contact) {
  const rA = contact.sub(a.position);
  const rB = contact.sub(b.position);

  const relativeVelocity = velocityAtPoint(b, rB).sub(velocityAtPoint(a, rA));
  const velAlongNormal = relativeVelocity.dot(normal);

  if (velAlongNormal > 0) return;

  const restitution = Math.min(a.restitution, b.restitution);

  const rACrossN = rA.cross(normal);
  const rBCrossN = rB.cross(normal);
  const invMassSum =
    a.invMass + b.invMass + rACrossN * rACrossN * a.invInertia + rBCrossN * rBCrossN * b.invInertia;

  if (invMassSum <= 0) return;

  let j = -(1 + restitution) * velAlongNormal;
  j /= invMassSum;

  const impulse = normal.scale(j);
  a.applyImpulse(impulse.negate(), rA);
  b.applyImpulse(impulse, rB);

  // Coulomb friction along the tangent of the contact
  const relVelAfterNormal = velocityAtPoint(b, rB).sub(velocityAtPoint(a, rA));
  let tangent = relVelAfterNormal.sub(normal.scale(relVelAfterNormal.dot(normal)));
  tangent = tangent.normalize();

  if (tangent.lengthSq() === 0) return;

  const rACrossT = rA.cross(tangent);
  const rBCrossT = rB.cross(tangent);
  const invMassSumT =
    a.invMass + b.invMass + rACrossT * rACrossT * a.invInertia + rBCrossT * rBCrossT * b.invInertia;

  if (invMassSumT <= 0) return;

  let jt = -relVelAfterNormal.dot(tangent);
  jt /= invMassSumT;

  const mu = Math.sqrt(a.friction * a.friction + b.friction * b.friction);
  const maxFriction = j * mu;
  jt = Math.max(-maxFriction, Math.min(maxFriction, jt));

  const frictionImpulse = tangent.scale(jt);
  a.applyImpulse(frictionImpulse.negate(), rA);
  b.applyImpulse(frictionImpulse, rB);
}

// Nudges the two bodies apart along the contact normal, proportional to how
// deeply they overlap. Intended to be called exactly once per manifold per
// step (after velocity iterations are done), since it is a purely positional
// fix-up rather than a physical force -- applying it repeatedly against the
// same stale penetration value is what caused the overcorrection instability
// described above.
export function correctPositions(manifold) {
  const { bodyA: a, bodyB: b, normal, contacts } = manifold;
  const invMassSum = a.invMass + b.invMass;
  if (invMassSum <= 0) return;

  for (const { penetration } of contacts) {
    const magnitude = Math.min(
      (Math.max(penetration - PENETRATION_SLOP, 0) / invMassSum) * CORRECTION_PERCENT,
      MAX_CORRECTION,
    );
    if (magnitude <= 0) continue;
    const correction = normal.scale(magnitude);
    if (!a.isStatic) a.position = a.position.sub(correction.scale(a.invMass));
    if (!b.isStatic) b.position = b.position.add(correction.scale(b.invMass));
  }
}
