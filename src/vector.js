export class Vec2 {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }

  static zero() {
    return new Vec2(0, 0);
  }

  add(v) {
    return new Vec2(this.x + v.x, this.y + v.y);
  }

  sub(v) {
    return new Vec2(this.x - v.x, this.y - v.y);
  }

  scale(s) {
    return new Vec2(this.x * s, this.y * s);
  }

  dot(v) {
    return this.x * v.x + this.y * v.y;
  }

  // 2D cross product of two vectors returns a scalar (z component of the 3D cross)
  cross(v) {
    return this.x * v.y - this.y * v.x;
  }

  length() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  }

  lengthSq() {
    return this.x * this.x + this.y * this.y;
  }

  normalize() {
    const len = this.length();
    if (len < 1e-12) return new Vec2(0, 0);
    return new Vec2(this.x / len, this.y / len);
  }

  perp() {
    return new Vec2(-this.y, this.x);
  }

  negate() {
    return new Vec2(-this.x, -this.y);
  }

  clone() {
    return new Vec2(this.x, this.y);
  }
}
