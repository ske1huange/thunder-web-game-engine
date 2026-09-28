/**
 * 三维向量。
 *
 * 与 three.js 一样，实例方法会修改自身并返回 `this`，便于链式调用；
 * 在热循环里请复用临时对象，避免频繁 `new` 造成 GC 压力。
 */
export class Vec3 {
  x: number;
  y: number;
  z: number;

  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  static readonly ZERO: Readonly<Vec3> = Object.freeze(new Vec3(0, 0, 0));
  static readonly UNIT_X: Readonly<Vec3> = Object.freeze(new Vec3(1, 0, 0));
  static readonly UNIT_Y: Readonly<Vec3> = Object.freeze(new Vec3(0, 1, 0));
  static readonly UNIT_Z: Readonly<Vec3> = Object.freeze(new Vec3(0, 0, 1));

  set(x: number, y: number, z: number): this {
    this.x = x;
    this.y = y;
    this.z = z;
    return this;
  }

  setZero(): this {
    this.x = 0;
    this.y = 0;
    this.z = 0;
    return this;
  }

  setScalar(s: number): this {
    this.x = s;
    this.y = s;
    this.z = s;
    return this;
  }

  copy(v: Readonly<Vec3>): this {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
    return this;
  }

  clone(): Vec3 {
    return new Vec3(this.x, this.y, this.z);
  }

  add(v: Readonly<Vec3>): this {
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
    return this;
  }

  addVectors(a: Readonly<Vec3>, b: Readonly<Vec3>): this {
    this.x = a.x + b.x;
    this.y = a.y + b.y;
    this.z = a.z + b.z;
    return this;
  }

  /** this += v * s */
  addScaled(v: Readonly<Vec3>, s: number): this {
    this.x += v.x * s;
    this.y += v.y * s;
    this.z += v.z * s;
    return this;
  }

  sub(v: Readonly<Vec3>): this {
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
    return this;
  }

  subVectors(a: Readonly<Vec3>, b: Readonly<Vec3>): this {
    this.x = a.x - b.x;
    this.y = a.y - b.y;
    this.z = a.z - b.z;
    return this;
  }

  scale(s: number): this {
    this.x *= s;
    this.y *= s;
    this.z *= s;
    return this;
  }

  /** 逐分量相乘 */
  multiply(v: Readonly<Vec3>): this {
    this.x *= v.x;
    this.y *= v.y;
    this.z *= v.z;
    return this;
  }

  negate(): this {
    this.x = -this.x;
    this.y = -this.y;
    this.z = -this.z;
    return this;
  }

  dot(v: Readonly<Vec3>): number {
    return this.x * v.x + this.y * v.y + this.z * v.z;
  }

  cross(v: Readonly<Vec3>): this {
    return this.crossVectors(this, v);
  }

  crossVectors(a: Readonly<Vec3>, b: Readonly<Vec3>): this {
    const ax = a.x,
      ay = a.y,
      az = a.z;
    const bx = b.x,
      by = b.y,
      bz = b.z;
    this.x = ay * bz - az * by;
    this.y = az * bx - ax * bz;
    this.z = ax * by - ay * bx;
    return this;
  }

  lengthSq(): number {
    return this.x * this.x + this.y * this.y + this.z * this.z;
  }

  length(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
  }

  distanceTo(v: Readonly<Vec3>): number {
    return Math.sqrt(this.distanceToSq(v));
  }

  distanceToSq(v: Readonly<Vec3>): number {
    const dx = this.x - v.x,
      dy = this.y - v.y,
      dz = this.z - v.z;
    return dx * dx + dy * dy + dz * dz;
  }

  /** 归一化，返回原长度；长度过小时保持为零向量 */
  normalize(): number {
    const len = this.length();
    if (len < 1e-12) {
      this.x = 0;
      this.y = 0;
      this.z = 0;
      return 0;
    }
    const inv = 1 / len;
    this.x *= inv;
    this.y *= inv;
    this.z *= inv;
    return len;
  }

  /** 线性插值：this = a + (b - a) * t */
  lerpVectors(a: Readonly<Vec3>, b: Readonly<Vec3>, t: number): this {
    this.x = a.x + (b.x - a.x) * t;
    this.y = a.y + (b.y - a.y) * t;
    this.z = a.z + (b.z - a.z) * t;
    return this;
  }

  min(v: Readonly<Vec3>): this {
    this.x = Math.min(this.x, v.x);
    this.y = Math.min(this.y, v.y);
    this.z = Math.min(this.z, v.z);
    return this;
  }

  max(v: Readonly<Vec3>): this {
    this.x = Math.max(this.x, v.x);
    this.y = Math.max(this.y, v.y);
    this.z = Math.max(this.z, v.z);
    return this;
  }

  abs(): this {
    this.x = Math.abs(this.x);
    this.y = Math.abs(this.y);
    this.z = Math.abs(this.z);
    return this;
  }

  equals(v: Readonly<Vec3>, eps = 0): boolean {
    return (
      Math.abs(this.x - v.x) <= eps && Math.abs(this.y - v.y) <= eps && Math.abs(this.z - v.z) <= eps
    );
  }

  isFinite(): boolean {
    return Number.isFinite(this.x) && Number.isFinite(this.y) && Number.isFinite(this.z);
  }

  /** 生成一个与自身垂直的单位向量（自身需为单位向量） */
  perpendicular(out: Vec3): Vec3 {
    // Erin Catto 的做法：选择绝对值最小的分量方向构造垂线
    if (Math.abs(this.x) >= 0.57735) {
      out.set(this.y, -this.x, 0);
    } else {
      out.set(0, this.z, -this.y);
    }
    out.normalize();
    return out;
  }

  getComponent(index: number): number {
    return index === 0 ? this.x : index === 1 ? this.y : this.z;
  }

  setComponent(index: number, value: number): this {
    if (index === 0) this.x = value;
    else if (index === 1) this.y = value;
    else this.z = value;
    return this;
  }

  toArray(out: number[] = [], offset = 0): number[] {
    out[offset] = this.x;
    out[offset + 1] = this.y;
    out[offset + 2] = this.z;
    return out;
  }

  fromArray(arr: ArrayLike<number>, offset = 0): this {
    this.x = arr[offset]!;
    this.y = arr[offset + 1]!;
    this.z = arr[offset + 2]!;
    return this;
  }

  toString(): string {
    return `Vec3(${this.x}, ${this.y}, ${this.z})`;
  }
}

/** 构造正交基：给定单位向量 n，输出与之两两正交的单位向量 t1、t2 */
export function computeBasis(n: Readonly<Vec3>, t1: Vec3, t2: Vec3): void {
  if (Math.abs(n.x) >= 0.57735) {
    t1.set(n.y, -n.x, 0);
  } else {
    t1.set(0, n.z, -n.y);
  }
  t1.normalize();
  t2.crossVectors(n, t1);
}
