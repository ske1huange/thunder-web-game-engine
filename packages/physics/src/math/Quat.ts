import { Vec3 } from './Vec3';

/** 单位四元数（x, y, z, w），用于表示旋转。乘法采用 Hamilton 约定。 */
export class Quat {
  x: number;
  y: number;
  z: number;
  w: number;

  constructor(x = 0, y = 0, z = 0, w = 1) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }

  static readonly IDENTITY: Readonly<Quat> = Object.freeze(new Quat());

  set(x: number, y: number, z: number, w: number): this {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
    return this;
  }

  identity(): this {
    return this.set(0, 0, 0, 1);
  }

  copy(q: Readonly<Quat>): this {
    this.x = q.x;
    this.y = q.y;
    this.z = q.z;
    this.w = q.w;
    return this;
  }

  clone(): Quat {
    return new Quat(this.x, this.y, this.z, this.w);
  }

  /** 由单位轴与弧度构造 */
  setFromAxisAngle(axis: Readonly<Vec3>, angle: number): this {
    const half = angle * 0.5;
    const s = Math.sin(half);
    this.x = axis.x * s;
    this.y = axis.y * s;
    this.z = axis.z * s;
    this.w = Math.cos(half);
    return this;
  }

  /** 由欧拉角构造（旋转顺序 XYZ，与 three.js 默认一致） */
  setFromEuler(x: number, y: number, z: number): this {
    const c1 = Math.cos(x / 2),
      c2 = Math.cos(y / 2),
      c3 = Math.cos(z / 2);
    const s1 = Math.sin(x / 2),
      s2 = Math.sin(y / 2),
      s3 = Math.sin(z / 2);
    this.x = s1 * c2 * c3 + c1 * s2 * s3;
    this.y = c1 * s2 * c3 - s1 * c2 * s3;
    this.z = c1 * c2 * s3 + s1 * s2 * c3;
    this.w = c1 * c2 * c3 - s1 * s2 * s3;
    return this;
  }

  /** 构造把单位向量 from 旋转到单位向量 to 的最短旋转 */
  setFromUnitVectors(from: Readonly<Vec3>, to: Readonly<Vec3>): this {
    let r = from.dot(to) + 1;
    if (r < 1e-8) {
      // 反向：任取一条垂直轴旋转 180°
      r = 0;
      if (Math.abs(from.x) > Math.abs(from.z)) {
        this.x = -from.y;
        this.y = from.x;
        this.z = 0;
      } else {
        this.x = 0;
        this.y = -from.z;
        this.z = from.y;
      }
    } else {
      this.x = from.y * to.z - from.z * to.y;
      this.y = from.z * to.x - from.x * to.z;
      this.z = from.x * to.y - from.y * to.x;
    }
    this.w = r;
    return this.normalize();
  }

  lengthSq(): number {
    return this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w;
  }

  length(): number {
    return Math.sqrt(this.lengthSq());
  }

  normalize(): this {
    const len = this.length();
    if (len < 1e-12) return this.identity();
    const inv = 1 / len;
    this.x *= inv;
    this.y *= inv;
    this.z *= inv;
    this.w *= inv;
    return this;
  }

  /** 共轭（单位四元数的逆） */
  conjugate(): this {
    this.x = -this.x;
    this.y = -this.y;
    this.z = -this.z;
    return this;
  }

  dot(q: Readonly<Quat>): number {
    return this.x * q.x + this.y * q.y + this.z * q.z + this.w * q.w;
  }

  /** this = this * q */
  multiply(q: Readonly<Quat>): this {
    return this.multiplyQuats(this, q);
  }

  /** this = q * this */
  premultiply(q: Readonly<Quat>): this {
    return this.multiplyQuats(q, this);
  }

  /** this = a * b */
  multiplyQuats(a: Readonly<Quat>, b: Readonly<Quat>): this {
    const ax = a.x,
      ay = a.y,
      az = a.z,
      aw = a.w;
    const bx = b.x,
      by = b.y,
      bz = b.z,
      bw = b.w;
    this.x = ax * bw + aw * bx + ay * bz - az * by;
    this.y = ay * bw + aw * by + az * bx - ax * bz;
    this.z = az * bw + aw * bz + ax * by - ay * bx;
    this.w = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }

  /** this = conj(a) * b，即 a⁻¹ * b */
  multiplyConjugateA(a: Readonly<Quat>, b: Readonly<Quat>): this {
    const ax = -a.x,
      ay = -a.y,
      az = -a.z,
      aw = a.w;
    const bx = b.x,
      by = b.y,
      bz = b.z,
      bw = b.w;
    this.x = ax * bw + aw * bx + ay * bz - az * by;
    this.y = ay * bw + aw * by + az * bx - ax * bz;
    this.z = az * bw + aw * bz + ax * by - ay * bx;
    this.w = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }

  /** 用该旋转变换向量 v，结果写入 out（out 可与 v 相同） */
  rotate(v: Readonly<Vec3>, out: Vec3): Vec3 {
    // t = 2 * cross(q.xyz, v); v' = v + w * t + cross(q.xyz, t)
    const qx = this.x,
      qy = this.y,
      qz = this.z,
      qw = this.w;
    const vx = v.x,
      vy = v.y,
      vz = v.z;
    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);
    out.x = vx + qw * tx + (qy * tz - qz * ty);
    out.y = vy + qw * ty + (qz * tx - qx * tz);
    out.z = vz + qw * tz + (qx * ty - qy * tx);
    return out;
  }

  /** 用该旋转的逆变换向量 v */
  invRotate(v: Readonly<Vec3>, out: Vec3): Vec3 {
    const qx = -this.x,
      qy = -this.y,
      qz = -this.z,
      qw = this.w;
    const vx = v.x,
      vy = v.y,
      vz = v.z;
    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);
    out.x = vx + qw * tx + (qy * tz - qz * ty);
    out.y = vy + qw * ty + (qz * tx - qx * tz);
    out.z = vz + qw * tz + (qx * ty - qy * tx);
    return out;
  }

  /**
   * 按世界系角速度 w 积分 dt 时间：q ← normalize(q + 0.5 * dt * (w, 0) * q)
   */
  integrate(w: Readonly<Vec3>, dt: number): this {
    const hx = 0.5 * dt * w.x,
      hy = 0.5 * dt * w.y,
      hz = 0.5 * dt * w.z;
    const qx = this.x,
      qy = this.y,
      qz = this.z,
      qw = this.w;
    this.x = qx + (hx * qw + hy * qz - hz * qy);
    this.y = qy + (hy * qw + hz * qx - hx * qz);
    this.z = qz + (hz * qw + hx * qy - hy * qx);
    this.w = qw - (hx * qx + hy * qy + hz * qz);
    return this.normalize();
  }

  /**
   * 转换为旋转向量（轴 * 角度，角度在 [-PI, PI]），常用于约束误差计算。
   */
  toRotationVector(out: Vec3): Vec3 {
    let x = this.x,
      y = this.y,
      z = this.z,
      w = this.w;
    if (w < 0) {
      x = -x;
      y = -y;
      z = -z;
      w = -w;
    }
    const s = Math.sqrt(x * x + y * y + z * z);
    if (s < 1e-9) {
      // 小角度近似：angle ≈ 2 * s
      return out.set(2 * x, 2 * y, 2 * z);
    }
    const angle = 2 * Math.atan2(s, w);
    const k = angle / s;
    return out.set(x * k, y * k, z * k);
  }

  /** 旋转角度（弧度，[0, PI]） */
  getAngle(): number {
    const w = Math.min(1, Math.abs(this.w));
    return 2 * Math.acos(w);
  }

  /** 球面线性插值 */
  slerpQuats(a: Readonly<Quat>, b: Readonly<Quat>, t: number): this {
    let bx = b.x,
      by = b.y,
      bz = b.z,
      bw = b.w;
    let cos = a.x * bx + a.y * by + a.z * bz + a.w * bw;
    if (cos < 0) {
      cos = -cos;
      bx = -bx;
      by = -by;
      bz = -bz;
      bw = -bw;
    }
    let k0: number, k1: number;
    if (cos > 0.9995) {
      k0 = 1 - t;
      k1 = t;
    } else {
      const theta = Math.acos(cos);
      const sin = Math.sin(theta);
      k0 = Math.sin((1 - t) * theta) / sin;
      k1 = Math.sin(t * theta) / sin;
    }
    this.x = a.x * k0 + bx * k1;
    this.y = a.y * k0 + by * k1;
    this.z = a.z * k0 + bz * k1;
    this.w = a.w * k0 + bw * k1;
    return this.normalize();
  }

  equals(q: Readonly<Quat>, eps = 0): boolean {
    return (
      Math.abs(this.x - q.x) <= eps &&
      Math.abs(this.y - q.y) <= eps &&
      Math.abs(this.z - q.z) <= eps &&
      Math.abs(this.w - q.w) <= eps
    );
  }

  toString(): string {
    return `Quat(${this.x}, ${this.y}, ${this.z}, ${this.w})`;
  }
}
