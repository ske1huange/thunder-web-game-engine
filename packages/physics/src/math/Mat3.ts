import type { Quat } from './Quat';
import type { Vec3 } from './Vec3';

/**
 * 3x3 矩阵（行主序存储：mRC 表示第 R 行第 C 列）。
 * 主要用于惯性张量与约束的有效质量。
 */
export class Mat3 {
  m00: number;
  m01: number;
  m02: number;
  m10: number;
  m11: number;
  m12: number;
  m20: number;
  m21: number;
  m22: number;

  constructor(
    m00 = 1,
    m01 = 0,
    m02 = 0,
    m10 = 0,
    m11 = 1,
    m12 = 0,
    m20 = 0,
    m21 = 0,
    m22 = 1,
  ) {
    this.m00 = m00;
    this.m01 = m01;
    this.m02 = m02;
    this.m10 = m10;
    this.m11 = m11;
    this.m12 = m12;
    this.m20 = m20;
    this.m21 = m21;
    this.m22 = m22;
  }

  set(
    m00: number,
    m01: number,
    m02: number,
    m10: number,
    m11: number,
    m12: number,
    m20: number,
    m21: number,
    m22: number,
  ): this {
    this.m00 = m00;
    this.m01 = m01;
    this.m02 = m02;
    this.m10 = m10;
    this.m11 = m11;
    this.m12 = m12;
    this.m20 = m20;
    this.m21 = m21;
    this.m22 = m22;
    return this;
  }

  identity(): this {
    return this.set(1, 0, 0, 0, 1, 0, 0, 0, 1);
  }

  setZero(): this {
    return this.set(0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  setDiagonal(x: number, y: number, z: number): this {
    return this.set(x, 0, 0, 0, y, 0, 0, 0, z);
  }

  copy(m: Readonly<Mat3>): this {
    return this.set(m.m00, m.m01, m.m02, m.m10, m.m11, m.m12, m.m20, m.m21, m.m22);
  }

  clone(): Mat3 {
    return new Mat3().copy(this);
  }

  /** 由单位四元数构造旋转矩阵 */
  setFromQuat(q: Readonly<Quat>): this {
    const x = q.x,
      y = q.y,
      z = q.z,
      w = q.w;
    const x2 = x + x,
      y2 = y + y,
      z2 = z + z;
    const xx = x * x2,
      xy = x * y2,
      xz = x * z2;
    const yy = y * y2,
      yz = y * z2,
      zz = z * z2;
    const wx = w * x2,
      wy = w * y2,
      wz = w * z2;
    return this.set(
      1 - (yy + zz),
      xy - wz,
      xz + wy,
      xy + wz,
      1 - (xx + zz),
      yz - wx,
      xz - wy,
      yz + wx,
      1 - (xx + yy),
    );
  }

  /** 叉乘矩阵 [v]×，满足 [v]× u = v × u */
  setSkew(v: Readonly<Vec3>): this {
    return this.set(0, -v.z, v.y, v.z, 0, -v.x, -v.y, v.x, 0);
  }

  add(m: Readonly<Mat3>): this {
    this.m00 += m.m00;
    this.m01 += m.m01;
    this.m02 += m.m02;
    this.m10 += m.m10;
    this.m11 += m.m11;
    this.m12 += m.m12;
    this.m20 += m.m20;
    this.m21 += m.m21;
    this.m22 += m.m22;
    return this;
  }

  scale(s: number): this {
    this.m00 *= s;
    this.m01 *= s;
    this.m02 *= s;
    this.m10 *= s;
    this.m11 *= s;
    this.m12 *= s;
    this.m20 *= s;
    this.m21 *= s;
    this.m22 *= s;
    return this;
  }

  /** this = a * b */
  multiplyMatrices(a: Readonly<Mat3>, b: Readonly<Mat3>): this {
    const a00 = a.m00,
      a01 = a.m01,
      a02 = a.m02,
      a10 = a.m10,
      a11 = a.m11,
      a12 = a.m12,
      a20 = a.m20,
      a21 = a.m21,
      a22 = a.m22;
    const b00 = b.m00,
      b01 = b.m01,
      b02 = b.m02,
      b10 = b.m10,
      b11 = b.m11,
      b12 = b.m12,
      b20 = b.m20,
      b21 = b.m21,
      b22 = b.m22;
    return this.set(
      a00 * b00 + a01 * b10 + a02 * b20,
      a00 * b01 + a01 * b11 + a02 * b21,
      a00 * b02 + a01 * b12 + a02 * b22,
      a10 * b00 + a11 * b10 + a12 * b20,
      a10 * b01 + a11 * b11 + a12 * b21,
      a10 * b02 + a11 * b12 + a12 * b22,
      a20 * b00 + a21 * b10 + a22 * b20,
      a20 * b01 + a21 * b11 + a22 * b21,
      a20 * b02 + a21 * b12 + a22 * b22,
    );
  }

  transpose(): this {
    let t = this.m01;
    this.m01 = this.m10;
    this.m10 = t;
    t = this.m02;
    this.m02 = this.m20;
    this.m20 = t;
    t = this.m12;
    this.m12 = this.m21;
    this.m21 = t;
    return this;
  }

  determinant(): number {
    return (
      this.m00 * (this.m11 * this.m22 - this.m12 * this.m21) -
      this.m01 * (this.m10 * this.m22 - this.m12 * this.m20) +
      this.m02 * (this.m10 * this.m21 - this.m11 * this.m20)
    );
  }

  /** 求逆；矩阵奇异时置零并返回 false */
  invert(): boolean {
    const a00 = this.m00,
      a01 = this.m01,
      a02 = this.m02,
      a10 = this.m10,
      a11 = this.m11,
      a12 = this.m12,
      a20 = this.m20,
      a21 = this.m21,
      a22 = this.m22;
    const c00 = a11 * a22 - a12 * a21;
    const c01 = a12 * a20 - a10 * a22;
    const c02 = a10 * a21 - a11 * a20;
    const det = a00 * c00 + a01 * c01 + a02 * c02;
    if (Math.abs(det) < 1e-18) {
      this.setZero();
      return false;
    }
    const inv = 1 / det;
    this.set(
      c00 * inv,
      (a02 * a21 - a01 * a22) * inv,
      (a01 * a12 - a02 * a11) * inv,
      c01 * inv,
      (a00 * a22 - a02 * a20) * inv,
      (a02 * a10 - a00 * a12) * inv,
      c02 * inv,
      (a01 * a20 - a00 * a21) * inv,
      (a00 * a11 - a01 * a10) * inv,
    );
    return true;
  }

  /** out = M * v（out 可与 v 相同） */
  transformVector(v: Readonly<Vec3>, out: Vec3): Vec3 {
    const x = v.x,
      y = v.y,
      z = v.z;
    out.x = this.m00 * x + this.m01 * y + this.m02 * z;
    out.y = this.m10 * x + this.m11 * y + this.m12 * z;
    out.z = this.m20 * x + this.m21 * y + this.m22 * z;
    return out;
  }

  /** out = Mᵀ * v */
  transposeTransformVector(v: Readonly<Vec3>, out: Vec3): Vec3 {
    const x = v.x,
      y = v.y,
      z = v.z;
    out.x = this.m00 * x + this.m10 * y + this.m20 * z;
    out.y = this.m01 * x + this.m11 * y + this.m21 * z;
    out.z = this.m02 * x + this.m12 * y + this.m22 * z;
    return out;
  }

  /** 二次型 vᵀ M v */
  quadraticForm(v: Readonly<Vec3>): number {
    const x = v.x,
      y = v.y,
      z = v.z;
    return (
      x * (this.m00 * x + this.m01 * y + this.m02 * z) +
      y * (this.m10 * x + this.m11 * y + this.m12 * z) +
      z * (this.m20 * x + this.m21 * y + this.m22 * z)
    );
  }

  /** this = R * I * Rᵀ（R 由四元数给出），用于把局部惯性张量变换到世界系 */
  setRotated(q: Readonly<Quat>, inertia: Readonly<Mat3>): this {
    const r = tmpRot.setFromQuat(q);
    tmpMat.multiplyMatrices(r, inertia);
    r.transpose();
    return this.multiplyMatrices(tmpMat, r);
  }

  /** 求解 M x = b（M 须可逆），结果写入 out */
  solve(b: Readonly<Vec3>, out: Vec3): Vec3 {
    const inv = tmpSolve.copy(this);
    if (!inv.invert()) return out.set(0, 0, 0);
    return inv.transformVector(b, out);
  }

  equals(m: Readonly<Mat3>, eps = 0): boolean {
    return (
      Math.abs(this.m00 - m.m00) <= eps &&
      Math.abs(this.m01 - m.m01) <= eps &&
      Math.abs(this.m02 - m.m02) <= eps &&
      Math.abs(this.m10 - m.m10) <= eps &&
      Math.abs(this.m11 - m.m11) <= eps &&
      Math.abs(this.m12 - m.m12) <= eps &&
      Math.abs(this.m20 - m.m20) <= eps &&
      Math.abs(this.m21 - m.m21) <= eps &&
      Math.abs(this.m22 - m.m22) <= eps
    );
  }
}

const tmpRot = new Mat3();
const tmpMat = new Mat3();
const tmpSolve = new Mat3();
