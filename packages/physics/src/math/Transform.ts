import { Quat } from './Quat';
import { Vec3 } from './Vec3';

/** 刚体变换：先旋转再平移（p' = R p + t） */
export class Transform {
  readonly position: Vec3;
  readonly rotation: Quat;

  constructor(position?: Readonly<Vec3>, rotation?: Readonly<Quat>) {
    this.position = position ? new Vec3().copy(position) : new Vec3();
    this.rotation = rotation ? new Quat().copy(rotation) : new Quat();
  }

  identity(): this {
    this.position.setZero();
    this.rotation.identity();
    return this;
  }

  copy(t: Readonly<Transform>): this {
    this.position.copy(t.position);
    this.rotation.copy(t.rotation);
    return this;
  }

  clone(): Transform {
    return new Transform(this.position, this.rotation);
  }

  /** 局部点 → 世界点 */
  transformPoint(p: Readonly<Vec3>, out: Vec3): Vec3 {
    this.rotation.rotate(p, out);
    return out.add(this.position);
  }

  /** 世界点 → 局部点 */
  inverseTransformPoint(p: Readonly<Vec3>, out: Vec3): Vec3 {
    out.subVectors(p, this.position);
    return this.rotation.invRotate(out, out);
  }

  /** 局部方向 → 世界方向 */
  transformVector(v: Readonly<Vec3>, out: Vec3): Vec3 {
    return this.rotation.rotate(v, out);
  }

  /** 世界方向 → 局部方向 */
  inverseTransformVector(v: Readonly<Vec3>, out: Vec3): Vec3 {
    return this.rotation.invRotate(v, out);
  }

  /** this = a * b（先应用 b，再应用 a） */
  multiplyTransforms(a: Readonly<Transform>, b: Readonly<Transform>): this {
    // 注意 this 可能与 a 或 b 相同，需先算平移
    const px = b.position.x,
      py = b.position.y,
      pz = b.position.z;
    tmpV.set(px, py, pz);
    a.rotation.rotate(tmpV, tmpV);
    tmpV.add(a.position);
    this.rotation.multiplyQuats(a.rotation, b.rotation);
    this.position.copy(tmpV);
    return this;
  }

  /** this = a⁻¹ * b（b 在 a 局部系下的表示） */
  multiplyInverseA(a: Readonly<Transform>, b: Readonly<Transform>): this {
    tmpV.subVectors(b.position, a.position);
    a.rotation.invRotate(tmpV, tmpV);
    this.rotation.multiplyConjugateA(a.rotation, b.rotation);
    this.position.copy(tmpV);
    return this;
  }
}

const tmpV = new Vec3();
