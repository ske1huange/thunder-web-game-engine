import type { AABB } from '../math/AABB';
import type { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import { type MassProperties, Shape, type ShapeRayHit, ShapeType } from './Shape';

/** 球体（中心位于局部原点） */
export class SphereShape extends Shape {
  readonly type = ShapeType.Sphere;
  readonly radius: number;
  private readonly core = [new Vec3()];

  constructor(radius: number) {
    super();
    if (!(radius > 0)) throw new Error('SphereShape: radius 必须为正数');
    this.radius = radius;
  }

  getCorePoints(): readonly Vec3[] {
    return this.core;
  }

  computeAABB(transform: Readonly<Transform>, out: AABB): AABB {
    const p = transform.position;
    const r = this.radius;
    return out.set(p.x - r, p.y - r, p.z - r, p.x + r, p.y + r, p.z + r);
  }

  getVolume(): number {
    return (4 / 3) * Math.PI * this.radius ** 3;
  }

  computeMass(density: number, out: MassProperties): MassProperties {
    const m = density * this.getVolume();
    const i = 0.4 * m * this.radius * this.radius;
    out.mass = m;
    out.center.setZero();
    out.inertia.setDiagonal(i, i, i);
    return out;
  }

  raycast(origin: Readonly<Vec3>, dir: Readonly<Vec3>, maxT: number, out: ShapeRayHit): boolean {
    const r = this.radius;
    const c = origin.lengthSq() - r * r;
    if (c <= 0) return false;
    const a = dir.lengthSq();
    const b = origin.dot(dir);
    if (b >= 0 || a === 0) return false;
    const disc = b * b - a * c;
    if (disc < 0) return false;
    const t = (-b - Math.sqrt(disc)) / a;
    if (t < 0 || t > maxT) return false;
    out.t = t;
    out.normal.copy(origin).addScaled(dir, t).normalize();
    return true;
  }
}
