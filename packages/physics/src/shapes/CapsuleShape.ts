import type { AABB } from '../math/AABB';
import type { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import { type MassProperties, Shape, type ShapeRayHit, ShapeType } from './Shape';

/** 胶囊体：沿局部 Y 轴的线段 [-halfHeight, +halfHeight] 加半径 */
export class CapsuleShape extends Shape {
  readonly type = ShapeType.Capsule;
  readonly radius: number;
  readonly halfHeight: number;
  private readonly core: Vec3[];

  constructor(radius: number, halfHeight: number) {
    super();
    if (!(radius > 0)) throw new Error('CapsuleShape: radius 必须为正数');
    if (!(halfHeight >= 0)) throw new Error('CapsuleShape: halfHeight 不能为负');
    this.radius = radius;
    this.halfHeight = halfHeight;
    this.core = [new Vec3(0, -halfHeight, 0), new Vec3(0, halfHeight, 0)];
  }

  getCorePoints(): readonly Vec3[] {
    return this.core;
  }

  computeAABB(transform: Readonly<Transform>, out: AABB): AABB {
    transform.transformPoint(this.core[0]!, tmpA);
    transform.transformPoint(this.core[1]!, tmpB);
    const r = this.radius;
    return out.set(
      Math.min(tmpA.x, tmpB.x) - r,
      Math.min(tmpA.y, tmpB.y) - r,
      Math.min(tmpA.z, tmpB.z) - r,
      Math.max(tmpA.x, tmpB.x) + r,
      Math.max(tmpA.y, tmpB.y) + r,
      Math.max(tmpA.z, tmpB.z) + r,
    );
  }

  getVolume(): number {
    const r = this.radius;
    return Math.PI * r * r * (2 * this.halfHeight) + (4 / 3) * Math.PI * r * r * r;
  }

  computeMass(density: number, out: MassProperties): MassProperties {
    const r = this.radius;
    const h = this.halfHeight;
    const mc = density * Math.PI * r * r * 2 * h; // 圆柱部分
    const ms = density * (4 / 3) * Math.PI * r * r * r; // 两个半球
    const iy = mc * r * r * 0.5 + ms * 0.4 * r * r;
    const ix = (mc * (3 * r * r + 4 * h * h)) / 12 + ms * (0.4 * r * r + h * h + 0.75 * h * r);
    out.mass = mc + ms;
    out.center.setZero();
    out.inertia.setDiagonal(ix, iy, ix);
    return out;
  }

  raycast(origin: Readonly<Vec3>, dir: Readonly<Vec3>, maxT: number, out: ShapeRayHit): boolean {
    const r = this.radius;
    const h = this.halfHeight;
    // 起点在内部则不命中
    const cy = Math.max(-h, Math.min(h, origin.y));
    const dx0 = origin.x,
      dy0 = origin.y - cy,
      dz0 = origin.z;
    if (dx0 * dx0 + dy0 * dy0 + dz0 * dz0 <= r * r) return false;

    let bestT = Infinity;
    // 圆柱侧面
    const a = dir.x * dir.x + dir.z * dir.z;
    if (a > 1e-12) {
      const b = origin.x * dir.x + origin.z * dir.z;
      const c = origin.x * origin.x + origin.z * origin.z - r * r;
      const disc = b * b - a * c;
      if (disc >= 0) {
        const t = (-b - Math.sqrt(disc)) / a;
        const y = origin.y + t * dir.y;
        if (t >= 0 && y >= -h && y <= h) {
          bestT = t;
          out.normal.set(origin.x + t * dir.x, 0, origin.z + t * dir.z).normalize();
        }
      }
    }
    // 两端半球
    for (let s = -1; s <= 1; s += 2) {
      const ox = origin.x,
        oy = origin.y - s * h,
        oz = origin.z;
      const c = ox * ox + oy * oy + oz * oz - r * r;
      const aa = dir.lengthSq();
      const b = ox * dir.x + oy * dir.y + oz * dir.z;
      const disc = b * b - aa * c;
      if (disc < 0 || aa === 0) continue;
      const t = (-b - Math.sqrt(disc)) / aa;
      if (t >= 0 && t < bestT) {
        const py = oy + t * dir.y;
        // 只接受半球外侧部分
        if (s * py >= 0) {
          bestT = t;
          out.normal.set(ox + t * dir.x, py, oz + t * dir.z).normalize();
        }
      }
    }
    if (bestT > maxT) return false;
    out.t = bestT;
    return true;
  }
}

const tmpA = new Vec3();
const tmpB = new Vec3();
