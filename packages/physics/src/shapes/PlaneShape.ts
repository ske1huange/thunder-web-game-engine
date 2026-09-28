import type { AABB } from '../math/AABB';
import type { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import { type MassProperties, Shape, type ShapeRayHit, ShapeType } from './Shape';

/** 平面形状的有效范围（包围盒尺寸），足够覆盖一般游戏场景 */
export const PLANE_EXTENT = 1e5;

/**
 * 无限大平面（半空间 n·x ≤ 0 为实体），局部法线为 +Y。
 * 只能挂在静态刚体上，常用作地面。可通过碰撞体的局部旋转/偏移调整朝向与位置。
 */
export class PlaneShape extends Shape {
  readonly type = ShapeType.Plane;
  readonly radius = 0;

  getCorePoints(): readonly Vec3[] {
    return NO_POINTS;
  }

  computeAABB(transform: Readonly<Transform>, out: AABB): AABB {
    const L = PLANE_EXTENT;
    out.set(-L, -L, -L, L, L, L);
    // 轴对齐时收紧法线方向，减少无谓的宽相配对
    const n = transform.rotation.rotate(UP, tmpN);
    const p = transform.position;
    const ax = Math.abs(n.x),
      ay = Math.abs(n.y),
      az = Math.abs(n.z);
    const aligned = 1 - 1e-9;
    if (ax > aligned) {
      if (n.x > 0) out.max.x = p.x;
      else out.min.x = p.x;
    } else if (ay > aligned) {
      if (n.y > 0) out.max.y = p.y;
      else out.min.y = p.y;
    } else if (az > aligned) {
      if (n.z > 0) out.max.z = p.z;
      else out.min.z = p.z;
    }
    return out;
  }

  getVolume(): number {
    return 0;
  }

  computeMass(_density: number, out: MassProperties): MassProperties {
    out.mass = 0;
    out.center.setZero();
    out.inertia.setZero();
    return out;
  }

  raycast(origin: Readonly<Vec3>, dir: Readonly<Vec3>, maxT: number, out: ShapeRayHit): boolean {
    if (origin.y <= 0 || dir.y >= 0) return false;
    const t = -origin.y / dir.y;
    if (t > maxT) return false;
    out.t = t;
    out.normal.set(0, 1, 0);
    return true;
  }
}

const NO_POINTS: readonly Vec3[] = [];
const UP = new Vec3(0, 1, 0);
const tmpN = new Vec3();
