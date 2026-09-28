import type { AABB } from '../math/AABB';
import type { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import { ConvexPolyhedron } from './ConvexPolyhedron';
import { type MassProperties, Shape, type ShapeRayHit, ShapeType, raycastPolyhedron } from './Shape';

/** 长方体（半边长） */
export class BoxShape extends Shape {
  readonly type = ShapeType.Box;
  readonly radius = 0;
  readonly halfExtents: Vec3;
  override readonly hull: ConvexPolyhedron;

  constructor(halfExtents: Readonly<Vec3>) {
    super();
    if (!(halfExtents.x > 0 && halfExtents.y > 0 && halfExtents.z > 0)) {
      throw new Error('BoxShape: halfExtents 各分量必须为正数');
    }
    this.halfExtents = new Vec3().copy(halfExtents);
    this.hull = ConvexPolyhedron.fromBox(halfExtents.x, halfExtents.y, halfExtents.z);
  }

  getCorePoints(): readonly Vec3[] {
    return this.hull.vertices;
  }

  computeAABB(transform: Readonly<Transform>, out: AABB): AABB {
    // 旋转后的半边长：e_i = Σ_j |R_ij| h_j
    const q = transform.rotation;
    const h = this.halfExtents;
    q.rotate(tmpX.set(h.x, 0, 0), tmpX);
    q.rotate(tmpY.set(0, h.y, 0), tmpY);
    q.rotate(tmpZ.set(0, 0, h.z), tmpZ);
    const ex = Math.abs(tmpX.x) + Math.abs(tmpY.x) + Math.abs(tmpZ.x);
    const ey = Math.abs(tmpX.y) + Math.abs(tmpY.y) + Math.abs(tmpZ.y);
    const ez = Math.abs(tmpX.z) + Math.abs(tmpY.z) + Math.abs(tmpZ.z);
    const p = transform.position;
    return out.set(p.x - ex, p.y - ey, p.z - ez, p.x + ex, p.y + ey, p.z + ez);
  }

  getVolume(): number {
    const h = this.halfExtents;
    return 8 * h.x * h.y * h.z;
  }

  computeMass(density: number, out: MassProperties): MassProperties {
    const h = this.halfExtents;
    const m = density * this.getVolume();
    const x2 = h.x * h.x,
      y2 = h.y * h.y,
      z2 = h.z * h.z;
    out.mass = m;
    out.center.setZero();
    out.inertia.setDiagonal((m * (y2 + z2)) / 3, (m * (x2 + z2)) / 3, (m * (x2 + y2)) / 3);
    return out;
  }

  raycast(origin: Readonly<Vec3>, dir: Readonly<Vec3>, maxT: number, out: ShapeRayHit): boolean {
    return raycastPolyhedron(this.hull, origin, dir, maxT, out);
  }
}

const tmpX = new Vec3();
const tmpY = new Vec3();
const tmpZ = new Vec3();
