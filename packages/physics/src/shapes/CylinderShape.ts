import type { AABB } from '../math/AABB';
import type { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import { ConvexPolyhedron } from './ConvexPolyhedron';
import {
  type MassProperties,
  Shape,
  type ShapeRayHit,
  ShapeType,
  aabbOfPoints,
  raycastPolyhedron,
} from './Shape';

/**
 * 圆柱体（沿局部 Y 轴）。
 *
 * 与 cannon-es 类似，碰撞几何用正 N 棱柱近似（默认 16 段），
 * 这样可以直接复用多面体的 SAT + 面裁剪，得到稳定的面接触流形；质量属性按真实圆柱计算。
 */
export class CylinderShape extends Shape {
  readonly type = ShapeType.Cylinder;
  readonly radius = 0;
  readonly cylinderRadius: number;
  readonly halfHeight: number;
  readonly segments: number;
  override readonly hull: ConvexPolyhedron;

  constructor(radius: number, halfHeight: number, segments = 16) {
    super();
    if (!(radius > 0 && halfHeight > 0)) {
      throw new Error('CylinderShape: radius 与 halfHeight 必须为正数');
    }
    if (!(segments >= 3)) throw new Error('CylinderShape: segments 至少为 3');
    this.cylinderRadius = radius;
    this.halfHeight = halfHeight;
    this.segments = segments;

    const verts: Vec3[] = [];
    for (let i = 0; i < segments; i++) {
      const a = (i / segments) * Math.PI * 2;
      verts.push(new Vec3(Math.cos(a) * radius, -halfHeight, Math.sin(a) * radius));
    }
    for (let i = 0; i < segments; i++) {
      const a = (i / segments) * Math.PI * 2;
      verts.push(new Vec3(Math.cos(a) * radius, halfHeight, Math.sin(a) * radius));
    }
    const faces: number[][] = [];
    // 顶面（从 +Y 看逆时针）与底面
    const top: number[] = [];
    const bottom: number[] = [];
    for (let i = 0; i < segments; i++) {
      top.push(segments + (segments - 1 - i));
      bottom.push(i);
    }
    faces.push(top, bottom);
    for (let i = 0; i < segments; i++) {
      const j = (i + 1) % segments;
      faces.push([i, segments + i, segments + j, j]);
    }
    this.hull = new ConvexPolyhedron(verts, faces);
  }

  getCorePoints(): readonly Vec3[] {
    return this.hull.vertices;
  }

  computeAABB(transform: Readonly<Transform>, out: AABB): AABB {
    return aabbOfPoints(this.hull.vertices, transform, 0, out);
  }

  getVolume(): number {
    return Math.PI * this.cylinderRadius ** 2 * 2 * this.halfHeight;
  }

  computeMass(density: number, out: MassProperties): MassProperties {
    const r = this.cylinderRadius;
    const h = this.halfHeight;
    const m = density * this.getVolume();
    const ix = (m * (3 * r * r + 4 * h * h)) / 12;
    out.mass = m;
    out.center.setZero();
    out.inertia.setDiagonal(ix, 0.5 * m * r * r, ix);
    return out;
  }

  raycast(origin: Readonly<Vec3>, dir: Readonly<Vec3>, maxT: number, out: ShapeRayHit): boolean {
    return raycastPolyhedron(this.hull, origin, dir, maxT, out);
  }
}
