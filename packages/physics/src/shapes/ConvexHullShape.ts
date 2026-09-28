import type { AABB } from '../math/AABB';
import { Mat3 } from '../math/Mat3';
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

/** 任意点集的凸包 */
export class ConvexHullShape extends Shape {
  readonly type = ShapeType.ConvexHull;
  readonly radius = 0;
  override readonly hull: ConvexPolyhedron;
  private readonly volume: number;
  private readonly unitCenter = new Vec3();
  private readonly unitInertia = new Mat3();

  constructor(points: readonly Readonly<Vec3>[]) {
    super();
    this.hull = ConvexPolyhedron.fromPoints(points);
    this.volume = this.hull.computeMassProperties(this.unitCenter, this.unitInertia);
  }

  getCorePoints(): readonly Vec3[] {
    return this.hull.vertices;
  }

  computeAABB(transform: Readonly<Transform>, out: AABB): AABB {
    return aabbOfPoints(this.hull.vertices, transform, 0, out);
  }

  getVolume(): number {
    return this.volume;
  }

  computeMass(density: number, out: MassProperties): MassProperties {
    out.mass = density * this.volume;
    out.center.copy(this.unitCenter);
    out.inertia.copy(this.unitInertia).scale(density);
    return out;
  }

  raycast(origin: Readonly<Vec3>, dir: Readonly<Vec3>, maxT: number, out: ShapeRayHit): boolean {
    return raycastPolyhedron(this.hull, origin, dir, maxT, out);
  }
}
