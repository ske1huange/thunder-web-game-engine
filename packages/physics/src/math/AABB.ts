import { Vec3 } from './Vec3';

/** 轴对齐包围盒 */
export class AABB {
  readonly min: Vec3;
  readonly max: Vec3;

  constructor(min?: Readonly<Vec3>, max?: Readonly<Vec3>) {
    this.min = min ? new Vec3().copy(min) : new Vec3();
    this.max = max ? new Vec3().copy(max) : new Vec3();
  }

  set(minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): this {
    this.min.set(minX, minY, minZ);
    this.max.set(maxX, maxY, maxZ);
    return this;
  }

  copy(b: Readonly<AABB>): this {
    this.min.copy(b.min);
    this.max.copy(b.max);
    return this;
  }

  clone(): AABB {
    return new AABB(this.min, this.max);
  }

  /** 置为“空”包围盒，便于随后 expandByPoint */
  makeEmpty(): this {
    this.min.setScalar(Infinity);
    this.max.setScalar(-Infinity);
    return this;
  }

  isValid(): boolean {
    return this.min.x <= this.max.x && this.min.y <= this.max.y && this.min.z <= this.max.z;
  }

  expandByPoint(p: Readonly<Vec3>): this {
    this.min.min(p);
    this.max.max(p);
    return this;
  }

  /** 各方向扩张 margin */
  expandByScalar(margin: number): this {
    this.min.x -= margin;
    this.min.y -= margin;
    this.min.z -= margin;
    this.max.x += margin;
    this.max.y += margin;
    this.max.z += margin;
    return this;
  }

  union(a: Readonly<AABB>, b: Readonly<AABB>): this {
    this.min.set(Math.min(a.min.x, b.min.x), Math.min(a.min.y, b.min.y), Math.min(a.min.z, b.min.z));
    this.max.set(Math.max(a.max.x, b.max.x), Math.max(a.max.y, b.max.y), Math.max(a.max.z, b.max.z));
    return this;
  }

  overlaps(b: Readonly<AABB>): boolean {
    return (
      this.min.x <= b.max.x &&
      this.max.x >= b.min.x &&
      this.min.y <= b.max.y &&
      this.max.y >= b.min.y &&
      this.min.z <= b.max.z &&
      this.max.z >= b.min.z
    );
  }

  /** 是否完全包含 b */
  contains(b: Readonly<AABB>): boolean {
    return (
      this.min.x <= b.min.x &&
      this.min.y <= b.min.y &&
      this.min.z <= b.min.z &&
      b.max.x <= this.max.x &&
      b.max.y <= this.max.y &&
      b.max.z <= this.max.z
    );
  }

  containsPoint(p: Readonly<Vec3>): boolean {
    return (
      p.x >= this.min.x &&
      p.x <= this.max.x &&
      p.y >= this.min.y &&
      p.y <= this.max.y &&
      p.z >= this.min.z &&
      p.z <= this.max.z
    );
  }

  /** 表面积，用作 BVH 的 SAH 代价 */
  surfaceArea(): number {
    const dx = this.max.x - this.min.x;
    const dy = this.max.y - this.min.y;
    const dz = this.max.z - this.min.z;
    return 2 * (dx * dy + dy * dz + dz * dx);
  }

  getCenter(out: Vec3): Vec3 {
    return out.addVectors(this.min, this.max).scale(0.5);
  }

  getExtents(out: Vec3): Vec3 {
    return out.subVectors(this.max, this.min).scale(0.5);
  }

  /**
   * 射线与包围盒求交（slab 法）。
   * @returns 进入参数 t（∈ [0, maxT]），不相交返回 -1
   */
  rayIntersect(origin: Readonly<Vec3>, dir: Readonly<Vec3>, maxT: number): number {
    let tmin = 0;
    let tmax = maxT;
    for (let i = 0; i < 3; i++) {
      const o = i === 0 ? origin.x : i === 1 ? origin.y : origin.z;
      const d = i === 0 ? dir.x : i === 1 ? dir.y : dir.z;
      const lo = i === 0 ? this.min.x : i === 1 ? this.min.y : this.min.z;
      const hi = i === 0 ? this.max.x : i === 1 ? this.max.y : this.max.z;
      if (Math.abs(d) < 1e-12) {
        if (o < lo || o > hi) return -1;
      } else {
        const inv = 1 / d;
        let t1 = (lo - o) * inv;
        let t2 = (hi - o) * inv;
        if (t1 > t2) {
          const t = t1;
          t1 = t2;
          t2 = t;
        }
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;
        if (tmin > tmax) return -1;
      }
    }
    return tmin;
  }
}
