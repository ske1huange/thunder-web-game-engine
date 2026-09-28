import { Vec3 } from '../math/Vec3';

/** 流形中的单个接触点 */
export class ManifoldPoint {
  /** 世界坐标接触点（两表面的中点） */
  readonly point = new Vec3();
  /** 接触点在刚体 A、B 局部坐标系中的位置（用于帧间匹配） */
  readonly localA = new Vec3();
  readonly localB = new Vec3();
  /** 分离距离，负值表示穿透 */
  separation = 0;
  /** 特征编号（由几何特征生成，用于帧间匹配以进行 warm start） */
  id = 0;

  // ---- 求解器状态（跨帧保留） ----
  normalImpulse = 0;
  tangentImpulse1 = 0;
  tangentImpulse2 = 0;
  /** 本步中最大的法向冲量，用于判断是否真正接触过 */
  maxNormalImpulse = 0;
  /** 本帧是否继承了上一帧的冲量 */
  persisted = false;

  // ---- 求解器临时量（每步重算） ----
  readonly anchorA = new Vec3();
  readonly anchorB = new Vec3();
  /**
   * 求解器预计算的雅可比数据（锚点在一步内固定，雅可比为常量）：
   * [rA×n, rB×n, IA⁻¹(rA×n), IB⁻¹(rB×n)] 以及两个切向的同样四组，共 36 个数。
   */
  readonly jacobian = new Float64Array(36);
  baseSeparation = 0;
  normalMass = 0;
  tangentMass1 = 0;
  tangentMass2 = 0;
  relativeVelocity = 0;

  copyGeometry(p: ManifoldPoint): void {
    this.point.copy(p.point);
    this.localA.copy(p.localA);
    this.localB.copy(p.localB);
    this.separation = p.separation;
    this.id = p.id;
  }
}

export const MAX_MANIFOLD_POINTS = 4;

/** 接触流形：共享法线（由 A 指向 B）与至多 4 个接触点 */
export class Manifold {
  readonly normal = new Vec3();
  readonly points: ManifoldPoint[] = [];
  pointCount = 0;
  /** 摩擦方向（求解器使用） */
  readonly tangent1 = new Vec3();
  readonly tangent2 = new Vec3();

  constructor() {
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.points.push(new ManifoldPoint());
  }

  clear(): void {
    this.pointCount = 0;
  }

  /** 追加一个接触点（超过上限时忽略），返回该点 */
  addPoint(point: Readonly<Vec3>, separation: number, id: number): ManifoldPoint | null {
    if (this.pointCount >= MAX_MANIFOLD_POINTS) return null;
    const p = this.points[this.pointCount++]!;
    p.point.copy(point);
    p.separation = separation;
    p.id = id;
    return p;
  }

  /** 最深的穿透距离（无接触点时返回 Infinity） */
  minSeparation(): number {
    let s = Infinity;
    for (let i = 0; i < this.pointCount; i++) s = Math.min(s, this.points[i]!.separation);
    return s;
  }
}

/**
 * 候选接触点缓冲区：裁剪可能产生超过 4 个点，先收集再约减。
 */
export class ContactBuffer {
  readonly points: Vec3[] = [];
  readonly separations: number[] = [];
  readonly ids: number[] = [];
  count = 0;

  clear(): void {
    this.count = 0;
  }

  push(p: Readonly<Vec3>, separation: number, id: number): void {
    if (this.points.length <= this.count) this.points.push(new Vec3());
    this.points[this.count]!.copy(p);
    this.separations[this.count] = separation;
    this.ids[this.count] = id;
    this.count++;
  }

  /**
   * 约减到至多 4 个点并写入流形：
   * 最深点 → 离它最远的点 → 与前两点构成最大三角形的点 → 最能扩大面积的点。
   */
  reduceInto(manifold: Manifold, normal: Readonly<Vec3>): void {
    const n = this.count;
    if (n <= MAX_MANIFOLD_POINTS) {
      for (let i = 0; i < n; i++)
        manifold.addPoint(this.points[i]!, this.separations[i]!, this.ids[i]!);
      return;
    }
    const pts = this.points;
    // 1. 最深点
    let i0 = 0;
    for (let i = 1; i < n; i++) if (this.separations[i]! < this.separations[i0]!) i0 = i;
    // 2. 最远点
    let i1 = -1;
    let best = -1;
    for (let i = 0; i < n; i++) {
      const d = pts[i]!.distanceToSq(pts[i0]!);
      if (d > best) {
        best = d;
        i1 = i;
      }
    }
    // 3. 最大三角形
    let i2 = -1;
    best = -1;
    let sign = 1;
    for (let i = 0; i < n; i++) {
      if (i === i0 || i === i1) continue;
      const area = signedArea(pts[i0]!, pts[i1]!, pts[i]!, normal);
      if (Math.abs(area) > best) {
        best = Math.abs(area);
        i2 = i;
        sign = area >= 0 ? 1 : -1;
      }
    }
    // 4. 位于三角形外侧、最能扩大面积的点
    let i3 = -1;
    if (i2 >= 0) {
      let most = 0;
      const a = pts[i0]!,
        b = pts[i1]!,
        c = pts[i2]!;
      for (let i = 0; i < n; i++) {
        if (i === i0 || i === i1 || i === i2) continue;
        const p = pts[i]!;
        // 相对三角形各边的有向面积，取最负者（越负越在外侧）
        const e0 = sign * signedArea(a, b, p, normal);
        const e1 = sign * signedArea(b, c, p, normal);
        const e2 = sign * signedArea(c, a, p, normal);
        const m = Math.min(e0, e1, e2);
        if (m < most) {
          most = m;
          i3 = i;
        }
      }
    }
    for (const i of [i0, i1, i2, i3]) {
      if (i >= 0) manifold.addPoint(pts[i]!, this.separations[i]!, this.ids[i]!);
    }
  }
}

const tmpU = new Vec3();
const tmpV = new Vec3();

function signedArea(a: Vec3, b: Vec3, c: Vec3, n: Readonly<Vec3>): number {
  tmpU.subVectors(b, a);
  tmpV.subVectors(c, a);
  return tmpU.cross(tmpV).dot(n);
}
