import type { Transform } from '../../math/Transform';
import { Vec3 } from '../../math/Vec3';
import type { Shape } from '../../shapes/Shape';

/**
 * GJK 使用的凸体代理：世界坐标下的核心点集 + 半径。
 */
export class DistanceProxy {
  /** 当前使用的点集（可能引用外部数组，只读使用） */
  points: readonly Vec3[];
  count = 0;
  radius = 0;
  private readonly storage: Vec3[] = [];

  constructor() {
    this.points = this.storage;
  }

  /** 由形状与变换填充（点被变换到世界坐标） */
  setShape(shape: Shape, transform: Readonly<Transform>): this {
    const core = shape.getCorePoints();
    this.ensure(core.length);
    for (let i = 0; i < core.length; i++) {
      transform.transformPoint(core[i]!, this.storage[i]!);
    }
    this.points = this.storage;
    this.count = core.length;
    this.radius = shape.radius;
    return this;
  }

  /** 复制点集 */
  setPoints(points: readonly Readonly<Vec3>[], radius: number, count = points.length): this {
    this.ensure(count);
    for (let i = 0; i < count; i++) this.storage[i]!.copy(points[i]!);
    this.points = this.storage;
    this.count = count;
    this.radius = radius;
    return this;
  }

  /** 直接引用外部点集（不复制，调用方需保证期间不被修改） */
  setPointsRef(points: readonly Vec3[], radius: number, count = points.length): this {
    this.points = points;
    this.count = count;
    this.radius = radius;
    return this;
  }

  /** 平移所有点（会先把引用的点集复制到内部存储） */
  translate(d: Readonly<Vec3>): this {
    if (this.points !== this.storage) this.setPoints(this.points, this.radius, this.count);
    for (let i = 0; i < this.count; i++) this.storage[i]!.add(d);
    return this;
  }

  support(dir: Readonly<Vec3>): number {
    let best = 0;
    let bestDot = -Infinity;
    for (let i = 0; i < this.count; i++) {
      const p = this.points[i]!;
      const d = p.x * dir.x + p.y * dir.y + p.z * dir.z;
      if (d > bestDot) {
        bestDot = d;
        best = i;
      }
    }
    return best;
  }

  private ensure(n: number): void {
    while (this.storage.length < n) this.storage.push(new Vec3());
  }
}

export class DistanceOutput {
  /** A 上的最近点 */
  readonly pointA = new Vec3();
  /** B 上的最近点 */
  readonly pointB = new Vec3();
  /** 由 A 指向 B 的单位法线（距离为 0 时可能为零向量） */
  readonly normal = new Vec3();
  distance = 0;
  iterations = 0;
  simplexCount = 0;
}

class SimplexVertex {
  readonly wA = new Vec3();
  readonly wB = new Vec3();
  readonly w = new Vec3();
  a = 0;
  indexA = 0;
  indexB = 0;

  copy(v: SimplexVertex): void {
    this.wA.copy(v.wA);
    this.wB.copy(v.wB);
    this.w.copy(v.w);
    this.a = v.a;
    this.indexA = v.indexA;
    this.indexB = v.indexB;
  }
}

const verts = [new SimplexVertex(), new SimplexVertex(), new SimplexVertex(), new SimplexVertex()];
const scratch = [
  new SimplexVertex(),
  new SimplexVertex(),
  new SimplexVertex(),
  new SimplexVertex(),
];
let count = 0;

const tmpA = new Vec3();
const tmpB = new Vec3();
const tmpC = new Vec3();
const tmpD = new Vec3();
const tmpE = new Vec3();
const weights = [0, 0, 0];
const bestWeights = [0, 0, 0];

const MAX_ITERATIONS = 32;

/**
 * 计算三角形 abc 上离原点最近的点的重心坐标，写入 w，返回距离平方。
 * 参考 Ericson《Real-Time Collision Detection》5.1.5。
 */
function closestOnTriangle(a: Vec3, b: Vec3, c: Vec3, w: number[]): number {
  const ab = tmpA.subVectors(b, a);
  const ac = tmpB.subVectors(c, a);
  // ap = -a
  const d1 = -ab.dot(a);
  const d2 = -ac.dot(a);
  if (d1 <= 0 && d2 <= 0) {
    w[0] = 1;
    w[1] = 0;
    w[2] = 0;
    return a.lengthSq();
  }
  const d3 = -ab.dot(b);
  const d4 = -ac.dot(b);
  if (d3 >= 0 && d4 <= d3) {
    w[0] = 0;
    w[1] = 1;
    w[2] = 0;
    return b.lengthSq();
  }
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const t = d1 / (d1 - d3);
    w[0] = 1 - t;
    w[1] = t;
    w[2] = 0;
    return tmpC.copy(a).addScaled(ab, t).lengthSq();
  }
  const d5 = -ab.dot(c);
  const d6 = -ac.dot(c);
  if (d6 >= 0 && d5 <= d6) {
    w[0] = 0;
    w[1] = 0;
    w[2] = 1;
    return c.lengthSq();
  }
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const t = d2 / (d2 - d6);
    w[0] = 1 - t;
    w[1] = 0;
    w[2] = t;
    return tmpC.copy(a).addScaled(ac, t).lengthSq();
  }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const t = (d4 - d3) / (d4 - d3 + (d5 - d6));
    w[0] = 0;
    w[1] = 1 - t;
    w[2] = t;
    return tmpC.subVectors(c, b).scale(t).add(b).lengthSq();
  }
  const sum = va + vb + vc;
  if (!(sum > 1e-300)) {
    // 退化三角形：取三条边中最近的
    return closestOnDegenerateTriangle(a, b, c, w);
  }
  const denom = 1 / sum;
  const v = vb * denom;
  const ww = vc * denom;
  w[0] = 1 - v - ww;
  w[1] = v;
  w[2] = ww;
  return tmpC.copy(a).addScaled(ab, v).addScaled(ac, ww).lengthSq();
}

function closestOnSegmentWeights(a: Vec3, b: Vec3): { t: number; d2: number } {
  const e = tmpD.subVectors(b, a);
  const ee = e.lengthSq();
  let t = ee > 0 ? -a.dot(e) / ee : 0;
  t = Math.max(0, Math.min(1, t));
  const d2 = tmpE.copy(a).addScaled(e, t).lengthSq();
  return { t, d2 };
}

function closestOnDegenerateTriangle(a: Vec3, b: Vec3, c: Vec3, w: number[]): number {
  let best = Infinity;
  const ab = closestOnSegmentWeights(a, b);
  if (ab.d2 < best) {
    best = ab.d2;
    w[0] = 1 - ab.t;
    w[1] = ab.t;
    w[2] = 0;
  }
  const bc = closestOnSegmentWeights(b, c);
  if (bc.d2 < best) {
    best = bc.d2;
    w[0] = 0;
    w[1] = 1 - bc.t;
    w[2] = bc.t;
  }
  const ca = closestOnSegmentWeights(c, a);
  if (ca.d2 < best) {
    best = ca.d2;
    w[0] = ca.t;
    w[1] = 0;
    w[2] = 1 - ca.t;
  }
  return best;
}

/** 去掉权重为 0 的顶点 */
function compact(): void {
  let n = 0;
  for (let i = 0; i < count; i++) {
    if (verts[i]!.a > 0) {
      if (n !== i) verts[n]!.copy(verts[i]!);
      n++;
    }
  }
  count = n;
}

function solve2(): void {
  const w1 = verts[0]!.w;
  const w2 = verts[1]!.w;
  const e12 = tmpA.subVectors(w2, w1);
  const d12_2 = -w1.dot(e12);
  if (d12_2 <= 0) {
    verts[0]!.a = 1;
    count = 1;
    return;
  }
  const d12_1 = w2.dot(e12);
  if (d12_1 <= 0) {
    verts[1]!.a = 1;
    verts[0]!.copy(verts[1]!);
    count = 1;
    return;
  }
  const inv = 1 / (d12_1 + d12_2);
  verts[0]!.a = d12_1 * inv;
  verts[1]!.a = d12_2 * inv;
  count = 2;
}

function solve3(): void {
  closestOnTriangle(verts[0]!.w, verts[1]!.w, verts[2]!.w, weights);
  verts[0]!.a = weights[0]!;
  verts[1]!.a = weights[1]!;
  verts[2]!.a = weights[2]!;
  compact();
}

/** 原点是否位于平面 abc 的、与 d 相反的一侧（退化时返回 true 以便检查该面） */
function originOutsideOfPlane(a: Vec3, b: Vec3, c: Vec3, d: Vec3): boolean {
  const n = tmpD.subVectors(b, a).cross(tmpE.subVectors(c, a));
  const signP = -n.dot(a);
  const signD = n.dot(tmpE.subVectors(d, a));
  if (signD * signD < 1e-24) return true;
  return signP * signD < 0;
}

const FACES = [
  [0, 1, 2, 3],
  [0, 2, 3, 1],
  [0, 3, 1, 2],
  [1, 3, 2, 0],
] as const;

function solve4(): void {
  let best = Infinity;
  let bestFace = -1;
  for (let f = 0; f < 4; f++) {
    const [i, j, k, l] = FACES[f]!;
    const a = verts[i]!.w,
      b = verts[j]!.w,
      c = verts[k]!.w,
      d = verts[l]!.w;
    if (!originOutsideOfPlane(a, b, c, d)) continue;
    const d2 = closestOnTriangle(a, b, c, weights);
    if (d2 < best) {
      best = d2;
      bestFace = f;
      bestWeights[0] = weights[0]!;
      bestWeights[1] = weights[1]!;
      bestWeights[2] = weights[2]!;
    }
  }
  if (bestFace < 0) {
    // 原点在四面体内：重叠
    count = 4;
    return;
  }
  const [i, j, k] = FACES[bestFace]!;
  scratch[0]!.copy(verts[i]!);
  scratch[1]!.copy(verts[j]!);
  scratch[2]!.copy(verts[k]!);
  scratch[0]!.a = bestWeights[0]!;
  scratch[1]!.a = bestWeights[1]!;
  scratch[2]!.a = bestWeights[2]!;
  verts[0]!.copy(scratch[0]!);
  verts[1]!.copy(scratch[1]!);
  verts[2]!.copy(scratch[2]!);
  count = 3;
  compact();
}

function closestPoint(out: Vec3): Vec3 {
  out.setZero();
  for (let i = 0; i < count; i++) out.addScaled(verts[i]!.w, verts[i]!.a);
  return out;
}

const saveA = [0, 0, 0, 0];
const saveB = [0, 0, 0, 0];
const searchDir = new Vec3();
const closest = new Vec3();

/**
 * GJK 求两个凸体（核心 + 半径）之间的距离与最近点。
 * @param useRadii 为 true 时扣除两者半径；若表面相交则距离为 0
 */
export function gjkDistance(
  proxyA: DistanceProxy,
  proxyB: DistanceProxy,
  useRadii: boolean,
  out: DistanceOutput,
): DistanceOutput {
  // 初始单纯形：两者的第 0 个点
  const v0 = verts[0]!;
  v0.indexA = 0;
  v0.indexB = 0;
  v0.wA.copy(proxyA.points[0]!);
  v0.wB.copy(proxyB.points[0]!);
  v0.w.subVectors(v0.wB, v0.wA);
  v0.a = 1;
  count = 1;

  let iter = 0;
  while (iter < MAX_ITERATIONS) {
    const saveCount = count;
    for (let i = 0; i < saveCount; i++) {
      saveA[i] = verts[i]!.indexA;
      saveB[i] = verts[i]!.indexB;
    }

    if (count === 2) solve2();
    else if (count === 3) solve3();
    else if (count === 4) solve4();

    if (count === 4) break;

    closestPoint(closest);
    const distSq = closest.lengthSq();
    if (distSq < 1e-24) break;

    // 搜索方向指向原点
    searchDir.copy(closest).negate();
    const vNew = verts[count]!;
    vNew.indexA = proxyA.support(closest); // A 沿 -d = closest 方向
    vNew.indexB = proxyB.support(searchDir);
    vNew.wA.copy(proxyA.points[vNew.indexA]!);
    vNew.wB.copy(proxyB.points[vNew.indexB]!);
    vNew.w.subVectors(vNew.wB, vNew.wA);
    iter++;

    // 重复顶点：无法继续改进
    let duplicate = false;
    for (let i = 0; i < saveCount; i++) {
      if (vNew.indexA === saveA[i] && vNew.indexB === saveB[i]) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) break;
    // 进展判据：新点在搜索方向上没有显著更靠近原点
    if (distSq - closest.dot(vNew.w) <= 1e-10 * distSq) break;
    count++;
  }

  // 见证点
  const pA = out.pointA.setZero();
  const pB = out.pointB.setZero();
  for (let i = 0; i < count; i++) {
    pA.addScaled(verts[i]!.wA, verts[i]!.a);
    pB.addScaled(verts[i]!.wB, verts[i]!.a);
  }
  out.iterations = iter;
  out.simplexCount = count;
  if (count === 4) {
    pB.copy(pA);
  }
  out.normal.subVectors(pB, pA);
  let distance = out.normal.normalize();
  if (useRadii) {
    const rA = proxyA.radius;
    const rB = proxyB.radius;
    if (distance > rA + rB && distance > 1e-12) {
      distance -= rA + rB;
      pA.addScaled(out.normal, rA);
      pB.addScaled(out.normal, -rB);
    } else {
      pA.add(pB).scale(0.5);
      pB.copy(pA);
      distance = 0;
    }
  }
  out.distance = distance;
  return out;
}
