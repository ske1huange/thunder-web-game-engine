import { Mat3 } from '../math/Mat3';
import { Vec3 } from '../math/Vec3';

/** 凸多面体的一个面：逆时针（从外侧看）排列的顶点索引与平面 n·x = d */
export interface PolyFace {
  readonly indices: readonly number[];
  readonly normal: Vec3;
  readonly d: number;
}

/** 凸多面体的一条棱：端点与两侧的面（faceA 中方向为 a→b，faceB 中为 b→a） */
export interface PolyEdge {
  readonly a: number;
  readonly b: number;
  readonly faceA: number;
  readonly faceB: number;
}

/**
 * 凸多面体几何数据（局部坐标系）。
 *
 * Box、Cylinder、ConvexHull 形状都用它参与 SAT 分离轴测试与面裁剪，
 * 参考 Dirk Gregorius 的 GDC 2013 演讲《The Separating Axis Test between Convex Polyhedra》。
 */
export class ConvexPolyhedron {
  readonly vertices: readonly Vec3[];
  readonly faces: readonly PolyFace[];
  readonly edges: readonly PolyEdge[];
  /** 内部参考点（顶点平均值），用于确定棱-棱分离轴方向 */
  readonly centroid: Vec3;
  /** 每条棱的方向 b - a（与 edges 一一对应） */
  readonly edgeDirs: readonly Vec3[];
  /** 每条棱在高斯图上的弧平面法线 cross(n_faceB, n_faceA)，用于 SAT 剪枝 */
  readonly edgeArcNormals: readonly Vec3[];

  constructor(vertices: Vec3[], faceLoops: number[][]) {
    this.vertices = vertices;
    const faces: PolyFace[] = [];
    for (const loop of faceLoops) {
      faces.push(makeFace(vertices, loop));
    }
    this.faces = faces;
    this.edges = buildEdges(faceLoops);
    const c = new Vec3();
    for (const v of vertices) c.add(v);
    c.scale(1 / vertices.length);
    this.centroid = c;
    this.edgeDirs = this.edges.map((e) => new Vec3().subVectors(vertices[e.b]!, vertices[e.a]!));
    this.edgeArcNormals = this.edges.map((e) =>
      new Vec3().crossVectors(faces[e.faceB]!.normal, faces[e.faceA]!.normal),
    );
  }

  /** 长方体（半边长 hx, hy, hz） */
  static fromBox(hx: number, hy: number, hz: number): ConvexPolyhedron {
    const v = [
      new Vec3(-hx, -hy, -hz),
      new Vec3(hx, -hy, -hz),
      new Vec3(hx, hy, -hz),
      new Vec3(-hx, hy, -hz),
      new Vec3(-hx, -hy, hz),
      new Vec3(hx, -hy, hz),
      new Vec3(hx, hy, hz),
      new Vec3(-hx, hy, hz),
    ];
    const faces = [
      [1, 2, 6, 5], // +X
      [0, 4, 7, 3], // -X
      [3, 7, 6, 2], // +Y
      [0, 1, 5, 4], // -Y
      [4, 5, 6, 7], // +Z
      [0, 3, 2, 1], // -Z
    ];
    return new ConvexPolyhedron(v, faces);
  }

  /** 由点集计算凸包（共面三角形会合并为多边形面） */
  static fromPoints(points: readonly Readonly<Vec3>[]): ConvexPolyhedron {
    const { vertices, faces } = computeConvexHull(points);
    return new ConvexPolyhedron(vertices, faces);
  }

  /** 局部方向 dir 上最远的顶点索引 */
  supportIndex(dir: Readonly<Vec3>): number {
    let best = 0;
    let bestDot = -Infinity;
    const verts = this.vertices;
    for (let i = 0; i < verts.length; i++) {
      const v = verts[i]!;
      const d = v.x * dir.x + v.y * dir.y + v.z * dir.z;
      if (d > bestDot) {
        bestDot = d;
        best = i;
      }
    }
    return best;
  }

  /**
   * 计算密度为 1 时的体积、质心与（关于质心的）惯性张量。
   * 做法：以内部点为公共顶点把多面体剖分成四面体，累加协方差矩阵。
   */
  computeMassProperties(outCenter: Vec3, outInertia: Mat3): number {
    const ref = this.centroid;
    let volume = 0;
    const center = new Vec3();
    // 协方差矩阵 C = ∫ x xᵀ dV（相对 ref）
    let c00 = 0,
      c01 = 0,
      c02 = 0,
      c11 = 0,
      c12 = 0,
      c22 = 0;
    const a = new Vec3(),
      b = new Vec3(),
      c = new Vec3();
    for (const face of this.faces) {
      const idx = face.indices;
      const v0 = this.vertices[idx[0]!]!;
      for (let i = 1; i + 1 < idx.length; i++) {
        a.subVectors(v0, ref);
        b.subVectors(this.vertices[idx[i]!]!, ref);
        c.subVectors(this.vertices[idx[i + 1]!]!, ref);
        const det =
          a.x * (b.y * c.z - b.z * c.y) -
          a.y * (b.x * c.z - b.z * c.x) +
          a.z * (b.x * c.y - b.y * c.x);
        const vol = det / 6;
        volume += vol;
        center.x += (vol * (a.x + b.x + c.x)) / 4;
        center.y += (vol * (a.y + b.y + c.y)) / 4;
        center.z += (vol * (a.z + b.z + c.z)) / 4;
        const sx = a.x + b.x + c.x,
          sy = a.y + b.y + c.y,
          sz = a.z + b.z + c.z;
        const k = det / 120;
        c00 += k * (a.x * a.x + b.x * b.x + c.x * c.x + sx * sx);
        c11 += k * (a.y * a.y + b.y * b.y + c.y * c.y + sy * sy);
        c22 += k * (a.z * a.z + b.z * b.z + c.z * c.z + sz * sz);
        c01 += k * (a.x * a.y + b.x * b.y + c.x * c.y + sx * sy);
        c02 += k * (a.x * a.z + b.x * b.z + c.x * c.z + sx * sz);
        c12 += k * (a.y * a.z + b.y * b.z + c.y * c.z + sy * sz);
      }
    }
    if (volume <= 0) {
      outCenter.copy(ref);
      outInertia.setZero();
      return 0;
    }
    center.scale(1 / volume);
    // 平移到质心：C_com = C - V * c cᵀ
    c00 -= volume * center.x * center.x;
    c11 -= volume * center.y * center.y;
    c22 -= volume * center.z * center.z;
    c01 -= volume * center.x * center.y;
    c02 -= volume * center.x * center.z;
    c12 -= volume * center.y * center.z;
    const tr = c00 + c11 + c22;
    outInertia.set(tr - c00, -c01, -c02, -c01, tr - c11, -c12, -c02, -c12, tr - c22);
    outCenter.addVectors(center, ref);
    return volume;
  }
}

function makeFace(vertices: readonly Vec3[], loop: number[]): PolyFace {
  // Newell 法求面法线，数值上对非严格共面的多边形也稳健
  const n = new Vec3();
  for (let i = 0; i < loop.length; i++) {
    const p = vertices[loop[i]!]!;
    const q = vertices[loop[(i + 1) % loop.length]!]!;
    n.x += (p.y - q.y) * (p.z + q.z);
    n.y += (p.z - q.z) * (p.x + q.x);
    n.z += (p.x - q.x) * (p.y + q.y);
  }
  n.normalize();
  // 取各顶点到平面投影的最大值，保证所有顶点都在面内侧
  let d = -Infinity;
  for (const i of loop) d = Math.max(d, n.dot(vertices[i]!));
  return { indices: loop, normal: n, d };
}

function buildEdges(faceLoops: number[][]): PolyEdge[] {
  const directed = new Map<string, number>();
  faceLoops.forEach((loop, f) => {
    for (let i = 0; i < loop.length; i++) {
      directed.set(`${loop[i]}_${loop[(i + 1) % loop.length]}`, f);
    }
  });
  const edges: PolyEdge[] = [];
  faceLoops.forEach((loop, f) => {
    for (let i = 0; i < loop.length; i++) {
      const a = loop[i]!;
      const b = loop[(i + 1) % loop.length]!;
      if (a < b) {
        const other = directed.get(`${b}_${a}`);
        if (other === undefined) {
          throw new Error('ConvexPolyhedron: 面拓扑不闭合');
        }
        edges.push({ a, b, faceA: f, faceB: other });
      }
    }
  });
  return edges;
}

interface HullTri {
  a: number;
  b: number;
  c: number;
  n: Vec3;
  d: number;
  alive: boolean;
}

/**
 * 增量式三维凸包（beneath-beyond），O(n²)，适合几十到几百个点的碰撞体。
 * 输出合并了共面三角形后的多边形面（逆时针，法线朝外）。
 */
export function computeConvexHull(input: readonly Readonly<Vec3>[]): {
  vertices: Vec3[];
  faces: number[][];
} {
  const pts = input.map((p) => new Vec3(p.x, p.y, p.z));
  const n = pts.length;
  if (n < 4) throw new Error('ConvexHull: 至少需要 4 个不共面的点');

  // 尺度相关的容差
  let scale = 0;
  for (const p of pts) scale = Math.max(scale, Math.abs(p.x), Math.abs(p.y), Math.abs(p.z));
  const eps = Math.max(scale, 1) * 1e-7;

  // --- 初始四面体 ---
  let i0 = 0,
    i1 = -1,
    i2 = -1,
    i3 = -1;
  // i0: x 最小；i1: 离 i0 最远
  for (let i = 1; i < n; i++) if (pts[i]!.x < pts[i0]!.x) i0 = i;
  let best = -1;
  for (let i = 0; i < n; i++) {
    const d = pts[i]!.distanceToSq(pts[i0]!);
    if (d > best) {
      best = d;
      i1 = i;
    }
  }
  const e01 = new Vec3().subVectors(pts[i1]!, pts[i0]!);
  best = -1;
  const tmp = new Vec3();
  for (let i = 0; i < n; i++) {
    tmp.subVectors(pts[i]!, pts[i0]!).cross(e01);
    const d = tmp.lengthSq();
    if (d > best) {
      best = d;
      i2 = i;
    }
  }
  const nrm = new Vec3().subVectors(pts[i2]!, pts[i0]!).cross(e01);
  nrm.normalize();
  best = -1;
  for (let i = 0; i < n; i++) {
    const d = Math.abs(tmp.subVectors(pts[i]!, pts[i0]!).dot(nrm));
    if (d > best) {
      best = d;
      i3 = i;
    }
  }
  if (best <= eps || e01.length() <= eps) {
    throw new Error('ConvexHull: 点集退化（共线或共面）');
  }

  const tris: HullTri[] = [];
  const addTri = (a: number, b: number, c: number): void => {
    const pa = pts[a]!,
      pb = pts[b]!,
      pc = pts[c]!;
    const nn = new Vec3().subVectors(pb, pa).cross(new Vec3().subVectors(pc, pa));
    nn.normalize();
    tris.push({ a, b, c, n: nn, d: nn.dot(pa), alive: true });
  };
  // 保证初始四面体各面朝外
  const above =
    tmp
      .subVectors(pts[i3]!, pts[i0]!)
      .dot(
        new Vec3().subVectors(pts[i1]!, pts[i0]!).cross(new Vec3().subVectors(pts[i2]!, pts[i0]!)),
      ) > 0;
  if (above) {
    addTri(i0, i2, i1);
    addTri(i0, i1, i3);
    addTri(i1, i2, i3);
    addTri(i2, i0, i3);
  } else {
    addTri(i0, i1, i2);
    addTri(i0, i3, i1);
    addTri(i1, i3, i2);
    addTri(i2, i3, i0);
  }

  // --- 逐点加入 ---
  for (let p = 0; p < n; p++) {
    if (p === i0 || p === i1 || p === i2 || p === i3) continue;
    const pt = pts[p]!;
    const visible: HullTri[] = [];
    for (const t of tris) {
      if (t.alive && t.n.dot(pt) - t.d > eps) visible.push(t);
    }
    if (visible.length === 0) continue;
    const edgeSet = new Set<string>();
    for (const t of visible) {
      edgeSet.add(`${t.a}_${t.b}`);
      edgeSet.add(`${t.b}_${t.c}`);
      edgeSet.add(`${t.c}_${t.a}`);
    }
    const horizon: [number, number][] = [];
    for (const t of visible) {
      for (const [a, b] of [
        [t.a, t.b],
        [t.b, t.c],
        [t.c, t.a],
      ] as const) {
        if (!edgeSet.has(`${b}_${a}`)) horizon.push([a, b]);
      }
      t.alive = false;
    }
    for (const [a, b] of horizon) addTri(a, b, p);
  }

  const alive = tris.filter((t) => t.alive);

  // --- 合并共面三角形 ---
  const triOfEdge = new Map<string, number>();
  alive.forEach((t, i) => {
    triOfEdge.set(`${t.a}_${t.b}`, i);
    triOfEdge.set(`${t.b}_${t.c}`, i);
    triOfEdge.set(`${t.c}_${t.a}`, i);
  });
  const group = new Array<number>(alive.length).fill(-1);
  const groups: number[][] = [];
  const coplanarCos = 1 - 1e-9;
  for (let i = 0; i < alive.length; i++) {
    if (group[i] !== -1) continue;
    const gid = groups.length;
    const members = [i];
    group[i] = gid;
    for (let k = 0; k < members.length; k++) {
      const t = alive[members[k]!]!;
      for (const [a, b] of [
        [t.a, t.b],
        [t.b, t.c],
        [t.c, t.a],
      ] as const) {
        const j = triOfEdge.get(`${b}_${a}`);
        if (j === undefined || group[j] !== -1) continue;
        const u = alive[j]!;
        if (u.n.dot(alive[i]!.n) > coplanarCos && Math.abs(u.d - alive[i]!.d) < eps * 10) {
          group[j] = gid;
          members.push(j);
        }
      }
    }
    groups.push(members);
  }

  const loops: number[][] = [];
  for (const members of groups) {
    // 边界边：在组内没有反向边的有向边，首尾相接即为多边形轮廓
    const inGroup = new Set<string>();
    for (const m of members) {
      const t = alive[m]!;
      inGroup.add(`${t.a}_${t.b}`);
      inGroup.add(`${t.b}_${t.c}`);
      inGroup.add(`${t.c}_${t.a}`);
    }
    const next = new Map<number, number>();
    for (const m of members) {
      const t = alive[m]!;
      for (const [a, b] of [
        [t.a, t.b],
        [t.b, t.c],
        [t.c, t.a],
      ] as const) {
        if (!inGroup.has(`${b}_${a}`)) next.set(a, b);
      }
    }
    const start = next.keys().next().value as number;
    const loop: number[] = [start];
    let cur = next.get(start)!;
    let guard = 0;
    while (cur !== start && guard++ < 10000) {
      loop.push(cur);
      cur = next.get(cur)!;
    }
    loops.push(loop);
  }

  // --- 去掉共线的冗余顶点 ---
  const collinearIn = (loop: number[], k: number): boolean => {
    const prev = pts[loop[(k + loop.length - 1) % loop.length]!]!;
    const cur = pts[loop[k]!]!;
    const nxt = pts[loop[(k + 1) % loop.length]!]!;
    const u = new Vec3().subVectors(cur, prev);
    const v = new Vec3().subVectors(nxt, cur);
    const cross = new Vec3().crossVectors(u, v).length();
    return cross <= eps * Math.max(u.length(), v.length(), 1e-12) * 10;
  };
  for (const loop of loops) {
    for (let k = loop.length - 1; k >= 0 && loop.length > 3; k--) {
      if (collinearIn(loop, k)) loop.splice(k, 1);
    }
  }

  // --- 压缩顶点编号 ---
  const remap = new Map<number, number>();
  const vertices: Vec3[] = [];
  const faces = loops.map((loop) =>
    loop.map((idx) => {
      let r = remap.get(idx);
      if (r === undefined) {
        r = vertices.length;
        remap.set(idx, r);
        vertices.push(pts[idx]!.clone());
      }
      return r;
    }),
  );
  return { vertices, faces };
}
