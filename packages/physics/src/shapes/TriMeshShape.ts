import type { AABB } from '../math/AABB';
import { Vec3 } from '../math/Vec3';
import { MeshBVH } from './MeshBVH';
import { MeshShape } from './MeshShape';
import { EDGE_AB, FLAT_EDGE_COS, isEdgeActive, rayTriangle } from './meshUtils';
import { type ShapeRayHit, ShapeType } from './Shape';

export interface TriMeshOptions {
  /** 两面都碰撞（默认 false：只有正面，即从外侧看逆时针的一面） */
  doubleSided?: boolean;
  /**
   * 顶点焊接容差（米），默认 1e-5。模型导出时常因法线 / UV 不同而复制顶点，
   * 焊接后才能正确识别相邻三角形，从而消除内部棱的“幽灵碰撞”。设为 0 关闭。
   */
  weldTolerance?: number;
}

const va = new Vec3();
const vb = new Vec3();
const vc = new Vec3();
const vd = new Vec3();
const na = new Vec3();
const nb = new Vec3();
const hit: ShapeRayHit = { t: 0, normal: new Vec3() };

/**
 * 三角网格形状：用于关卡几何（地面、墙体、楼梯、斜坡等任意静态模型）。
 *
 * - 只能挂在静态或运动学刚体上
 * - BVH 加速的三角形查询与射线检测
 * - 默认单面：物体从背面进入不会被推出（与大多数游戏引擎一致）
 * - 预先计算每条棱是否“活跃”，平坦或内凹的内部棱不会产生侧向法线，
 *   物体在网格上滑动 / 滚动时不会被三角形接缝绊住
 */
export class TriMeshShape extends MeshShape {
  readonly type = ShapeType.TriMesh;
  readonly doubleSided: boolean;
  /** 顶点坐标（焊接后），每 3 个数一个顶点 */
  readonly positions: Float64Array;
  /** 三角形顶点下标，每 3 个一组 */
  readonly indices: Uint32Array;
  /** 每个三角形的活跃棱标志 */
  readonly edgeFlags: Uint8Array;
  readonly bvh: MeshBVH;
  readonly triangleCount: number;

  /**
   * @param vertices 顶点坐标（扁平数组 [x, y, z, ...] 或 Vec3 数组）
   * @param indices 三角形顶点下标（逆时针为正面）；省略时每 3 个顶点构成一个三角形
   */
  constructor(
    vertices: ArrayLike<number> | readonly Readonly<Vec3>[],
    indices?: ArrayLike<number> | null,
    options: TriMeshOptions = {},
  ) {
    super();
    this.doubleSided = options.doubleSided ?? false;
    const flat = toFlat(vertices);
    const vertexCount = flat.length / 3;
    const rawIndices = indices ?? identityIndices(vertexCount);
    if (rawIndices.length % 3 !== 0) throw new Error('TriMeshShape: 下标数量必须是 3 的倍数');
    for (let i = 0; i < rawIndices.length; i++) {
      const v = rawIndices[i]!;
      if (!(v >= 0 && v < vertexCount)) throw new Error(`TriMeshShape: 下标越界 (${v})`);
    }

    const { positions, remap } = weld(flat, options.weldTolerance ?? 1e-5);
    this.positions = positions;

    // 去掉退化三角形（焊接后顶点重合或面积为 0）
    const tris: number[] = [];
    for (let i = 0; i < rawIndices.length; i += 3) {
      const a = remap[rawIndices[i]!]!;
      const b = remap[rawIndices[i + 1]!]!;
      const c = remap[rawIndices[i + 2]!]!;
      if (a === b || b === c || c === a) continue;
      readVertex(positions, a, va);
      readVertex(positions, b, vb);
      readVertex(positions, c, vc);
      na.subVectors(vb, va).cross(vd.subVectors(vc, va));
      if (na.lengthSq() < 1e-20) continue;
      tris.push(a, b, c);
    }
    if (tris.length === 0) throw new Error('TriMeshShape: 没有有效的三角形');
    this.indices = Uint32Array.from(tris);
    const triCount = tris.length / 3;
    this.triangleCount = triCount;

    // 包围盒与 BVH
    const triBounds = new Float64Array(triCount * 6);
    this.localBounds.makeEmpty();
    for (let t = 0; t < triCount; t++) {
      this.getTriangle(t, va, vb, vc);
      const o = t * 6;
      triBounds[o] = Math.min(va.x, vb.x, vc.x);
      triBounds[o + 1] = Math.min(va.y, vb.y, vc.y);
      triBounds[o + 2] = Math.min(va.z, vb.z, vc.z);
      triBounds[o + 3] = Math.max(va.x, vb.x, vc.x);
      triBounds[o + 4] = Math.max(va.y, vb.y, vc.y);
      triBounds[o + 5] = Math.max(va.z, vb.z, vc.z);
      this.localBounds.expandByPoint(va).expandByPoint(vb).expandByPoint(vc);
    }
    this.bvh = new MeshBVH(triBounds, triCount);
    this.edgeFlags = computeEdgeFlags(this);
  }

  getTriangle(index: number, a: Vec3, b: Vec3, c: Vec3): void {
    const idx = this.indices;
    const p = this.positions;
    let k = idx[index * 3]! * 3;
    a.set(p[k]!, p[k + 1]!, p[k + 2]!);
    k = idx[index * 3 + 1]! * 3;
    b.set(p[k]!, p[k + 1]!, p[k + 2]!);
    k = idx[index * 3 + 2]! * 3;
    c.set(p[k]!, p[k + 1]!, p[k + 2]!);
  }

  getEdgeFlags(index: number): number {
    return this.edgeFlags[index]!;
  }

  queryTriangles(localAabb: Readonly<AABB>, callback: (index: number) => boolean | void): void {
    this.bvh.query(localAabb, callback);
  }

  raycast(origin: Readonly<Vec3>, dir: Readonly<Vec3>, maxT: number, out: ShapeRayHit): boolean {
    let found = false;
    this.bvh.raycast(origin, dir, maxT, (tri, tMax) => {
      this.getTriangle(tri, va, vb, vc);
      if (!rayTriangle(origin, dir, va, vb, vc, tMax, this.doubleSided, hit)) return -1;
      found = true;
      out.t = hit.t;
      out.normal.copy(hit.normal);
      return hit.t;
    });
    return found;
  }
}

function toFlat(vertices: ArrayLike<number> | readonly Readonly<Vec3>[]): ArrayLike<number> {
  if (vertices.length === 0) throw new Error('TriMeshShape: 顶点为空');
  const first = vertices[0];
  if (typeof first === 'number') {
    if (vertices.length % 3 !== 0) throw new Error('TriMeshShape: 顶点坐标数量必须是 3 的倍数');
    return vertices as ArrayLike<number>;
  }
  const list = vertices as readonly Readonly<Vec3>[];
  const out = new Float64Array(list.length * 3);
  for (let i = 0; i < list.length; i++) {
    const v = list[i]!;
    out[i * 3] = v.x;
    out[i * 3 + 1] = v.y;
    out[i * 3 + 2] = v.z;
  }
  return out;
}

function identityIndices(n: number): Uint32Array {
  const out = new Uint32Array(n - (n % 3));
  for (let i = 0; i < out.length; i++) out[i] = i;
  return out;
}

function readVertex(p: ArrayLike<number>, i: number, out: Vec3): Vec3 {
  return out.set(p[i * 3]!, p[i * 3 + 1]!, p[i * 3 + 2]!);
}

/** 按量化坐标焊接重复顶点，返回新的顶点数组与旧下标 → 新下标的映射 */
function weld(
  flat: ArrayLike<number>,
  tolerance: number,
): { positions: Float64Array; remap: Uint32Array } {
  const n = flat.length / 3;
  const remap = new Uint32Array(n);
  if (!(tolerance > 0)) {
    for (let i = 0; i < n; i++) remap[i] = i;
    return { positions: Float64Array.from(flat), remap };
  }
  const inv = 1 / tolerance;
  const map = new Map<string, number>();
  const out: number[] = [];
  for (let i = 0; i < n; i++) {
    const x = flat[i * 3]!,
      y = flat[i * 3 + 1]!,
      z = flat[i * 3 + 2]!;
    const key = `${Math.round(x * inv)},${Math.round(y * inv)},${Math.round(z * inv)}`;
    let j = map.get(key);
    if (j === undefined) {
      j = out.length / 3;
      map.set(key, j);
      out.push(x, y, z);
    }
    remap[i] = j;
  }
  return { positions: Float64Array.from(out), remap };
}

/**
 * 由相邻关系计算每个三角形三条棱的活跃标志。
 * 只有一个三角形的边界棱、非流形棱（多于两个三角形共用）与朝向不一致的棱视为活跃。
 */
function computeEdgeFlags(mesh: TriMeshShape): Uint8Array {
  const idx = mesh.indices;
  const triCount = mesh.triangleCount;
  const flags = new Uint8Array(triCount);
  // 无向棱 → 使用它的 (三角形, 棱序号) 列表
  const edgeMap = new Map<number, number[]>();
  const vertexCount = mesh.positions.length / 3;
  const key = (a: number, b: number): number => (a < b ? a * vertexCount + b : b * vertexCount + a);
  for (let t = 0; t < triCount; t++) {
    for (let e = 0; e < 3; e++) {
      const a = idx[t * 3 + e]!;
      const b = idx[t * 3 + ((e + 1) % 3)]!;
      const k = key(a, b);
      let list = edgeMap.get(k);
      if (!list) {
        list = [];
        edgeMap.set(k, list);
      }
      list.push(t * 3 + e);
    }
  }
  for (const list of edgeMap.values()) {
    if (list.length !== 2) {
      for (const te of list) flags[(te / 3) | 0]! |= EDGE_AB << (te % 3);
      continue;
    }
    const [te1, te2] = list as [number, number];
    const t1 = (te1 / 3) | 0,
      e1 = te1 % 3;
    const t2 = (te2 / 3) | 0,
      e2 = te2 % 3;
    // 一致的朝向：两个三角形中该棱方向相反
    const a1 = idx[t1 * 3 + e1]!;
    const a2 = idx[t2 * 3 + e2]!;
    const consistent = a1 !== a2;
    const active1 = !consistent || edgeActiveBetween(mesh, t1, e1, t2, e2);
    const active2 = !consistent || edgeActiveBetween(mesh, t2, e2, t1, e1);
    if (active1) flags[t1]! |= EDGE_AB << e1;
    if (active2) flags[t2]! |= EDGE_AB << e2;
  }
  return flags;
}

/** 三角形 t1 的第 e1 条棱（与三角形 t2 的第 e2 条棱相同）对 t1 是否活跃 */
function edgeActiveBetween(
  mesh: TriMeshShape,
  t1: number,
  e1: number,
  t2: number,
  e2: number,
): boolean {
  const idx = mesh.indices;
  const p = mesh.positions;
  const edgeStart = readVertex(p, idx[t1 * 3 + e1]!, vd);
  const opposite = readVertex(p, idx[t2 * 3 + ((e2 + 2) % 3)]!, vc);
  triangleNormal(mesh, t1, na);
  triangleNormal(mesh, t2, nb);
  // 双面网格没有“内外”之分，只把平坦的棱视为非活跃
  if (mesh.doubleSided) return na.dot(nb) <= FLAT_EDGE_COS;
  return isEdgeActive(na, edgeStart, opposite, nb);
}

const ta = new Vec3();
const tb = new Vec3();
const tc = new Vec3();

function triangleNormal(mesh: TriMeshShape, t: number, out: Vec3): Vec3 {
  mesh.getTriangle(t, ta, tb, tc);
  out.subVectors(tb, ta).cross(tc.sub(ta));
  out.normalize();
  return out;
}
