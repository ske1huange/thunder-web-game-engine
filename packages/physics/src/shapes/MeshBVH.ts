import type { AABB } from '../math/AABB';
import type { Vec3 } from '../math/Vec3';

/** 叶子节点最多容纳的三角形数 */
const LEAF_SIZE = 4;

/**
 * 遍历栈：所有 BVH 共用一块内存，嵌套调用从当前栈顶继续，退出时恢复。
 * 中位数二分保证深度不超过 log2(n) + 1，因此一次遍历最多占用约 2·log2(n) 个槽位。
 */
let stack = new Int32Array(256);
let stackTop = 0;

function ensureStack(n: number): void {
  if (n <= stack.length) return;
  const grown = new Int32Array(Math.max(n, stack.length * 2));
  grown.set(stack);
  stack = grown;
}

/**
 * 静态三角形包围体层次（BVH）。
 *
 * 构建：按三角形包围盒中心在最长轴上取中位数二分，叶子至多 4 个三角形；
 * 节点按深度优先顺序平铺在 TypedArray 中（左孩子紧跟父节点，只记录右孩子下标），
 * 构建后不再修改，查询与射线遍历都不分配内存。
 */
export class MeshBVH {
  /** 节点包围盒，每个节点 6 个数：minX, minY, minZ, maxX, maxY, maxZ */
  readonly bounds: Float64Array;
  /** 内部节点为右孩子下标（左孩子为 i + 1）；叶子为 -1 */
  readonly right: Int32Array;
  /** 叶子在 triIndices 中的起始位置 */
  readonly start: Uint32Array;
  /** 叶子的三角形数（内部节点为 0） */
  readonly count: Uint32Array;
  /** 按叶子顺序排列的三角形编号 */
  readonly triIndices: Uint32Array;
  readonly nodeCount: number;
  /** 树的深度 */
  readonly depth: number;

  /**
   * @param triBounds 每个三角形的包围盒（6 个数一组，布局同 bounds）
   * @param triCount 三角形数量
   */
  constructor(triBounds: ArrayLike<number>, triCount: number) {
    const maxNodes = Math.max(1, 2 * Math.ceil(triCount / LEAF_SIZE) + 1) * 2;
    const bounds = new Float64Array(maxNodes * 6);
    const right = new Int32Array(maxNodes);
    const start = new Uint32Array(maxNodes);
    const count = new Uint32Array(maxNodes);
    const indices = new Uint32Array(triCount);
    const centers = new Float64Array(triCount * 3);
    for (let i = 0; i < triCount; i++) {
      indices[i] = i;
      const o = i * 6;
      centers[i * 3] = 0.5 * (triBounds[o]! + triBounds[o + 3]!);
      centers[i * 3 + 1] = 0.5 * (triBounds[o + 1]! + triBounds[o + 4]!);
      centers[i * 3 + 2] = 0.5 * (triBounds[o + 2]! + triBounds[o + 5]!);
    }

    let nodeCount = 0;
    let maxDepth = 0;
    const build = (lo: number, hi: number, depth: number): number => {
      const node = nodeCount++;
      if (depth > maxDepth) maxDepth = depth;
      // 节点包围盒与中心包围盒
      let bx0 = Infinity,
        by0 = Infinity,
        bz0 = Infinity,
        bx1 = -Infinity,
        by1 = -Infinity,
        bz1 = -Infinity;
      let cx0 = Infinity,
        cy0 = Infinity,
        cz0 = Infinity,
        cx1 = -Infinity,
        cy1 = -Infinity,
        cz1 = -Infinity;
      for (let k = lo; k < hi; k++) {
        const t = indices[k]!;
        const o = t * 6;
        if (triBounds[o]! < bx0) bx0 = triBounds[o]!;
        if (triBounds[o + 1]! < by0) by0 = triBounds[o + 1]!;
        if (triBounds[o + 2]! < bz0) bz0 = triBounds[o + 2]!;
        if (triBounds[o + 3]! > bx1) bx1 = triBounds[o + 3]!;
        if (triBounds[o + 4]! > by1) by1 = triBounds[o + 4]!;
        if (triBounds[o + 5]! > bz1) bz1 = triBounds[o + 5]!;
        const cx = centers[t * 3]!,
          cy = centers[t * 3 + 1]!,
          cz = centers[t * 3 + 2]!;
        if (cx < cx0) cx0 = cx;
        if (cy < cy0) cy0 = cy;
        if (cz < cz0) cz0 = cz;
        if (cx > cx1) cx1 = cx;
        if (cy > cy1) cy1 = cy;
        if (cz > cz1) cz1 = cz;
      }
      const b = node * 6;
      bounds[b] = bx0;
      bounds[b + 1] = by0;
      bounds[b + 2] = bz0;
      bounds[b + 3] = bx1;
      bounds[b + 4] = by1;
      bounds[b + 5] = bz1;

      const ex = cx1 - cx0,
        ey = cy1 - cy0,
        ez = cz1 - cz0;
      const extent = Math.max(ex, ey, ez);
      if (hi - lo <= LEAF_SIZE || !(extent > 0)) {
        right[node] = -1;
        start[node] = lo;
        count[node] = hi - lo;
        return node;
      }
      const axis = ex === extent ? 0 : ey === extent ? 1 : 2;
      const mid = (lo + hi) >>> 1;
      selectNth(indices, centers, axis, lo, hi - 1, mid);
      build(lo, mid, depth + 1);
      right[node] = build(mid, hi, depth + 1);
      count[node] = 0;
      return node;
    };
    if (triCount > 0) build(0, triCount, 1);
    else {
      nodeCount = 1;
      right[0] = -1;
      bounds.fill(0, 0, 6);
    }

    this.nodeCount = nodeCount;
    this.depth = maxDepth;
    this.bounds = bounds.slice(0, nodeCount * 6);
    this.right = right.slice(0, nodeCount);
    this.start = start.slice(0, nodeCount);
    this.count = count.slice(0, nodeCount);
    this.triIndices = indices;
  }

  /** 遍历包围盒与 aabb 相交的三角形；回调返回 false 时提前结束 */
  query(aabb: Readonly<AABB>, callback: (tri: number) => boolean | void): void {
    const bounds = this.bounds;
    const minX = aabb.min.x,
      minY = aabb.min.y,
      minZ = aabb.min.z;
    const maxX = aabb.max.x,
      maxY = aabb.max.y,
      maxZ = aabb.max.z;
    const base = stackTop;
    ensureStack(base + this.depth * 2 + 2);
    let sp = base;
    stack[sp++] = 0;
    stackTop = base + this.depth * 2 + 2;
    try {
      while (sp > base) {
        const node = stack[--sp]!;
        const b = node * 6;
        if (
          bounds[b]! > maxX ||
          bounds[b + 1]! > maxY ||
          bounds[b + 2]! > maxZ ||
          bounds[b + 3]! < minX ||
          bounds[b + 4]! < minY ||
          bounds[b + 5]! < minZ
        ) {
          continue;
        }
        const r = this.right[node]!;
        if (r < 0) {
          const s = this.start[node]!;
          const e = s + this.count[node]!;
          for (let k = s; k < e; k++) {
            if (callback(this.triIndices[k]!) === false) return;
          }
        } else {
          stack[sp++] = r;
          stack[sp++] = node + 1;
        }
      }
    } finally {
      stackTop = base;
    }
  }

  /**
   * 射线遍历：按由近到远的顺序访问叶子中的三角形。
   * 回调返回命中参数 t（用于收紧后续搜索范围），未命中返回 -1。
   * @param dir 射线方向（不要求单位长度，t 以 dir 的长度为单位）
   */
  raycast(
    origin: Readonly<Vec3>,
    dir: Readonly<Vec3>,
    maxT: number,
    callback: (tri: number, maxT: number) => number,
  ): void {
    const bounds = this.bounds;
    const ox = origin.x,
      oy = origin.y,
      oz = origin.z;
    const ix = 1 / dir.x,
      iy = 1 / dir.y,
      iz = 1 / dir.z;
    const base = stackTop;
    ensureStack(base + this.depth * 2 + 2);
    let sp = base;
    stack[sp++] = 0;
    stackTop = base + this.depth * 2 + 2;
    let tMax = maxT;
    // 节点进入参数（未命中返回 Infinity）
    const enter = (node: number): number => {
      const b = node * 6;
      let t0 = (bounds[b]! - ox) * ix;
      let t1 = (bounds[b + 3]! - ox) * ix;
      let tmin = Math.min(t0, t1);
      let tmax = Math.max(t0, t1);
      t0 = (bounds[b + 1]! - oy) * iy;
      t1 = (bounds[b + 4]! - oy) * iy;
      tmin = Math.max(tmin, Math.min(t0, t1));
      tmax = Math.min(tmax, Math.max(t0, t1));
      t0 = (bounds[b + 2]! - oz) * iz;
      t1 = (bounds[b + 5]! - oz) * iz;
      tmin = Math.max(tmin, Math.min(t0, t1));
      tmax = Math.min(tmax, Math.max(t0, t1));
      // 方向分量为 0 时 0 * Infinity = NaN，Math.min/max 会传播 NaN，这里单独处理
      if (tmin !== tmin || tmax !== tmax) return slowEnter(bounds, b, origin, dir, tMax);
      if (tmax < Math.max(tmin, 0) || tmin > tMax) return Infinity;
      return Math.max(tmin, 0);
    };
    try {
      if (enter(0) === Infinity) return;
      while (sp > base) {
        const node = stack[--sp]!;
        const r = this.right[node]!;
        if (r < 0) {
          const s = this.start[node]!;
          const e = s + this.count[node]!;
          for (let k = s; k < e; k++) {
            const t = callback(this.triIndices[k]!, tMax);
            if (t >= 0 && t < tMax) tMax = t;
          }
          continue;
        }
        const tl = enter(node + 1);
        const tr = enter(r);
        // 先压远的再压近的，保证近的先出栈
        if (tl <= tr) {
          if (tr <= tMax) stack[sp++] = r;
          if (tl <= tMax) stack[sp++] = node + 1;
        } else {
          if (tl <= tMax) stack[sp++] = node + 1;
          if (tr <= tMax) stack[sp++] = r;
        }
      }
    } finally {
      stackTop = base;
    }
  }
}

/** 方向分量为 0 时的逐轴 slab 测试 */
function slowEnter(
  bounds: Float64Array,
  b: number,
  origin: Readonly<Vec3>,
  dir: Readonly<Vec3>,
  maxT: number,
): number {
  let tmin = 0;
  let tmax = maxT;
  for (let i = 0; i < 3; i++) {
    const o = i === 0 ? origin.x : i === 1 ? origin.y : origin.z;
    const d = i === 0 ? dir.x : i === 1 ? dir.y : dir.z;
    const lo = bounds[b + i]!;
    const hi = bounds[b + 3 + i]!;
    if (d === 0) {
      if (o < lo || o > hi) return Infinity;
      continue;
    }
    let t0 = (lo - o) / d;
    let t1 = (hi - o) / d;
    if (t0 > t1) {
      const t = t0;
      t0 = t1;
      t1 = t;
    }
    if (t0 > tmin) tmin = t0;
    if (t1 < tmax) tmax = t1;
    if (tmin > tmax) return Infinity;
  }
  return tmin;
}

/** 快速选择：把 indices[lo..hi] 按中心坐标部分排序，使第 k 个元素就位 */
function selectNth(
  indices: Uint32Array,
  centers: Float64Array,
  axis: number,
  lo: number,
  hi: number,
  k: number,
): void {
  while (hi > lo) {
    const pivot = centers[indices[(lo + hi) >>> 1]! * 3 + axis]!;
    let i = lo;
    let j = hi;
    while (i <= j) {
      while (centers[indices[i]! * 3 + axis]! < pivot) i++;
      while (centers[indices[j]! * 3 + axis]! > pivot) j--;
      if (i <= j) {
        const t = indices[i]!;
        indices[i] = indices[j]!;
        indices[j] = t;
        i++;
        j--;
      }
    }
    if (k <= j) hi = j;
    else if (k >= i) lo = i;
    else return;
  }
}
