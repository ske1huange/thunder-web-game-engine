import type { AABB } from '../math/AABB';
import { Vec3 } from '../math/Vec3';
import { MeshShape } from './MeshShape';
import { EDGE_AB, isEdgeActive, rayTriangle } from './meshUtils';
import { type ShapeRayHit, ShapeType } from './Shape';

export interface HeightfieldOptions {
  /** 高度采样，行优先：heights[row * cols + col]，row 沿 +Z，col 沿 +X */
  heights: ArrayLike<number>;
  /** 沿 Z 方向的采样数（≥ 2） */
  rows: number;
  /** 沿 X 方向的采样数（≥ 2） */
  cols: number;
  /** 相邻采样点的间距（米），默认 1；可分别指定 x / z */
  cellSize?: number | { x: number; z: number };
  /** 高度缩放，默认 1 */
  heightScale?: number;
}

const va = new Vec3();
const vb = new Vec3();
const vc = new Vec3();
const n0 = new Vec3();
const n1 = new Vec3();
const opp = new Vec3();
const corners = [new Vec3(), new Vec3(), new Vec3()];
const hit: ShapeRayHit = { t: 0, normal: new Vec3() };

/**
 * 高度场地形：规则网格上的高度采样，局部坐标系中以原点为中心（XZ 平面），+Y 为上。
 *
 * 每个格子拆成两个三角形（对角线从 (col, row) 到 (col + 1, row + 1)），
 * 与三角网格共享碰撞代码；查询按格子直接定位，射线检测用二维 DDA 逐格前进，
 * 内存只有高度数组本身加每个三角形 1 字节的棱标志。只有正面（上方）碰撞。
 */
export class HeightfieldShape extends MeshShape {
  readonly type = ShapeType.Heightfield;
  readonly doubleSided = false;
  readonly rows: number;
  readonly cols: number;
  readonly cellSizeX: number;
  readonly cellSizeZ: number;
  readonly heightScale: number;
  /** 原始高度（未缩放） */
  readonly heights: Float64Array;
  readonly triangleCount: number;
  /** 每个三角形的活跃棱标志 */
  readonly edgeFlags: Uint8Array;
  /** 局部坐标系中第 0 列 / 第 0 行的位置 */
  readonly originX: number;
  readonly originZ: number;

  constructor(options: HeightfieldOptions) {
    super();
    const { rows, cols } = options;
    if (!(rows >= 2 && cols >= 2) || !Number.isInteger(rows) || !Number.isInteger(cols)) {
      throw new Error('HeightfieldShape: rows 与 cols 必须是 ≥ 2 的整数');
    }
    if (options.heights.length !== rows * cols) {
      throw new Error(
        `HeightfieldShape: 需要 ${rows * cols} 个高度，实际 ${options.heights.length}`,
      );
    }
    const cell = options.cellSize ?? 1;
    this.rows = rows;
    this.cols = cols;
    this.cellSizeX = typeof cell === 'number' ? cell : cell.x;
    this.cellSizeZ = typeof cell === 'number' ? cell : cell.z;
    if (!(this.cellSizeX > 0 && this.cellSizeZ > 0)) {
      throw new Error('HeightfieldShape: cellSize 必须为正数');
    }
    this.heightScale = options.heightScale ?? 1;
    this.heights = Float64Array.from(options.heights);
    this.originX = (-(cols - 1) * this.cellSizeX) / 2;
    this.originZ = (-(rows - 1) * this.cellSizeZ) / 2;
    this.triangleCount = (rows - 1) * (cols - 1) * 2;

    let minY = Infinity;
    let maxY = -Infinity;
    for (let i = 0; i < this.heights.length; i++) {
      const y = this.heights[i]! * this.heightScale;
      if (!Number.isFinite(y)) throw new Error('HeightfieldShape: 高度必须是有限数');
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }
    this.localBounds.min.set(this.originX, minY, this.originZ);
    this.localBounds.max.set(-this.originX, maxY, -this.originZ);
    this.edgeFlags = new Uint8Array(this.triangleCount);
    for (let t = 0; t < this.triangleCount; t++) this.edgeFlags[t] = this.computeEdgeFlags(t);
  }

  /** 第 row 行、第 col 列采样点的局部坐标 */
  getVertex(row: number, col: number, out: Vec3): Vec3 {
    return out.set(
      this.originX + col * this.cellSizeX,
      this.heights[row * this.cols + col]! * this.heightScale,
      this.originZ + row * this.cellSizeZ,
    );
  }

  getTriangle(index: number, a: Vec3, b: Vec3, c: Vec3): void {
    const cell = index >> 1;
    const cc = this.cols - 1;
    const row = (cell / cc) | 0;
    const col = cell - row * cc;
    // 三角形 0：(c, r) → (c, r+1) → (c+1, r+1)；三角形 1：(c, r) → (c+1, r+1) → (c+1, r)
    this.getVertex(row, col, a);
    if ((index & 1) === 0) {
      this.getVertex(row + 1, col, b);
      this.getVertex(row + 1, col + 1, c);
    } else {
      this.getVertex(row + 1, col + 1, b);
      this.getVertex(row, col + 1, c);
    }
  }

  getEdgeFlags(index: number): number {
    return this.edgeFlags[index]!;
  }

  /**
   * 局部坐标 (x, z) 处的地面高度（按所在三角形插值），超出范围返回 NaN。
   */
  heightAt(x: number, z: number): number {
    const fx = (x - this.originX) / this.cellSizeX;
    const fz = (z - this.originZ) / this.cellSizeZ;
    if (!(fx >= 0 && fz >= 0 && fx <= this.cols - 1 && fz <= this.rows - 1)) return NaN;
    const col = Math.min(Math.floor(fx), this.cols - 2);
    const row = Math.min(Math.floor(fz), this.rows - 2);
    const u = fx - col;
    const v = fz - row;
    const s = this.heightScale;
    const h = this.heights;
    const i = row * this.cols + col;
    const h00 = h[i]! * s;
    const h11 = h[i + this.cols + 1]! * s;
    // 对角线 u = v 的哪一侧
    if (v >= u) {
      const h01 = h[i + this.cols]! * s;
      return h00 + u * (h11 - h01) + v * (h01 - h00);
    }
    const h10 = h[i + 1]! * s;
    return h00 + u * (h10 - h00) + v * (h11 - h10);
  }

  queryTriangles(localAabb: Readonly<AABB>, callback: (index: number) => boolean | void): void {
    const b = this.localBounds;
    const mn = localAabb.min;
    const mx = localAabb.max;
    if (mx.y < b.min.y || mn.y > b.max.y) return;
    const c0 = Math.max(0, Math.floor((mn.x - this.originX) / this.cellSizeX));
    const c1 = Math.min(this.cols - 2, Math.floor((mx.x - this.originX) / this.cellSizeX));
    const r0 = Math.max(0, Math.floor((mn.z - this.originZ) / this.cellSizeZ));
    const r1 = Math.min(this.rows - 2, Math.floor((mx.z - this.originZ) / this.cellSizeZ));
    const h = this.heights;
    const s = this.heightScale;
    const cols = this.cols;
    for (let r = r0; r <= r1; r++) {
      for (let c = c0; c <= c1; c++) {
        // 用格子四角高度做一次竖直方向的剔除
        const i = r * cols + c;
        const y0 = h[i]! * s,
          y1 = h[i + 1]! * s,
          y2 = h[i + cols]! * s,
          y3 = h[i + cols + 1]! * s;
        if (Math.max(y0, y1, y2, y3) < mn.y || Math.min(y0, y1, y2, y3) > mx.y) continue;
        const tri = (r * (cols - 1) + c) * 2;
        if (callback(tri) === false) return;
        if (callback(tri + 1) === false) return;
      }
    }
  }

  raycast(origin: Readonly<Vec3>, dir: Readonly<Vec3>, maxT: number, out: ShapeRayHit): boolean {
    // 先把射线裁剪到包围盒
    const b = this.localBounds;
    let tEnter = 0;
    let tExit = maxT;
    for (let i = 0; i < 3; i++) {
      const o = i === 0 ? origin.x : i === 1 ? origin.y : origin.z;
      const d = i === 0 ? dir.x : i === 1 ? dir.y : dir.z;
      const lo = i === 0 ? b.min.x : i === 1 ? b.min.y : b.min.z;
      const hi = i === 0 ? b.max.x : i === 1 ? b.max.y : b.max.z;
      if (d === 0) {
        if (o < lo || o > hi) return false;
        continue;
      }
      let t0 = (lo - o) / d;
      let t1 = (hi - o) / d;
      if (t0 > t1) {
        const t = t0;
        t0 = t1;
        t1 = t;
      }
      if (t0 > tEnter) tEnter = t0;
      if (t1 < tExit) tExit = t1;
      if (tEnter > tExit) return false;
    }

    // 二维 DDA（Amanatides & Woo）逐格前进
    const sx = this.cellSizeX;
    const sz = this.cellSizeZ;
    const px = origin.x + dir.x * tEnter - this.originX;
    const pz = origin.z + dir.z * tEnter - this.originZ;
    let col = Math.min(Math.max(Math.floor(px / sx), 0), this.cols - 2);
    let row = Math.min(Math.max(Math.floor(pz / sz), 0), this.rows - 2);
    const stepC = dir.x > 0 ? 1 : dir.x < 0 ? -1 : 0;
    const stepR = dir.z > 0 ? 1 : dir.z < 0 ? -1 : 0;
    const dtX = stepC !== 0 ? sx / Math.abs(dir.x) : Infinity;
    const dtZ = stepR !== 0 ? sz / Math.abs(dir.z) : Infinity;
    let tNextX =
      stepC > 0
        ? tEnter + ((col + 1) * sx - px) / dir.x
        : stepC < 0
          ? tEnter + (col * sx - px) / dir.x
          : Infinity;
    let tNextZ =
      stepR > 0
        ? tEnter + ((row + 1) * sz - pz) / dir.z
        : stepR < 0
          ? tEnter + (row * sz - pz) / dir.z
          : Infinity;

    const cc = this.cols - 1;
    for (;;) {
      let best = Infinity;
      for (let k = 0; k < 2; k++) {
        this.getTriangle((row * cc + col) * 2 + k, va, vb, vc);
        if (rayTriangle(origin, dir, va, vb, vc, tExit, false, hit) && hit.t < best) {
          best = hit.t;
          out.t = hit.t;
          out.normal.copy(hit.normal);
        }
      }
      if (best < Infinity) return true;
      if (tNextX < tNextZ) {
        if (tNextX > tExit) return false;
        col += stepC;
        tNextX += dtX;
        if (col < 0 || col >= cc) return false;
      } else {
        if (tNextZ > tExit || tNextZ === Infinity) return false;
        row += stepR;
        tNextZ += dtZ;
        if (row < 0 || row >= this.rows - 1) return false;
      }
    }
  }

  /** 三角形 tri 的第 edge 条棱的相邻三角形与其对顶点（行、列），边界返回 null */
  private neighbor(tri: number, edge: number): [number, number, number] | null {
    const cell = tri >> 1;
    const cc = this.cols - 1;
    const row = (cell / cc) | 0;
    const col = cell - row * cc;
    const rows = this.rows - 1;
    const idx = (r: number, c: number, k: number): number => (r * cc + c) * 2 + k;
    if ((tri & 1) === 0) {
      if (edge === 0) return col > 0 ? [idx(row, col - 1, 1), row, col - 1] : null;
      if (edge === 1) return row + 1 < rows ? [idx(row + 1, col, 1), row + 2, col + 1] : null;
      return [idx(row, col, 1), row, col + 1];
    }
    if (edge === 0) return [idx(row, col, 0), row + 1, col];
    if (edge === 1) return col + 1 < cc ? [idx(row, col + 1, 0), row + 1, col + 2] : null;
    return row > 0 ? [idx(row - 1, col, 0), row - 1, col] : null;
  }

  private computeEdgeFlags(tri: number): number {
    this.getTriangle(tri, va, vb, vc);
    n0.subVectors(vb, va).cross(n1.subVectors(vc, va));
    n0.normalize();
    corners[0]!.copy(va);
    corners[1]!.copy(vb);
    corners[2]!.copy(vc);
    let flags = 0;
    for (let e = 0; e < 3; e++) {
      const nb = this.neighbor(tri, e);
      if (!nb) {
        flags |= EDGE_AB << e;
        continue;
      }
      this.getTriangle(nb[0], va, vb, vc);
      n1.subVectors(vb, va).cross(vc.sub(va));
      n1.normalize();
      this.getVertex(nb[1], nb[2], opp);
      if (isEdgeActive(n0, corners[e]!, opp, n1)) flags |= EDGE_AB << e;
    }
    return flags;
  }
}
