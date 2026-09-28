import { AABB } from '../../math/AABB';
import { Transform } from '../../math/Transform';
import { Vec3 } from '../../math/Vec3';
import { ConvexPolyhedron } from '../../shapes/ConvexPolyhedron';
import type { MeshShape } from '../../shapes/MeshShape';
import { EDGE_AB } from '../../shapes/meshUtils';
import { type Shape, ShapeType } from '../../shapes/Shape';
import { ContactBuffer, Manifold } from '../Manifold';
import {
  type CollideConfig,
  collideHullSegment,
  collideHullsPrepared,
  prepareHullB,
} from './Collide';

/**
 * 三角网格 / 高度场与凸形状的接触生成。
 *
 * 1. 把凸形状变换到网格局部坐标系，按其包围盒查询三角形
 * 2. 每个三角形当作“扁平多面体”，复用凸体的 SAT + 裁剪（多面体）或 GJK + SAT（球 / 胶囊）
 * 3. 单面网格丢弃来自背面的接触
 * 4. 内部棱修正（ghost collision）：法线不是三角形面法线、且接触点附近的棱都不活跃（平坦或内凹）时，
 *    把法线改成面法线并重新计算分离距离，避免物体滑过三角形接缝时被绊住
 * 5. 法线相近（约 5°）的接触点合并到同一流形，每个流形约减到 4 个点
 */

/** 可容纳多个流形的接触输出（Contact 实现了该接口） */
export interface ManifoldList {
  readonly manifolds: Manifold[];
  manifoldCount: number;
}

/** 与一个网格接触时最多生成的流形数 */
export const MAX_MESH_MANIFOLDS = 8;

/** 法线夹角小于约 5° 的接触合并为一个流形（法线取平均），减少起伏地形上的约束数量 */
const MERGE_COS = Math.cos((5 * Math.PI) / 180);
/** 法线与面法线夹角小于约 0.8° 时视为面接触，无需修正 */
const FACE_COS = 0.9999;
/** 判断点是否投影在三角形内的容差（米）：公共棱上的点至少会被一侧的三角形接受 */
const INSIDE_TOLERANCE = 1e-6;

interface Group {
  /** 平均法线（单位向量） */
  readonly normal: Vec3;
  /** 组内法线之和 */
  readonly sum: Vec3;
  readonly buffer: ContactBuffer;
}

const groups: Group[] = [];
let groupCount = 0;

const triHull = ConvexPolyhedron.createTriangle();
const triManifold = new Manifold();
const IDENTITY = new Transform();
const relXf = new Transform();
const localAabb = new AABB();
const segP = new Vec3();
const segQ = new Vec3();
const va = new Vec3();
const vb = new Vec3();
const vc = new Vec3();
const faceN = new Vec3();
const pTri = new Vec3();
const pOther = new Vec3();
const worldN = new Vec3();
const worldP = new Vec3();
const tmp = new Vec3();
const cullN = new Vec3();
const e1 = new Vec3();
const localVerts: Vec3[] = [];

function isMesh(shape: Shape): boolean {
  return shape.type === ShapeType.TriMesh || shape.type === ShapeType.Heightfield;
}

const closest = new Vec3();

/**
 * 球（球心 c、半径 r）与当前三角形 (va, vb, vc) 的接触：三角形上离球心最近的点
 * （Ericson《Real-Time Collision Detection》5.1.5），比通用 GJK 快得多。
 */
function collideSphereTriangle(c: Readonly<Vec3>, r: number, m: Manifold, spec: number): void {
  const abx = vb.x - va.x,
    aby = vb.y - va.y,
    abz = vb.z - va.z;
  const acx = vc.x - va.x,
    acy = vc.y - va.y,
    acz = vc.z - va.z;
  const apx = c.x - va.x,
    apy = c.y - va.y,
    apz = c.z - va.z;
  const d1 = abx * apx + aby * apy + abz * apz;
  const d2 = acx * apx + acy * apy + acz * apz;
  const bpx = c.x - vb.x,
    bpy = c.y - vb.y,
    bpz = c.z - vb.z;
  const d3 = abx * bpx + aby * bpy + abz * bpz;
  const d4 = acx * bpx + acy * bpy + acz * bpz;
  const cpx = c.x - vc.x,
    cpy = c.y - vc.y,
    cpz = c.z - vc.z;
  const d5 = abx * cpx + aby * cpy + abz * cpz;
  const d6 = acx * cpx + acy * cpy + acz * cpz;
  const q = closest;
  const vcW = d1 * d4 - d3 * d2;
  const vbW = d5 * d2 - d1 * d6;
  const vaW = d3 * d6 - d5 * d4;
  if (d1 <= 0 && d2 <= 0) q.copy(va);
  else if (d3 >= 0 && d4 <= d3) q.copy(vb);
  else if (vcW <= 0 && d1 >= 0 && d3 <= 0) {
    const t = d1 / (d1 - d3);
    q.set(va.x + abx * t, va.y + aby * t, va.z + abz * t);
  } else if (d6 >= 0 && d5 <= d6) q.copy(vc);
  else if (vbW <= 0 && d2 >= 0 && d6 <= 0) {
    const t = d2 / (d2 - d6);
    q.set(va.x + acx * t, va.y + acy * t, va.z + acz * t);
  } else if (vaW <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const t = (d4 - d3) / (d4 - d3 + (d5 - d6));
    q.set(vb.x + (vc.x - vb.x) * t, vb.y + (vc.y - vb.y) * t, vb.z + (vc.z - vb.z) * t);
  } else {
    const inv = 1 / (vaW + vbW + vcW);
    const v = vbW * inv;
    const w = vcW * inv;
    q.set(va.x + abx * v + acx * w, va.y + aby * v + acy * w, va.z + abz * v + acz * w);
  }
  const n = m.normal;
  n.subVectors(c, q);
  let dist = n.normalize();
  if (dist < 1e-9) {
    // 球心恰在三角形上：取面法线
    n.copy(cullN);
    dist = 0;
  }
  const sep = dist - r;
  if (sep > spec) return;
  // 中点：三角形上的 q 与球面上的 c - n·r
  tmp.copy(q).addScaled(n, 0.5 * sep);
  m.addPoint(tmp, sep, 0);
}

/** 点到线段的距离平方 */
function distSqToSegment(p: Readonly<Vec3>, a: Readonly<Vec3>, b: Readonly<Vec3>): number {
  const abx = b.x - a.x,
    aby = b.y - a.y,
    abz = b.z - a.z;
  const apx = p.x - a.x,
    apy = p.y - a.y,
    apz = p.z - a.z;
  const len = abx * abx + aby * aby + abz * abz;
  let t = len > 0 ? (apx * abx + apy * aby + apz * abz) / len : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const dx = apx - abx * t,
    dy = apy - aby * t,
    dz = apz - abz * t;
  return dx * dx + dy * dy + dz * dz;
}

/** 接触点（三角形上的点）附近的棱，按 EDGE_* 位返回 */
function nearbyEdges(p: Readonly<Vec3>, tolSq: number): number {
  let bits = 0;
  if (distSqToSegment(p, va, vb) <= tolSq) bits |= EDGE_AB;
  if (distSqToSegment(p, vb, vc) <= tolSq) bits |= EDGE_AB << 1;
  if (distSqToSegment(p, vc, va) <= tolSq) bits |= EDGE_AB << 2;
  return bits;
}

/**
 * 点 p 沿面法线投影后是否落在三角形 (va, vb, vc) 内（允许 tol 的外扩）。
 * n 为按顶点顺序（逆时针）得到的单位法线。
 */
function insideTriangle(p: Readonly<Vec3>, n: Readonly<Vec3>, tol: number): boolean {
  return (
    edgeSide(va, vb, p, n) >= -tol &&
    edgeSide(vb, vc, p, n) >= -tol &&
    edgeSide(vc, va, p, n) >= -tol
  );
}

/** p 到有向棱 a→b 的平面内有符号距离（三角形内侧为正） */
function edgeSide(
  a: Readonly<Vec3>,
  b: Readonly<Vec3>,
  p: Readonly<Vec3>,
  n: Readonly<Vec3>,
): number {
  const ex = b.x - a.x,
    ey = b.y - a.y,
    ez = b.z - a.z;
  const px = p.x - a.x,
    py = p.y - a.y,
    pz = p.z - a.z;
  // ((b - a) × (p - a)) · n / |b - a|
  const cx = ey * pz - ez * py,
    cy = ez * px - ex * pz,
    cz = ex * py - ey * px;
  return (cx * n.x + cy * n.y + cz * n.z) / Math.sqrt(ex * ex + ey * ey + ez * ez);
}

/**
 * 把接触点加入法线相近的组（组法线为组内法线的平均）。组内与已有点几乎重合的点
 * （相邻三角形在公共棱 / 顶点处各生成一次）只保留较深的一个。
 */
function addToGroup(
  normal: Readonly<Vec3>,
  point: Readonly<Vec3>,
  sep: number,
  id: number,
  mergeSq: number,
): void {
  let best = -1;
  let bestDot = MERGE_COS;
  for (let g = 0; g < groupCount; g++) {
    const d = groups[g]!.normal.dot(normal);
    if (d > bestDot) {
      bestDot = d;
      best = g;
    }
  }
  if (best < 0) {
    if (groupCount < MAX_MESH_MANIFOLDS) {
      if (groups.length <= groupCount)
        groups.push({ normal: new Vec3(), sum: new Vec3(), buffer: new ContactBuffer() });
      best = groupCount++;
      groups[best]!.normal.copy(normal);
      groups[best]!.sum.setZero();
      groups[best]!.buffer.clear();
    } else {
      // 流形已满：并入法线最接近的一组
      bestDot = -Infinity;
      for (let g = 0; g < groupCount; g++) {
        const d = groups[g]!.normal.dot(normal);
        if (d > bestDot) {
          bestDot = d;
          best = g;
        }
      }
    }
  }
  const group = groups[best]!;
  const buf = group.buffer;
  group.sum.add(normal);
  group.normal.copy(group.sum);
  group.normal.normalize();
  for (let i = 0; i < buf.count; i++) {
    if (buf.points[i]!.distanceToSq(point) <= mergeSq) {
      if (sep < buf.separations[i]!) {
        buf.points[i]!.copy(point);
        buf.separations[i] = sep;
        buf.ids[i] = id;
      }
      return;
    }
  }
  buf.push(point, sep, id);
}

/**
 * 网格与凸形状碰撞。
 * @param flip 为 true 时网格是接触中的第二个碰撞体：输出法线由凸形状指向网格
 */
export function collideMesh(
  mesh: MeshShape,
  xfMesh: Readonly<Transform>,
  other: Shape,
  xfOther: Readonly<Transform>,
  flip: boolean,
  out: ManifoldList,
  config: CollideConfig,
): void {
  out.manifoldCount = 0;
  if (isMesh(other) || other.type === ShapeType.Plane) return;
  const spec = config.speculativeDistance;
  const tolSq = config.linearSlop * config.linearSlop;
  const mergeSq = 0.04 * tolSq;
  const isSphere = other.type === ShapeType.Sphere;
  const isSegmentKind = isSphere || other.type === ShapeType.Capsule;
  const doubleSided = mesh.doubleSided;

  relXf.multiplyInverseA(xfMesh, xfOther);
  other.computeAABB(relXf, localAabb);
  localAabb.expandByScalar(spec);
  const radius = other.radius;
  let nv = 0;
  if (isSegmentKind) {
    const core = other.getCorePoints();
    relXf.transformPoint(core[0]!, segP);
    if (core.length > 1) relXf.transformPoint(core[1]!, segQ);
    else segQ.copy(segP);
  } else {
    // 多面体只变换一次，所有三角形共用
    const hull = other.hull!;
    prepareHullB(hull, relXf);
    nv = hull.vertices.length;
    while (localVerts.length < nv) localVerts.push(new Vec3());
    for (let i = 0; i < nv; i++) relXf.transformPoint(hull.vertices[i]!, localVerts[i]!);
  }
  const bmin = localAabb.min;
  const bmax = localAabb.max;

  groupCount = 0;
  mesh.queryTriangles(localAabb, (tri) => {
    mesh.getTriangle(tri, va, vb, vc);
    // ---- 快速剔除：三角形包围盒、三角形平面 ----
    if (
      Math.min(va.x, vb.x, vc.x) > bmax.x ||
      Math.max(va.x, vb.x, vc.x) < bmin.x ||
      Math.min(va.y, vb.y, vc.y) > bmax.y ||
      Math.max(va.y, vb.y, vc.y) < bmin.y ||
      Math.min(va.z, vb.z, vc.z) > bmax.z ||
      Math.max(va.z, vb.z, vc.z) < bmin.z
    ) {
      return;
    }
    cullN.subVectors(vb, va).cross(e1.subVectors(vc, va));
    if (cullN.normalize() < 1e-12) return;
    const d = cullN.dot(va);
    let lo: number;
    let hi: number;
    if (isSegmentKind) {
      const hp = cullN.dot(segP) - d;
      const hq = cullN.dot(segQ) - d;
      lo = Math.min(hp, hq) - radius;
      hi = Math.max(hp, hq) + radius;
    } else {
      lo = Infinity;
      hi = -Infinity;
      for (let i = 0; i < nv; i++) {
        const h = cullN.dot(localVerts[i]!) - d;
        if (h < lo) lo = h;
        if (h > hi) hi = h;
      }
    }
    // 整体在正面推测距离之外，或（单面）整体在背面
    if (lo > spec || hi < (doubleSided ? -spec : 0)) return;

    triManifold.clear();
    if (isSphere) {
      collideSphereTriangle(segP, radius, triManifold, spec);
    } else {
      if (!triHull.setTriangle(va, vb, vc)) return;
      if (isSegmentKind) {
        collideHullSegment(triHull, IDENTITY, segP, segQ, radius, triManifold, config);
      } else {
        collideHullsPrepared(triHull, IDENTITY, other.hull!, triManifold, config);
      }
    }
    const count = triManifold.pointCount;
    if (count === 0) return;

    const n = triManifold.normal;
    // cullN 与三角形“多面体”的正面法线相同（按顶点顺序计算）
    faceN.copy(cullN);
    let dotN = n.dot(faceN);
    if (dotN < 0) {
      if (!doubleSided) return; // 单面：来自背面
      faceN.negate();
      dotN = -dotN;
    }

    // ---- 内部棱修正 ----
    let snap = false;
    if (dotN < FACE_COS) {
      const flags = mesh.getEdgeFlags(tri);
      let near = 0;
      for (let i = 0; i < count; i++) {
        const p = triManifold.points[i]!;
        // 接触点是两表面的中点，退回到三角形表面
        pTri.copy(p.point).addScaled(n, -0.5 * p.separation);
        near |= nearbyEdges(pTri, tolSq);
      }
      snap = (near & flags) === 0;
    }

    const idBase = Math.imul(tri + 1, 0x9e3779b1);
    if (snap) {
      // 接触特征是非活跃棱 / 顶点。取另一形状沿 -面法线 的最深点 pOther
      // （核心点 = 中点 + n·(s/2 + r)，再沿面法线下移半径 r）：
      // 其投影在本三角形内时改用面法线并重算分离距离；落在三角形外时丢弃，
      // 由相邻三角形的面接触负责（否则滚动的球会被侧向的推测接触点“绊”住）
      const windingN = cullN;
      const r = radius;
      xfMesh.transformVector(faceN, worldN);
      if (flip) worldN.negate();
      for (let i = 0; i < count; i++) {
        const p = triManifold.points[i]!;
        pOther
          .copy(p.point)
          .addScaled(n, 0.5 * p.separation + r)
          .addScaled(faceN, -r);
        if (!insideTriangle(pOther, windingN, INSIDE_TOLERANCE)) continue;
        const sep = tmp.subVectors(pOther, va).dot(faceN);
        if (sep > spec) continue;
        pOther.addScaled(faceN, -0.5 * sep);
        xfMesh.transformPoint(pOther, worldP);
        addToGroup(worldN, worldP, sep, (idBase ^ p.id) >>> 0, mergeSq);
      }
      return;
    }

    xfMesh.transformVector(n, worldN);
    if (flip) worldN.negate();
    for (let i = 0; i < count; i++) {
      const p = triManifold.points[i]!;
      xfMesh.transformPoint(p.point, worldP);
      addToGroup(worldN, worldP, p.separation, (idBase ^ p.id) >>> 0, mergeSq);
    }
  });

  // ---- 输出：每组约减到 4 个点 ----
  const manifolds = out.manifolds;
  let k = 0;
  for (let g = 0; g < groupCount; g++) {
    const group = groups[g]!;
    if (group.buffer.count === 0) continue;
    if (manifolds.length <= k) manifolds.push(new Manifold());
    const m = manifolds[k]!;
    m.clear();
    m.normal.copy(group.normal);
    group.buffer.reduceInto(m, m.normal);
    k++;
  }
  out.manifoldCount = k;
}
