import type { AABB } from '../math/AABB';
import { Vec3 } from '../math/Vec3';
import type { ShapeRayHit } from './Shape';

/** 棱的活跃标志位：bit0 = a→b，bit1 = b→c，bit2 = c→a */
export const EDGE_AB = 1;
export const EDGE_BC = 2;
export const EDGE_CA = 4;
export const ALL_EDGES_ACTIVE = EDGE_AB | EDGE_BC | EDGE_CA;

/** 两个相邻三角形法线夹角小于该值（约 3°）时，公共棱视为平坦 */
export const FLAT_EDGE_COS = Math.cos((3 * Math.PI) / 180);

const e1 = new Vec3();
const e2 = new Vec3();
const pv = new Vec3();
const tv = new Vec3();
const qv = new Vec3();

/**
 * 判断公共棱是否“活跃”（凸且足够尖锐）。
 * 平坦或内凹的棱不应产生侧向碰撞法线，否则物体滑过网格内部棱时会被“绊”一下（ghost collision）。
 *
 * @param normal 本三角形的单位法线
 * @param edgeStart 公共棱上的一点
 * @param opposite 相邻三角形中不在公共棱上的顶点
 * @param neighborNormal 相邻三角形的单位法线
 */
export function isEdgeActive(
  normal: Readonly<Vec3>,
  edgeStart: Readonly<Vec3>,
  opposite: Readonly<Vec3>,
  neighborNormal: Readonly<Vec3>,
): boolean {
  if (normal.dot(neighborNormal) > FLAT_EDGE_COS) return false; // 平坦
  // 相邻三角形的对顶点在本三角形平面之上 → 内凹
  const h = tv.subVectors(opposite, edgeStart).dot(normal);
  return h < 0;
}

/**
 * 射线与三角形求交（Möller–Trumbore）。
 * @param doubleSided 为 false 时只命中正面（逆时针为正面）
 */
export function rayTriangle(
  origin: Readonly<Vec3>,
  dir: Readonly<Vec3>,
  a: Readonly<Vec3>,
  b: Readonly<Vec3>,
  c: Readonly<Vec3>,
  maxT: number,
  doubleSided: boolean,
  out: ShapeRayHit,
): boolean {
  e1.subVectors(b, a);
  e2.subVectors(c, a);
  pv.crossVectors(dir, e2);
  const det = e1.dot(pv);
  if (doubleSided ? Math.abs(det) < 1e-14 : det < 1e-14) return false;
  const inv = 1 / det;
  tv.subVectors(origin, a);
  const u = tv.dot(pv) * inv;
  if (u < 0 || u > 1) return false;
  qv.crossVectors(tv, e1);
  const v = dir.dot(qv) * inv;
  if (v < 0 || u + v > 1) return false;
  const t = e2.dot(qv) * inv;
  if (t < 0 || t > maxT) return false;
  out.t = t;
  out.normal.crossVectors(e1, e2);
  out.normal.normalize();
  // 法线朝向射线来的一侧
  if (out.normal.dot(dir) > 0) out.normal.negate();
  return true;
}

/** 三角形的包围盒 */
export function triangleAabb(
  a: Readonly<Vec3>,
  b: Readonly<Vec3>,
  c: Readonly<Vec3>,
  out: AABB,
): AABB {
  out.min.set(Math.min(a.x, b.x, c.x), Math.min(a.y, b.y, c.y), Math.min(a.z, b.z, c.z));
  out.max.set(Math.max(a.x, b.x, c.x), Math.max(a.y, b.y, c.y), Math.max(a.z, b.z, c.z));
  return out;
}
