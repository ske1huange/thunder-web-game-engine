import type { BroadPhase } from '../collision/broadphase/BroadPhase';
import { testOverlap } from '../collision/narrowphase/Collide';
import { ShapeCastOutput, shapeCast } from '../collision/narrowphase/ShapeCast';
import type { Collider } from '../dynamics/Collider';
import type { RigidBody } from '../dynamics/RigidBody';
import { AABB } from '../math/AABB';
import { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import type { Shape, ShapeRayHit } from '../shapes/Shape';

/** 查询过滤条件（与 Box2D 相同：双方类别/掩码互相匹配） */
export interface QueryFilter {
  /** 查询自身的类别，默认 0x0001 */
  categoryBits?: number;
  /** 查询要命中的类别，默认全部 */
  maskBits?: number;
  /** 是否包含传感器，默认 false */
  includeSensors?: boolean;
  /** 排除某个刚体（例如角色自身） */
  excludeBody?: RigidBody;
  /** 自定义过滤，返回 false 表示忽略 */
  predicate?: (collider: Collider) => boolean;
}

export interface RaycastHit {
  collider: Collider;
  body: RigidBody;
  /** 世界坐标命中点 */
  point: Vec3;
  /** 世界坐标命中法线 */
  normal: Vec3;
  /** 距射线起点的距离 */
  distance: number;
}

export interface ShapeCastHit {
  collider: Collider;
  body: RigidBody;
  /** 命中比例（0..1），命中时位移 = fraction * translation */
  fraction: number;
  /** 命中点（世界坐标，位于被命中物体表面） */
  point: Vec3;
  /** 命中法线（由被投射形状指向被命中物体） */
  normal: Vec3;
}

export function passesFilter(collider: Collider, filter: QueryFilter | undefined): boolean {
  if (!filter) return !collider.isSensor;
  if (collider.isSensor && !filter.includeSensors) return false;
  if (filter.excludeBody && collider.body === filter.excludeBody) return false;
  const cat = filter.categoryBits ?? 0x0001;
  const mask = filter.maskBits ?? 0xffffffff;
  if ((collider.filter.categoryBits & mask) === 0 || (collider.filter.maskBits & cat) === 0) {
    return false;
  }
  if (filter.predicate && !filter.predicate(collider)) return false;
  return true;
}

const localOrigin = new Vec3();
const localDir = new Vec3();
const rayHit: ShapeRayHit = { t: 0, normal: new Vec3() };
const unitDir = new Vec3();

/** 对单个碰撞体做射线检测（dir 为单位向量，t 为距离） */
export function raycastCollider(
  collider: Collider,
  origin: Readonly<Vec3>,
  dir: Readonly<Vec3>,
  maxDistance: number,
): RaycastHit | null {
  const xf = collider.worldTransform;
  xf.inverseTransformPoint(origin, localOrigin);
  xf.inverseTransformVector(dir, localDir);
  if (!collider.shape.raycast(localOrigin, localDir, maxDistance, rayHit)) return null;
  const point = new Vec3().copy(origin).addScaled(dir, rayHit.t);
  const normal = xf.transformVector(rayHit.normal, new Vec3());
  return { collider, body: collider.body, point, normal, distance: rayHit.t };
}

/** 最近命中 */
export function raycastClosest(
  broadPhase: BroadPhase<Collider>,
  origin: Readonly<Vec3>,
  direction: Readonly<Vec3>,
  maxDistance: number,
  filter?: QueryFilter,
): RaycastHit | null {
  unitDir.copy(direction);
  if (unitDir.normalize() === 0) return null;
  const dir = unitDir.clone();
  let best: RaycastHit | null = null;
  broadPhase.raycast(origin, dir, maxDistance, (collider, maxT) => {
    if (!passesFilter(collider, filter)) return -1;
    const hit = raycastCollider(collider, origin, dir, maxT);
    if (!hit) return -1;
    if (!best || hit.distance < best.distance) best = hit;
    return hit.distance;
  });
  return best;
}

/** 所有命中，按距离排序 */
export function raycastAll(
  broadPhase: BroadPhase<Collider>,
  origin: Readonly<Vec3>,
  direction: Readonly<Vec3>,
  maxDistance: number,
  filter?: QueryFilter,
): RaycastHit[] {
  unitDir.copy(direction);
  if (unitDir.normalize() === 0) return [];
  const dir = unitDir.clone();
  const hits: RaycastHit[] = [];
  broadPhase.raycast(origin, dir, maxDistance, (collider) => {
    if (!passesFilter(collider, filter)) return -1;
    const hit = raycastCollider(collider, origin, dir, maxDistance);
    if (hit) hits.push(hit);
    return -1;
  });
  hits.sort((a, b) => a.distance - b.distance);
  return hits;
}

/** 与 AABB 相交（按扩展包围盒粗筛，再用紧致包围盒精确判断）的碰撞体 */
export function queryAABB(
  broadPhase: BroadPhase<Collider>,
  aabb: Readonly<AABB>,
  filter?: QueryFilter,
): Collider[] {
  const result: Collider[] = [];
  broadPhase.query(aabb, (collider) => {
    if (passesFilter(collider, filter) && collider.aabb.overlaps(aabb)) result.push(collider);
    return true;
  });
  return result;
}

const shapeAabb = new AABB();

/** 与给定形状（世界变换）重叠的碰撞体 */
export function overlapShape(
  broadPhase: BroadPhase<Collider>,
  shape: Shape,
  transform: Readonly<Transform>,
  filter?: QueryFilter,
): Collider[] {
  shape.computeAABB(transform, shapeAabb);
  const result: Collider[] = [];
  broadPhase.query(shapeAabb, (collider) => {
    if (!passesFilter(collider, filter)) return true;
    if (!collider.aabb.overlaps(shapeAabb)) return true;
    if (testOverlap(shape, transform, collider.shape, collider.worldTransform)) result.push(collider);
    return true;
  });
  return result;
}

const castOut = new ShapeCastOutput();
const startAabb = new AABB();
const endAabb = new AABB();
const sweepAabb = new AABB();
const endXf = new Transform();

/** 形状沿 translation 平移，返回最先命中的碰撞体 */
export function castShape(
  broadPhase: BroadPhase<Collider>,
  shape: Shape,
  transform: Readonly<Transform>,
  translation: Readonly<Vec3>,
  filter: QueryFilter | undefined,
  linearSlop: number,
): ShapeCastHit | null {
  shape.computeAABB(transform, startAabb);
  endXf.copy(transform);
  endXf.position.add(translation);
  shape.computeAABB(endXf, endAabb);
  sweepAabb.union(startAabb, endAabb);
  let best: ShapeCastHit | null = null;
  let maxT = 1;
  broadPhase.query(sweepAabb, (collider) => {
    if (!passesFilter(collider, filter)) return true;
    if (shapeCast(shape, transform, translation, collider.shape, collider.worldTransform, maxT, linearSlop, castOut)) {
      if (!best || castOut.t < best.fraction) {
        maxT = castOut.t;
        best = {
          collider,
          body: collider.body,
          fraction: castOut.t,
          point: castOut.point.clone(),
          normal: castOut.normal.clone(),
        };
      }
    }
    return true;
  });
  return best;
}
