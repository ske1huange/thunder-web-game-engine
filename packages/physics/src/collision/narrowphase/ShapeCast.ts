import { AABB } from '../../math/AABB';
import { Transform } from '../../math/Transform';
import { Vec3 } from '../../math/Vec3';
import type { MeshShape } from '../../shapes/MeshShape';
import { type Shape, ShapeType } from '../../shapes/Shape';
import { DistanceOutput, DistanceProxy, gjkDistance } from './GJK';

/**
 * 连续碰撞检测对网格投射时的附加信息。
 *
 * 为避免高速物体在起伏的网格上“卡顿”（本步内接触到前方三角形而被回退），
 * 只有质心本步结束时距三角形平面不足 0.5 × minExtent（即将深度穿透或穿过）的三角形才参与投射；
 * 起点已接触（t = 0）时改用质心处半径 0.25 × minExtent 的小球投射（参考 Box2D v3）。
 */
export interface ContinuousCastInfo {
  /** 起点处刚体质心（世界坐标） */
  centroid: Readonly<Vec3>;
  /** 刚体最小半边长 */
  minExtent: number;
}

export class ShapeCastOutput {
  /** 命中参数（0..maxT），位移 = t * translation */
  t = 0;
  /** 命中时 B 表面上的点 */
  readonly point = new Vec3();
  /** 命中时由 A 指向 B 的法线 */
  readonly normal = new Vec3();
  iterations = 0;
}

const movedA = new DistanceProxy();
const gjkOut = new DistanceOutput();
const UP = new Vec3(0, 1, 0);
const tmp = new Vec3();
const planeN = new Vec3();

/**
 * 线性形状投射：代理 A 沿 translation 平移，B 静止。
 * 使用保守推进（conservative advancement）：每次用 GJK 求最近距离与法线，
 * 沿法线的接近速度推进 t，保证不会越过接触（参考 Box2D v3 b2ShapeCast）。
 *
 * @param target 希望停下时两表面的间距（一般取 linearSlop）
 * @returns 是否在 [0, maxT] 内命中
 */
export function shapeCastProxies(
  proxyA: DistanceProxy,
  translation: Readonly<Vec3>,
  proxyB: DistanceProxy,
  maxT: number,
  target: number,
  out: ShapeCastOutput,
): boolean {
  const totalRadius = proxyA.radius + proxyB.radius;
  const tolerance = Math.max(0.25 * target, 1e-5);
  let t = 0;
  movedA.setPoints(proxyA.points, 0, proxyA.count);
  for (let iter = 0; iter < 32; iter++) {
    out.iterations = iter + 1;
    gjkDistance(movedA, proxyB, false, gjkOut);
    const sep = gjkOut.distance - totalRadius;
    if (sep < target + tolerance) {
      out.t = t;
      if (gjkOut.distance > 1e-12) {
        out.normal.copy(gjkOut.normal);
      } else {
        // 初始重叠：法线取平移方向
        out.normal.copy(translation);
        out.normal.normalize();
      }
      out.point.copy(gjkOut.pointB).addScaled(out.normal, -proxyB.radius);
      return true;
    }
    const approach = translation.dot(gjkOut.normal);
    if (approach <= 1e-12) return false;
    const dt = (sep - target) / approach;
    t += dt;
    if (t > maxT) return false;
    tmp.copy(translation).scale(dt);
    movedA.translate(tmp);
  }
  // 未收敛：保守地报告当前 t
  out.t = t;
  out.normal.copy(gjkOut.normal);
  out.point.copy(gjkOut.pointB);
  return true;
}

const castProxyA = new DistanceProxy();
const castProxyB = new DistanceProxy();

function isMesh(shape: Shape): boolean {
  return shape.type === ShapeType.TriMesh || shape.type === ShapeType.Heightfield;
}

/**
 * 形状 A（初始变换 xfA）沿 translation 平移，对静止形状 B 做投射。
 * 支持平面、三角网格与高度场作为 B（单面网格只会被正面命中）。
 */
export function shapeCast(
  shapeA: Shape,
  xfA: Readonly<Transform>,
  translation: Readonly<Vec3>,
  shapeB: Shape,
  xfB: Readonly<Transform>,
  maxT: number,
  target: number,
  out: ShapeCastOutput,
  continuous?: ContinuousCastInfo,
): boolean {
  if (shapeA.type === ShapeType.Plane || isMesh(shapeA)) return false;
  if (isMesh(shapeB)) {
    return castAgainstMesh(
      shapeA,
      xfA,
      translation,
      shapeB as MeshShape,
      xfB,
      maxT,
      target,
      out,
      continuous,
    );
  }
  if (shapeB.type === ShapeType.Plane) {
    // 平面：分离距离关于 t 线性，直接求解
    xfB.rotation.rotate(UP, planeN);
    const core = shapeA.getCorePoints();
    let minH = Infinity;
    let deepest = 0;
    for (let i = 0; i < core.length; i++) {
      xfA.transformPoint(core[i]!, tmp);
      const h = tmp.sub(xfB.position).dot(planeN);
      if (h < minH) {
        minH = h;
        deepest = i;
      }
    }
    const sep = minH - shapeA.radius;
    const rate = translation.dot(planeN);
    let t: number;
    if (sep <= target) t = 0;
    else {
      if (rate >= 0) return false;
      t = (sep - target) / -rate;
      if (t > maxT) return false;
    }
    out.t = t;
    out.iterations = 1;
    out.normal.copy(planeN).negate();
    xfA.transformPoint(core[deepest]!, out.point).addScaled(translation, t);
    out.point.addScaled(planeN, -(minH + rate * t));
    return true;
  }
  castProxyA.setShape(shapeA, xfA);
  castProxyB.setShape(shapeB, xfB);
  return shapeCastProxies(castProxyA, translation, castProxyB, maxT, target, out);
}

const meshRel = new Transform();
const meshTrans = new Vec3();
const sweepAabb = new AABB();
const endAabb = new AABB();
const triPts = [new Vec3(), new Vec3(), new Vec3()];
const triProxy = new DistanceProxy();
const localProxy = new DistanceProxy();
const coreProxy = new DistanceProxy();
const triOut = new ShapeCastOutput();
const triN = new Vec3();
const e1 = new Vec3();
const e2 = new Vec3();
const cStart = new Vec3();
const cEnd = new Vec3();
const corePts = [new Vec3()];

function castAgainstMesh(
  shapeA: Shape,
  xfA: Readonly<Transform>,
  translation: Readonly<Vec3>,
  mesh: MeshShape,
  xfMesh: Readonly<Transform>,
  maxT: number,
  target: number,
  out: ShapeCastOutput,
  continuous: ContinuousCastInfo | undefined,
): boolean {
  // 在网格局部坐标系中计算
  meshRel.multiplyInverseA(xfMesh, xfA);
  xfMesh.inverseTransformVector(translation, meshTrans);
  shapeA.computeAABB(meshRel, sweepAabb);
  endAabb.copy(sweepAabb);
  tmp.copy(meshTrans).scale(maxT);
  endAabb.min.add(tmp);
  endAabb.max.add(tmp);
  sweepAabb.union(sweepAabb, endAabb).expandByScalar(target);
  localProxy.setShape(shapeA, meshRel);
  const doubleSided = mesh.doubleSided;
  if (continuous) {
    xfMesh.inverseTransformPoint(continuous.centroid, cStart);
    cEnd.copy(cStart).addScaled(meshTrans, maxT);
    corePts[0]!.copy(cStart);
    coreProxy.setPointsRef(corePts, 0.25 * continuous.minExtent, 1);
  }

  let best = maxT;
  let found = false;
  mesh.queryTriangles(sweepAabb, (tri) => {
    const a = triPts[0]!;
    const b = triPts[1]!;
    const c = triPts[2]!;
    mesh.getTriangle(tri, a, b, c);
    triN.crossVectors(e1.subVectors(b, a), e2.subVectors(c, a));
    if (triN.normalize() < 1e-12) return;
    let rate = triN.dot(meshTrans);
    if (rate > 0) {
      if (!doubleSided) return; // 单面：从背面穿过
      triN.negate();
      rate = -rate;
    }
    if (rate === 0) return;
    if (continuous) {
      const d1 = e1.subVectors(cStart, a).dot(triN);
      const d2 = e1.subVectors(cEnd, a).dot(triN);
      if (d1 < 0 || d2 >= 0.5 * continuous.minExtent) return;
    }
    triProxy.setPointsRef(triPts, 0, 3);
    if (!shapeCastProxies(localProxy, meshTrans, triProxy, best, target, triOut)) return;
    if (continuous && triOut.t === 0) {
      if (!shapeCastProxies(coreProxy, meshTrans, triProxy, best, target, triOut)) return;
      if (triOut.t === 0) return;
    }
    if (!found || triOut.t < best) {
      found = true;
      best = triOut.t;
      out.t = triOut.t;
      out.iterations = triOut.iterations;
      xfMesh.transformVector(triOut.normal, out.normal);
      xfMesh.transformPoint(triOut.point, out.point);
    }
  });
  return found;
}
