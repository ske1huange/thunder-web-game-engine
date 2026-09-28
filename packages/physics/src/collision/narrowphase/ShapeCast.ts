import type { Transform } from '../../math/Transform';
import { Vec3 } from '../../math/Vec3';
import { type Shape, ShapeType } from '../../shapes/Shape';
import { DistanceOutput, DistanceProxy, gjkDistance } from './GJK';

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

/**
 * 形状 A（初始变换 xfA）沿 translation 平移，对静止形状 B 做投射。支持平面作为 B。
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
): boolean {
  if (shapeA.type === ShapeType.Plane) return false;
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
