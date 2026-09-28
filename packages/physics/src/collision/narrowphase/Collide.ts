import { Transform } from '../../math/Transform';
import { Vec3 } from '../../math/Vec3';
import type { ConvexPolyhedron } from '../../shapes/ConvexPolyhedron';
import { type Shape, ShapeType } from '../../shapes/Shape';
import { ContactBuffer, type Manifold } from '../Manifold';
import { DistanceOutput, DistanceProxy, gjkDistance } from './GJK';

/**
 * 窄相碰撞：按形状类别分派到专用的接触生成函数。
 *
 * 形状类别：平面 < 线段类（球 = 退化线段，胶囊）< 多面体（长方体/圆柱/凸包）。
 * 所有生成函数输出的法线都由第一个形状指向第二个形状，分离距离为负表示穿透。
 * 分离距离小于 speculativeDistance 的点也会输出（推测接触，参考 Box2D v3）。
 */

export interface CollideConfig {
  /** 推测接触距离：分离小于该值即生成接触点 */
  speculativeDistance: number;
  /** 线性容差 */
  linearSlop: number;
}

const enum Kind {
  Plane = 0,
  Segment = 1,
  Hull = 2,
}

function kindOf(shape: Shape): Kind {
  switch (shape.type) {
    case ShapeType.Plane:
      return Kind.Plane;
    case ShapeType.Sphere:
    case ShapeType.Capsule:
      return Kind.Segment;
    default:
      return Kind.Hull;
  }
}

// ---------------------------------------------------------------------------
// 临时对象
// ---------------------------------------------------------------------------
const buffer = new ContactBuffer();
const segA0 = new Vec3();
const segA1 = new Vec3();
const segB0 = new Vec3();
const segB1 = new Vec3();
const planeN = new Vec3();
const c1 = new Vec3();
const c2 = new Vec3();
const t0 = new Vec3();
const t1 = new Vec3();
const t2 = new Vec3();
const t3 = new Vec3();
const t4 = new Vec3();
const UP = new Vec3(0, 1, 0);

/**
 * 两形状碰撞，结果写入 manifold（会先清空）。
 */
export function collideShapes(
  shapeA: Shape,
  xfA: Readonly<Transform>,
  shapeB: Shape,
  xfB: Readonly<Transform>,
  manifold: Manifold,
  config: CollideConfig,
): void {
  manifold.clear();
  const kA = kindOf(shapeA);
  const kB = kindOf(shapeB);
  if (kA > kB) {
    collideOrdered(shapeB, xfB, kB, shapeA, xfA, kA, manifold, config);
    manifold.normal.negate();
  } else {
    collideOrdered(shapeA, xfA, kA, shapeB, xfB, kB, manifold, config);
  }
}

function collideOrdered(
  shapeA: Shape,
  xfA: Readonly<Transform>,
  kA: Kind,
  shapeB: Shape,
  xfB: Readonly<Transform>,
  kB: Kind,
  manifold: Manifold,
  config: CollideConfig,
): void {
  if (kA === Kind.Plane) {
    if (kB === Kind.Plane) return;
    xfA.rotation.rotate(UP, planeN);
    if (kB === Kind.Segment) {
      segmentOf(shapeB, xfB, segB0, segB1);
      collidePlaneSegment(planeN, xfA.position, segB0, segB1, shapeB.radius, manifold, config);
    } else {
      collidePlaneHull(planeN, xfA.position, shapeB.hull!, xfB, manifold, config);
    }
    return;
  }
  if (kA === Kind.Segment) {
    segmentOf(shapeA, xfA, segA0, segA1);
    if (kB === Kind.Segment) {
      segmentOf(shapeB, xfB, segB0, segB1);
      collideSegments(segA0, segA1, shapeA.radius, segB0, segB1, shapeB.radius, manifold, config);
    } else {
      // 多面体为第一参数，结果翻转
      collideHullSegment(shapeB.hull!, xfB, segA0, segA1, shapeA.radius, manifold, config);
      manifold.normal.negate();
    }
    return;
  }
  collideHulls(shapeA.hull!, xfA, shapeB.hull!, xfB, manifold, config);
}

/** 线段类形状的世界坐标端点（球的两个端点重合） */
function segmentOf(shape: Shape, xf: Readonly<Transform>, p: Vec3, q: Vec3): void {
  const core = shape.getCorePoints();
  xf.transformPoint(core[0]!, p);
  if (core.length > 1) xf.transformPoint(core[1]!, q);
  else q.copy(p);
}

// ---------------------------------------------------------------------------
// 线段工具
// ---------------------------------------------------------------------------

/**
 * 两线段最近点（Ericson 5.1.9）。结果写入 outA、outB，返回参数 s、t。
 */
export function closestPointsSegments(
  p1: Readonly<Vec3>,
  q1: Readonly<Vec3>,
  p2: Readonly<Vec3>,
  q2: Readonly<Vec3>,
  outA: Vec3,
  outB: Vec3,
): void {
  const d1x = q1.x - p1.x,
    d1y = q1.y - p1.y,
    d1z = q1.z - p1.z;
  const d2x = q2.x - p2.x,
    d2y = q2.y - p2.y,
    d2z = q2.z - p2.z;
  const rx = p1.x - p2.x,
    ry = p1.y - p2.y,
    rz = p1.z - p2.z;
  const a = d1x * d1x + d1y * d1y + d1z * d1z;
  const e = d2x * d2x + d2y * d2y + d2z * d2z;
  const f = d2x * rx + d2y * ry + d2z * rz;
  const EPS = 1e-12;
  let s: number, t: number;
  if (a <= EPS && e <= EPS) {
    s = 0;
    t = 0;
  } else if (a <= EPS) {
    s = 0;
    t = clamp01(f / e);
  } else {
    const c = d1x * rx + d1y * ry + d1z * rz;
    if (e <= EPS) {
      t = 0;
      s = clamp01(-c / a);
    } else {
      const b = d1x * d2x + d1y * d2y + d1z * d2z;
      const denom = a * e - b * b;
      s = denom > EPS * a * e ? clamp01((b * f - c * e) / denom) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = clamp01(-c / a);
      } else if (t > 1) {
        t = 1;
        s = clamp01((b - c) / a);
      }
    }
  }
  outA.set(p1.x + d1x * s, p1.y + d1y * s, p1.z + d1z * s);
  outB.set(p2.x + d2x * t, p2.y + d2y * t, p2.z + d2z * t);
}

function clamp01(x: number): number {
  return x < 0 ? 0 : x > 1 ? 1 : x;
}

/** 为零距离情形挑选一个稳定的法线方向 */
function fallbackNormal(u1: Readonly<Vec3>, u2: Readonly<Vec3>, out: Vec3): Vec3 {
  const l1 = u1.lengthSq();
  const l2 = u2.lengthSq();
  if (l1 > 1e-12 && l2 > 1e-12) {
    out.crossVectors(u1, u2);
    if (out.normalize() > 1e-9) return out;
  }
  if (l1 > 1e-12) {
    t4.copy(u1);
    t4.normalize();
    return t4.perpendicular(out);
  }
  if (l2 > 1e-12) {
    t4.copy(u2);
    t4.normalize();
    return t4.perpendicular(out);
  }
  return out.set(0, 1, 0);
}

// ---------------------------------------------------------------------------
// 线段类 vs 线段类（球-球、球-胶囊、胶囊-胶囊）
// ---------------------------------------------------------------------------

export function collideSegments(
  p1: Readonly<Vec3>,
  q1: Readonly<Vec3>,
  rA: number,
  p2: Readonly<Vec3>,
  q2: Readonly<Vec3>,
  rB: number,
  manifold: Manifold,
  config: CollideConfig,
): void {
  closestPointsSegments(p1, q1, p2, q2, c1, c2);
  const normal = manifold.normal;
  normal.subVectors(c2, c1);
  const dist = normal.normalize();
  const sep = dist - rA - rB;
  if (sep > config.speculativeDistance) return;

  const u1 = t0.subVectors(q1, p1);
  const u2 = t1.subVectors(q2, p2);
  if (dist < 1e-9) fallbackNormal(u1, u2, normal);

  // 近似平行的两根胶囊：生成两个接触点，避免绕接触点来回翻滚
  const lenA = u1.length();
  const lenB = u2.length();
  if (lenA > config.linearSlop && lenB > config.linearSlop) {
    const uA = u1.scale(1 / lenA);
    const uB = t2.copy(u2).scale(1 / lenB);
    if (Math.abs(uA.dot(uB)) > 0.995) {
      const sP = t3.subVectors(p2, p1).dot(uA);
      const sQ = t3.subVectors(q2, p1).dot(uA);
      const lo = Math.max(0, Math.min(sP, sQ));
      const hi = Math.min(lenA, Math.max(sP, sQ));
      if (hi - lo > config.linearSlop) {
        const start = manifold.pointCount;
        for (let i = 0; i < 2; i++) {
          const s = i === 0 ? lo : hi;
          const tt = clamp01((s - sP) / (sQ - sP));
          const pb = t3.copy(p2).addScaled(u2, tt);
          const pa = t4.copy(p1).addScaled(uA, s);
          const sepI = t2.subVectors(pb, pa).dot(normal) - rA - rB;
          if (sepI <= config.speculativeDistance) {
            // 中点：A 表面 pa + n rA 与 B 表面 pb - n rB
            pa.addScaled(normal, rA).add(pb.addScaled(normal, -rB)).scale(0.5);
            manifold.addPoint(pa, sepI, i + 1);
          }
        }
        if (manifold.pointCount > start) return;
      }
    }
  }

  c1.addScaled(normal, rA);
  c2.addScaled(normal, -rB);
  c1.add(c2).scale(0.5);
  manifold.addPoint(c1, sep, 0);
}

// ---------------------------------------------------------------------------
// 平面
// ---------------------------------------------------------------------------

function collidePlaneSegment(
  n: Readonly<Vec3>,
  p0: Readonly<Vec3>,
  p: Readonly<Vec3>,
  q: Readonly<Vec3>,
  r: number,
  manifold: Manifold,
  config: CollideConfig,
): void {
  manifold.normal.copy(n);
  const count = p.equals(q) ? 1 : 2;
  for (let i = 0; i < count; i++) {
    const c = i === 0 ? p : q;
    const sep = t0.subVectors(c, p0).dot(n) - r;
    if (sep <= config.speculativeDistance) {
      t1.copy(c).addScaled(n, -(r + sep * 0.5));
      manifold.addPoint(t1, sep, i + 1);
    }
  }
}

function collidePlaneHull(
  n: Readonly<Vec3>,
  p0: Readonly<Vec3>,
  hull: ConvexPolyhedron,
  xf: Readonly<Transform>,
  manifold: Manifold,
  config: CollideConfig,
): void {
  manifold.normal.copy(n);
  buffer.clear();
  const verts = hull.vertices;
  for (let i = 0; i < verts.length; i++) {
    const v = xf.transformPoint(verts[i]!, t0);
    const sep = t1.subVectors(v, p0).dot(n);
    if (sep <= config.speculativeDistance) {
      v.addScaled(n, -sep * 0.5);
      buffer.push(v, sep, i + 1);
    }
  }
  buffer.reduceInto(manifold, n);
}

// ---------------------------------------------------------------------------
// 多面体 vs 线段类（长方体/凸包 与 球/胶囊）
// ---------------------------------------------------------------------------

const hullProxy = new DistanceProxy();
const segProxy = new DistanceProxy();
const gjkOut = new DistanceOutput();
const segLocal = [new Vec3(), new Vec3()];
const clipA = new Vec3();
const clipB = new Vec3();
const localNormal = new Vec3();
const edgeAxis = new Vec3();

/** 用多面体面 face 的侧平面裁剪线段 [a, b]（局部坐标），全部在外侧时返回 false */
function clipSegmentToFace(hull: ConvexPolyhedron, faceIndex: number, a: Vec3, b: Vec3): boolean {
  const face = hull.faces[faceIndex]!;
  const idx = face.indices;
  const n = face.normal;
  for (let i = 0; i < idx.length; i++) {
    const v0 = hull.vertices[idx[i]!]!;
    const v1 = hull.vertices[idx[(i + 1) % idx.length]!]!;
    const side = t2.subVectors(v1, v0).cross(n); // 指向面外侧
    const offset = side.dot(v0);
    const da = side.dot(a) - offset;
    const db = side.dot(b) - offset;
    if (da > 0 && db > 0) return false;
    if (da > 0) {
      const t = da / (da - db);
      a.lerpVectors(a, b, t);
    } else if (db > 0) {
      const t = db / (db - da);
      b.lerpVectors(b, a, t);
    }
  }
  return true;
}

/** 多面体（A）与线段类（B）碰撞，法线由多面体指向线段 */
export function collideHullSegment(
  hull: ConvexPolyhedron,
  xf: Readonly<Transform>,
  pw: Readonly<Vec3>,
  qw: Readonly<Vec3>,
  radius: number,
  manifold: Manifold,
  config: CollideConfig,
): void {
  const spec = config.speculativeDistance;
  const p = xf.inverseTransformPoint(pw, segLocal[0]!);
  const q = xf.inverseTransformPoint(qw, segLocal[1]!);
  const isSegment = p.distanceToSq(q) > 1e-12;

  hullProxy.setPointsRef(hull.vertices, 0);
  segProxy.setPointsRef(segLocal, 0, isSegment ? 2 : 1);
  const out = gjkDistance(hullProxy, segProxy, false, gjkOut);
  if (out.distance > radius + spec) return;

  const n = localNormal;
  buffer.clear();

  if (out.distance > 0.1 * config.linearSlop) {
    // ---- 核心分离：GJK 法线 ----
    n.copy(out.normal);
    if (isSegment) {
      // 若最近特征是一个面（线段平躺在面上），裁剪得到两个接触点
      let bestFace = 0;
      let bestDot = -Infinity;
      const faces = hull.faces;
      for (let i = 0; i < faces.length; i++) {
        const d = faces[i]!.normal.dot(n);
        if (d > bestDot) {
          bestDot = d;
          bestFace = i;
        }
      }
      if (bestDot > 0.99) {
        const face = faces[bestFace]!;
        clipA.copy(p);
        clipB.copy(q);
        if (clipSegmentToFace(hull, bestFace, clipA, clipB)) {
          for (let i = 0; i < 2; i++) {
            const c = i === 0 ? clipA : clipB;
            const h = face.normal.dot(c) - face.d;
            const sep = h - radius;
            if (sep <= spec) {
              // 中点：线段表面 c - n r 与面上投影 c - n h
              t3.copy(c).addScaled(face.normal, -(radius + h) * 0.5);
              buffer.push(t3, sep, (bestFace + 1) * 4 + i);
            }
          }
          if (buffer.count > 0) {
            n.copy(face.normal);
            finishLocal(xf, n, manifold);
            return;
          }
        }
      }
    }
    const sep = out.distance - radius;
    // 中点：多面体表面 pointA 与线段表面 pointB - n r
    t3.copy(out.pointB).addScaled(n, -radius).add(out.pointA).scale(0.5);
    buffer.push(t3, sep, 0);
    finishLocal(xf, n, manifold);
    return;
  }

  // ---- 核心相交：SAT 求最小穿透轴 ----
  const faces = hull.faces;
  let faceSep = -Infinity;
  let faceIndex = 0;
  for (let i = 0; i < faces.length; i++) {
    const f = faces[i]!;
    const s = Math.min(f.normal.dot(p), isSegment ? f.normal.dot(q) : Infinity) - f.d;
    if (s > faceSep) {
      faceSep = s;
      faceIndex = i;
    }
  }
  let edgeSep = -Infinity;
  let edgeIndex = -1;
  if (isSegment) {
    const u = t0.subVectors(q, p);
    u.normalize();
    const edges = hull.edges;
    for (let i = 0; i < edges.length; i++) {
      const e = edges[i]!;
      const va = hull.vertices[e.a]!;
      const vb = hull.vertices[e.b]!;
      const n1 = faces[e.faceA]!.normal;
      const n2 = faces[e.faceB]!.normal;
      // 高斯图剪枝：棱的弧须与线段对应的大圆相交
      if (n1.dot(u) * n2.dot(u) >= 0) continue;
      const axis = t1.subVectors(vb, va).cross(u);
      if (axis.normalize() < 1e-9) continue;
      if (axis.dot(t2.subVectors(va, hull.centroid)) < 0) axis.negate();
      const s = t2.subVectors(p, va).dot(axis);
      if (s > edgeSep) {
        edgeSep = s;
        edgeIndex = i;
        edgeAxis.copy(axis);
      }
    }
  }

  if (edgeIndex >= 0 && edgeSep > faceSep + 0.1 * config.linearSlop) {
    const e = hull.edges[edgeIndex]!;
    closestPointsSegments(hull.vertices[e.a]!, hull.vertices[e.b]!, p, q, c1, c2);
    n.copy(edgeAxis);
    const sep = edgeSep - radius;
    t3.copy(c2).addScaled(n, -radius).add(c1).scale(0.5);
    buffer.push(t3, sep, 1000 + edgeIndex);
    finishLocal(xf, n, manifold);
    return;
  }

  const face = faces[faceIndex]!;
  n.copy(face.normal);
  clipA.copy(p);
  clipB.copy(q);
  const count = isSegment && clipSegmentToFace(hull, faceIndex, clipA, clipB) ? 2 : 1;
  if (count === 1) {
    // 取较深的端点
    if (isSegment && face.normal.dot(q) < face.normal.dot(p)) clipA.copy(q);
    else clipA.copy(p);
  }
  for (let i = 0; i < count; i++) {
    const c = i === 0 ? clipA : clipB;
    const h = face.normal.dot(c) - face.d;
    const sep = h - radius;
    if (sep <= spec) {
      t3.copy(c).addScaled(face.normal, -(radius + h) * 0.5);
      buffer.push(t3, sep, (faceIndex + 1) * 4 + i);
    }
  }
  finishLocal(xf, n, manifold);
}

/** 把局部坐标下的候选点与法线变换到世界并写入流形 */
function finishLocal(xf: Readonly<Transform>, localN: Readonly<Vec3>, manifold: Manifold): void {
  xf.transformVector(localN, manifold.normal);
  for (let i = 0; i < buffer.count; i++) {
    xf.transformPoint(buffer.points[i]!, buffer.points[i]!);
  }
  buffer.reduceInto(manifold, manifold.normal);
}

// ---------------------------------------------------------------------------
// 多面体 vs 多面体：SAT（面 A、面 B、棱-棱）+ 参考面裁剪
// ---------------------------------------------------------------------------

const relXf = new Transform();
const vertsB: Vec3[] = [];
const normalsB: Vec3[] = [];
const offsetsB: number[] = [];
const centroidB = new Vec3();
const edgeAxisBest = new Vec3();

let polyIn: Vec3[] = [];
let polyInIds: number[] = [];
let polyOut: Vec3[] = [];
let polyOutIds: number[] = [];
const polyPool: Vec3[] = [];
let polyPoolUsed = 0;

function poolVec(): Vec3 {
  if (polyPoolUsed >= polyPool.length) polyPool.push(new Vec3());
  return polyPool[polyPoolUsed++]!;
}

function ensureB(n: number, arr: Vec3[]): void {
  while (arr.length < n) arr.push(new Vec3());
}

function isMinkowskiFace(
  a: Readonly<Vec3>,
  b: Readonly<Vec3>,
  bxa: Readonly<Vec3>,
  c: Readonly<Vec3>,
  d: Readonly<Vec3>,
  dxc: Readonly<Vec3>,
): boolean {
  const cba = c.dot(bxa);
  const dba = d.dot(bxa);
  const adc = a.dot(dxc);
  const bdc = b.dot(dxc);
  return cba * dba < 0 && adc * bdc < 0 && cba * bdc > 0;
}

const gA = new Vec3();
const gB = new Vec3();
const gC = new Vec3();
const gD = new Vec3();
const gBxA = new Vec3();
const gDxC = new Vec3();

export function collideHulls(
  hullA: ConvexPolyhedron,
  xfA: Readonly<Transform>,
  hullB: ConvexPolyhedron,
  xfB: Readonly<Transform>,
  manifold: Manifold,
  config: CollideConfig,
): void {
  const spec = config.speculativeDistance;
  const slop = config.linearSlop;

  // 在 A 的局部坐标系中计算
  relXf.multiplyInverseA(xfA, xfB);
  const nvB = hullB.vertices.length;
  const nfB = hullB.faces.length;
  ensureB(nvB, vertsB);
  ensureB(nfB, normalsB);
  for (let i = 0; i < nvB; i++) relXf.transformPoint(hullB.vertices[i]!, vertsB[i]!);
  for (let i = 0; i < nfB; i++) {
    const f = hullB.faces[i]!;
    relXf.transformVector(f.normal, normalsB[i]!);
    offsetsB[i] = f.d + normalsB[i]!.dot(relXf.position);
  }
  relXf.transformPoint(hullB.centroid, centroidB);

  // ---- 面 A ----
  let sepA = -Infinity;
  let faceA = 0;
  const facesA = hullA.faces;
  for (let i = 0; i < facesA.length; i++) {
    const f = facesA[i]!;
    let minD = Infinity;
    for (let k = 0; k < nvB; k++) {
      const d = f.normal.dot(vertsB[k]!);
      if (d < minD) minD = d;
    }
    const s = minD - f.d;
    if (s > sepA) {
      sepA = s;
      faceA = i;
      if (s > spec) return;
    }
  }

  // ---- 面 B ----
  let sepB = -Infinity;
  let faceB = 0;
  const vertsA = hullA.vertices;
  for (let i = 0; i < nfB; i++) {
    const nB = normalsB[i]!;
    let minD = Infinity;
    for (let k = 0; k < vertsA.length; k++) {
      const d = nB.dot(vertsA[k]!);
      if (d < minD) minD = d;
    }
    const s = minD - offsetsB[i]!;
    if (s > sepB) {
      sepB = s;
      faceB = i;
      if (s > spec) return;
    }
  }

  // ---- 棱-棱 ----
  let sepE = -Infinity;
  let edgeA = -1;
  let edgeB = -1;
  const edgesA = hullA.edges;
  const edgesB = hullB.edges;
  for (let i = 0; i < edgesA.length; i++) {
    const ea = edgesA[i]!;
    const pa = vertsA[ea.a]!;
    const qa = vertsA[ea.b]!;
    gA.copy(facesA[ea.faceA]!.normal);
    gB.copy(facesA[ea.faceB]!.normal);
    gBxA.crossVectors(gB, gA);
    const eAx = qa.x - pa.x,
      eAy = qa.y - pa.y,
      eAz = qa.z - pa.z;
    for (let j = 0; j < edgesB.length; j++) {
      const eb = edgesB[j]!;
      gC.copy(normalsB[eb.faceA]!).negate();
      gD.copy(normalsB[eb.faceB]!).negate();
      gDxC.crossVectors(gD, gC);
      if (!isMinkowskiFace(gA, gB, gBxA, gC, gD, gDxC)) continue;
      const pb = vertsB[eb.a]!;
      const qb = vertsB[eb.b]!;
      const eBx = qb.x - pb.x,
        eBy = qb.y - pb.y,
        eBz = qb.z - pb.z;
      let ax = eAy * eBz - eAz * eBy;
      let ay = eAz * eBx - eAx * eBz;
      let az = eAx * eBy - eAy * eBx;
      const len = Math.sqrt(ax * ax + ay * ay + az * az);
      const scale = Math.sqrt(
        (eAx * eAx + eAy * eAy + eAz * eAz) * (eBx * eBx + eBy * eBy + eBz * eBz),
      );
      if (len < 1e-5 * scale) continue; // 平行棱
      ax /= len;
      ay /= len;
      az /= len;
      const cx = pa.x - hullA.centroid.x,
        cy = pa.y - hullA.centroid.y,
        cz = pa.z - hullA.centroid.z;
      if (ax * cx + ay * cy + az * cz < 0) {
        ax = -ax;
        ay = -ay;
        az = -az;
      }
      const s = ax * (pb.x - pa.x) + ay * (pb.y - pa.y) + az * (pb.z - pa.z);
      if (s > sepE) {
        sepE = s;
        edgeA = i;
        edgeB = j;
        edgeAxisBest.set(ax, ay, az);
        if (s > spec) return;
      }
    }
  }

  buffer.clear();
  const n = localNormal;
  const faceMax = Math.max(sepA, sepB);

  if (edgeA >= 0 && sepE > faceMax + 0.5 * slop) {
    // ---- 棱-棱接触：一个点 ----
    const ea = edgesA[edgeA]!;
    const eb = edgesB[edgeB]!;
    closestPointsSegments(vertsA[ea.a]!, vertsA[ea.b]!, vertsB[eb.a]!, vertsB[eb.b]!, c1, c2);
    n.copy(edgeAxisBest);
    c1.add(c2).scale(0.5);
    buffer.push(c1, sepE, 0x40000000 + edgeA * 4096 + edgeB);
    finishLocal(xfA, n, manifold);
    return;
  }

  // ---- 面接触：选择参考面 ----
  const flip = sepB > sepA + 0.1 * slop;
  let refNormal: Vec3;
  let refD: number;
  let refLoop: readonly number[];
  let refVerts: readonly Vec3[];
  let incVerts: readonly Vec3[];
  let incLoop: readonly number[];
  let refIndex: number;
  if (!flip) {
    const rf = facesA[faceA]!;
    refNormal = rf.normal;
    refD = rf.d;
    refLoop = rf.indices;
    refVerts = vertsA;
    refIndex = faceA;
    // B 上的入射面：法线与参考法线最反向
    let inc = 0;
    let minDot = Infinity;
    for (let i = 0; i < nfB; i++) {
      const d = normalsB[i]!.dot(refNormal);
      if (d < minDot) {
        minDot = d;
        inc = i;
      }
    }
    incLoop = hullB.faces[inc]!.indices;
    incVerts = vertsB;
  } else {
    refNormal = normalsB[faceB]!;
    refD = offsetsB[faceB]!;
    refLoop = hullB.faces[faceB]!.indices;
    refVerts = vertsB;
    refIndex = faceB;
    let inc = 0;
    let minDot = Infinity;
    for (let i = 0; i < facesA.length; i++) {
      const d = facesA[i]!.normal.dot(refNormal);
      if (d < minDot) {
        minDot = d;
        inc = i;
      }
    }
    incLoop = facesA[inc]!.indices;
    incVerts = vertsA;
  }

  // 入射多边形
  polyPoolUsed = 0;
  polyIn.length = 0;
  polyInIds.length = 0;
  for (const vi of incLoop) {
    polyIn.push(poolVec().copy(incVerts[vi]!));
    polyInIds.push(vi + 1);
  }

  // Sutherland–Hodgman：用参考面的每条侧平面裁剪
  for (let i = 0; i < refLoop.length && polyIn.length > 0; i++) {
    const v0 = refVerts[refLoop[i]!]!;
    const v1 = refVerts[refLoop[(i + 1) % refLoop.length]!]!;
    const side = t2.subVectors(v1, v0).cross(refNormal);
    const offset = side.dot(v0);
    polyOut.length = 0;
    polyOutIds.length = 0;
    const m = polyIn.length;
    for (let k = 0; k < m; k++) {
      const a = polyIn[k]!;
      const b = polyIn[(k + 1) % m]!;
      const ida = polyInIds[k]!;
      const idb = polyInIds[(k + 1) % m]!;
      const da = side.dot(a) - offset;
      const db = side.dot(b) - offset;
      if (da <= 0) {
        polyOut.push(a);
        polyOutIds.push(ida);
      }
      if ((da <= 0 && db > 0) || (da > 0 && db <= 0)) {
        const t = da / (da - db);
        polyOut.push(poolVec().lerpVectors(a, b, t));
        const lo = Math.min(ida, idb) & 0xff;
        const hi = Math.max(ida, idb) & 0xff;
        polyOutIds.push(0x10000 * (i + 1) + lo * 256 + hi);
      }
    }
    // 交换输入输出
    const tp = polyIn;
    polyIn = polyOut;
    polyOut = tp;
    const ti = polyInIds;
    polyInIds = polyOutIds;
    polyOutIds = ti;
  }

  const idBase = (refIndex * 2 + (flip ? 1 : 0)) * 0x1000000;
  for (let k = 0; k < polyIn.length; k++) {
    const v = polyIn[k]!;
    const sep = refNormal.dot(v) - refD;
    if (sep <= spec) {
      t3.copy(v).addScaled(refNormal, -sep * 0.5);
      buffer.push(t3, sep, idBase + polyInIds[k]!);
    }
  }
  n.copy(refNormal);
  if (flip) n.negate();
  finishLocal(xfA, n, manifold);
}

// ---------------------------------------------------------------------------
// 重叠测试（传感器使用）
// ---------------------------------------------------------------------------

const ovA = new DistanceProxy();
const ovB = new DistanceProxy();
const ovOut = new DistanceOutput();

/** 两形状是否相交（接触也算） */
export function testOverlap(
  shapeA: Shape,
  xfA: Readonly<Transform>,
  shapeB: Shape,
  xfB: Readonly<Transform>,
): boolean {
  const aPlane = shapeA.type === ShapeType.Plane;
  const bPlane = shapeB.type === ShapeType.Plane;
  if (aPlane && bPlane) return false;
  if (aPlane || bPlane) {
    const plane = aPlane ? xfA : xfB;
    const other = aPlane ? shapeB : shapeA;
    const xfO = aPlane ? xfB : xfA;
    plane.rotation.rotate(UP, planeN);
    const core = other.getCorePoints();
    for (const c of core) {
      xfO.transformPoint(c, t0);
      if (t0.sub(plane.position).dot(planeN) - other.radius <= 0) return true;
    }
    return false;
  }
  ovA.setShape(shapeA, xfA);
  ovB.setShape(shapeB, xfB);
  return gjkDistance(ovA, ovB, true, ovOut).distance <= 1e-9;
}
