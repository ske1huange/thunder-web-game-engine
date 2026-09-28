import type { Collider } from '../dynamics/Collider';
import type { Joint } from '../dynamics/joints/Joint';
import type { RigidBody } from '../dynamics/RigidBody';
import type { Contact } from '../dynamics/Contact';
import type { AABB } from '../math/AABB';
import { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import type { CapsuleShape } from '../shapes/CapsuleShape';
import type { MeshShape } from '../shapes/MeshShape';
import { ShapeType } from '../shapes/Shape';

/**
 * 调试绘制接口：引擎只输出线段与点，由渲染端（three.js / Canvas / WebGL）自行绘制。
 * 颜色为 0xRRGGBB。
 */
export interface DebugDrawer {
  drawLine(from: Readonly<Vec3>, to: Readonly<Vec3>, color: number): void;
  drawPoint?(point: Readonly<Vec3>, size: number, color: number): void;
}

export interface DebugDrawOptions {
  shapes?: boolean;
  aabbs?: boolean;
  contacts?: boolean;
  joints?: boolean;
  centerOfMass?: boolean;
}

export const DebugColors = {
  static: 0x808080,
  kinematic: 0x4d8cff,
  dynamic: 0xe6b34d,
  sleeping: 0x8c8c99,
  sensor: 0x4dcc66,
  aabb: 0xb266ff,
  contactPoint: 0xff3333,
  contactNormal: 0x33ff66,
  joint: 0x33ccff,
  centerOfMass: 0xffffff,
} as const;

interface DebugWorld {
  readonly bodies: readonly RigidBody[];
  readonly joints: readonly Joint[];
  readonly contacts: readonly Contact[];
}

const a = new Vec3();
const b = new Vec3();
const c = new Vec3();
const d = new Vec3();
const localXf = new Transform();
const CIRCLE_SEGMENTS = 16;

function bodyColor(collider: Collider): number {
  if (collider.isSensor) return DebugColors.sensor;
  const body = collider.body;
  if (body.isStatic()) return DebugColors.static;
  if (body.isKinematic()) return DebugColors.kinematic;
  return body.isAwake ? DebugColors.dynamic : DebugColors.sleeping;
}

/** 在 xf 下画一个圆：圆心 center（局部），法线轴 axis（0=X,1=Y,2=Z），半径 r */
function drawCircle(
  drawer: DebugDrawer,
  xf: Readonly<Transform>,
  center: Readonly<Vec3>,
  axis: number,
  r: number,
  color: number,
  startAngle = 0,
  sweep = Math.PI * 2,
  segments = CIRCLE_SEGMENTS,
): void {
  const point = (t: number, out: Vec3): Vec3 => {
    const cs = Math.cos(t) * r;
    const sn = Math.sin(t) * r;
    if (axis === 0) out.set(center.x, center.y + cs, center.z + sn);
    else if (axis === 1) out.set(center.x + cs, center.y, center.z + sn);
    else out.set(center.x + cs, center.y + sn, center.z);
    return xf.transformPoint(out, out);
  };
  point(startAngle, a);
  for (let i = 1; i <= segments; i++) {
    point(startAngle + (sweep * i) / segments, b);
    drawer.drawLine(a, b, color);
    a.copy(b);
  }
}

export function drawCollider(
  drawer: DebugDrawer,
  collider: Collider,
  color = bodyColor(collider),
): void {
  const shape = collider.shape;
  const xf = collider.worldTransform;
  switch (shape.type) {
    case ShapeType.Sphere: {
      const origin = c.set(0, 0, 0);
      for (let axis = 0; axis < 3; axis++)
        drawCircle(drawer, xf, origin, axis, shape.radius, color);
      break;
    }
    case ShapeType.Capsule: {
      const cap = shape as CapsuleShape;
      const r = cap.radius;
      const h = cap.halfHeight;
      const top = new Vec3(0, h, 0);
      const bottom = new Vec3(0, -h, 0);
      drawCircle(drawer, xf, top, 1, r, color);
      drawCircle(drawer, xf, bottom, 1, r, color);
      // 两端半球的经线
      drawCircle(drawer, xf, top, 2, r, color, 0, Math.PI, 8);
      drawCircle(drawer, xf, bottom, 2, r, color, Math.PI, Math.PI, 8);
      drawCircle(drawer, xf, top, 0, r, color, 0, Math.PI, 8);
      drawCircle(drawer, xf, bottom, 0, r, color, Math.PI, Math.PI, 8);
      for (const [x, z] of [
        [r, 0],
        [-r, 0],
        [0, r],
        [0, -r],
      ] as const) {
        xf.transformPoint(c.set(x, h, z), c);
        xf.transformPoint(d.set(x, -h, z), d);
        drawer.drawLine(c, d, color);
      }
      break;
    }
    case ShapeType.Plane: {
      // 以局部原点为中心画一个 20m 的网格
      const size = 10;
      const step = 2;
      for (let i = -size; i <= size; i += step) {
        xf.transformPoint(c.set(i, 0, -size), c);
        xf.transformPoint(d.set(i, 0, size), d);
        drawer.drawLine(c, d, color);
        xf.transformPoint(c.set(-size, 0, i), c);
        xf.transformPoint(d.set(size, 0, i), d);
        drawer.drawLine(c, d, color);
      }
      break;
    }
    case ShapeType.TriMesh:
    case ShapeType.Heightfield: {
      const mesh = shape as MeshShape;
      for (let t = 0; t < mesh.triangleCount; t++) {
        mesh.getTriangle(t, a, b, c);
        xf.transformPoint(a, a);
        xf.transformPoint(b, b);
        xf.transformPoint(c, c);
        drawer.drawLine(a, b, color);
        drawer.drawLine(b, c, color);
        drawer.drawLine(c, a, color);
      }
      break;
    }
    default: {
      const hull = shape.hull;
      if (!hull) break;
      for (const e of hull.edges) {
        xf.transformPoint(hull.vertices[e.a]!, c);
        xf.transformPoint(hull.vertices[e.b]!, d);
        drawer.drawLine(c, d, color);
      }
    }
  }
}

export function drawAABB(drawer: DebugDrawer, box: Readonly<AABB>, color: number): void {
  const mn = box.min;
  const mx = box.max;
  const corners = [
    [mn.x, mn.y, mn.z],
    [mx.x, mn.y, mn.z],
    [mx.x, mx.y, mn.z],
    [mn.x, mx.y, mn.z],
    [mn.x, mn.y, mx.z],
    [mx.x, mn.y, mx.z],
    [mx.x, mx.y, mx.z],
    [mn.x, mx.y, mx.z],
  ] as const;
  const edges = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 0],
    [4, 5],
    [5, 6],
    [6, 7],
    [7, 4],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7],
  ] as const;
  for (const [i, j] of edges) {
    const p = corners[i]!;
    const q = corners[j]!;
    drawer.drawLine(c.set(p[0], p[1], p[2]), d.set(q[0], q[1], q[2]), color);
  }
}

/** 绘制整个世界 */
export function debugDrawWorld(
  world: DebugWorld,
  drawer: DebugDrawer,
  options: DebugDrawOptions = {},
): void {
  const {
    shapes = true,
    aabbs = false,
    contacts = false,
    joints = true,
    centerOfMass = false,
  } = options;
  for (const body of world.bodies) {
    for (const collider of body.colliders) {
      if (shapes) drawCollider(drawer, collider);
      if (aabbs && collider.shape.type !== ShapeType.Plane)
        drawAABB(drawer, collider.aabb, DebugColors.aabb);
    }
    if (centerOfMass && body.isDynamic()) {
      localXf.position.copy(body.center);
      localXf.rotation.copy(body.rotation);
      for (let axis = 0; axis < 3; axis++) {
        d.setScalar(0).setComponent(axis, 0.2);
        localXf.transformPoint(d, d);
        drawer.drawLine(body.center, d, DebugColors.centerOfMass);
      }
    }
  }
  if (joints) {
    for (const j of world.joints) {
      j.getAnchorA(a);
      j.getAnchorB(b);
      drawer.drawLine(j.bodyA.center, a, DebugColors.joint);
      drawer.drawLine(a, b, DebugColors.joint);
      drawer.drawLine(j.bodyB.center, b, DebugColors.joint);
    }
  }
  if (contacts) {
    for (const contact of world.contacts) {
      if (!contact.touching || contact.isSensor) continue;
      for (let mi = 0; mi < contact.manifoldCount; mi++) {
        const m = contact.manifolds[mi]!;
        for (let i = 0; i < m.pointCount; i++) {
          const p = m.points[i]!.point;
          drawer.drawPoint?.(p, 4, DebugColors.contactPoint);
          c.copy(p).addScaled(m.normal, 0.3);
          drawer.drawLine(p, c, DebugColors.contactNormal);
        }
      }
    }
  }
}
