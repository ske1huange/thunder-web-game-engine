import type { CollisionFilter } from '../dynamics/Collider';
import { BallSocketJoint } from '../dynamics/joints/BallSocketJoint';
import { ConeTwistJoint } from '../dynamics/joints/ConeTwistJoint';
import { DistanceJoint } from '../dynamics/joints/DistanceJoint';
import { FixedJoint } from '../dynamics/joints/FixedJoint';
import { HingeJoint } from '../dynamics/joints/HingeJoint';
import type { Joint } from '../dynamics/joints/Joint';
import { MouseJoint } from '../dynamics/joints/MouseJoint';
import { SliderJoint } from '../dynamics/joints/SliderJoint';
import type { BodyType, RigidBody } from '../dynamics/RigidBody';
import { Quat } from '../math/Quat';
import { Vec3 } from '../math/Vec3';
import { BoxShape } from '../shapes/BoxShape';
import { CapsuleShape } from '../shapes/CapsuleShape';
import { ConvexHullShape } from '../shapes/ConvexHullShape';
import { CylinderShape } from '../shapes/CylinderShape';
import { HeightfieldShape } from '../shapes/HeightfieldShape';
import { PlaneShape } from '../shapes/PlaneShape';
import type { Shape } from '../shapes/Shape';
import { SphereShape } from '../shapes/SphereShape';
import { TriMeshShape } from '../shapes/TriMeshShape';
import type { World } from '../world/World';

/**
 * 可序列化的描述（纯数据，可 JSON 化 / 通过 postMessage 传给 Worker）。
 * 向量写成 [x, y, z]，四元数写成 [x, y, z, w]。
 */
export type Vec3Tuple = readonly [number, number, number];
export type QuatTuple = readonly [number, number, number, number];

export type ShapeDesc =
  | { type: 'sphere'; radius: number }
  | { type: 'box'; halfExtents: Vec3Tuple }
  | { type: 'capsule'; radius: number; halfHeight: number }
  | { type: 'cylinder'; radius: number; halfHeight: number; segments?: number }
  | { type: 'convexHull'; points: ArrayLike<number> }
  | { type: 'plane' }
  | {
      type: 'trimesh';
      positions: ArrayLike<number>;
      indices?: ArrayLike<number>;
      doubleSided?: boolean;
      weldTolerance?: number;
    }
  | {
      type: 'heightfield';
      heights: ArrayLike<number>;
      rows: number;
      cols: number;
      cellSize?: number | { x: number; z: number };
      heightScale?: number;
    };

export interface ColliderDesc {
  shape: ShapeDesc;
  position?: Vec3Tuple;
  rotation?: QuatTuple;
  density?: number;
  friction?: number;
  restitution?: number;
  isSensor?: boolean;
  filter?: Partial<CollisionFilter>;
}

export interface BodyDesc {
  type?: BodyType;
  position?: Vec3Tuple;
  rotation?: QuatTuple;
  linearVelocity?: Vec3Tuple;
  angularVelocity?: Vec3Tuple;
  linearDamping?: number;
  angularDamping?: number;
  gravityScale?: number;
  allowSleep?: boolean;
  awake?: boolean;
  fixedRotation?: boolean;
  isBullet?: boolean;
  enableCCD?: boolean;
  colliders?: ColliderDesc[];
}

interface JointDescBase {
  collideConnected?: boolean;
  /** 其余选项与对应关节类的构造参数相同，向量写成 [x, y, z] */
  [option: string]: unknown;
}

export type JointDesc = JointDescBase & {
  type: 'ballSocket' | 'hinge' | 'coneTwist' | 'distance' | 'fixed' | 'slider' | 'mouse';
};

const v = (t: Vec3Tuple | undefined): Vec3 | undefined =>
  t ? new Vec3(t[0], t[1], t[2]) : undefined;
const q = (t: QuatTuple | undefined): Quat | undefined =>
  t ? new Quat(t[0], t[1], t[2], t[3]) : undefined;

/** 由描述创建形状 */
export function shapeFromDesc(desc: ShapeDesc): Shape {
  switch (desc.type) {
    case 'sphere':
      return new SphereShape(desc.radius);
    case 'box':
      return new BoxShape(v(desc.halfExtents)!);
    case 'capsule':
      return new CapsuleShape(desc.radius, desc.halfHeight);
    case 'cylinder':
      return new CylinderShape(desc.radius, desc.halfHeight, desc.segments);
    case 'convexHull': {
      const pts: Vec3[] = [];
      for (let i = 0; i + 2 < desc.points.length; i += 3) {
        pts.push(new Vec3(desc.points[i]!, desc.points[i + 1]!, desc.points[i + 2]!));
      }
      return new ConvexHullShape(pts);
    }
    case 'plane':
      return new PlaneShape();
    case 'trimesh':
      return new TriMeshShape(desc.positions, desc.indices ?? null, {
        doubleSided: desc.doubleSided,
        weldTolerance: desc.weldTolerance,
      });
    case 'heightfield':
      return new HeightfieldShape(desc);
    default:
      throw new Error(`未知的形状类型：${(desc as { type: string }).type}`);
  }
}

/** 由描述创建刚体（含碰撞体） */
export function createBodyFromDesc(world: World, desc: BodyDesc): RigidBody {
  const body = world.createBody({
    type: desc.type,
    position: v(desc.position),
    rotation: q(desc.rotation),
    linearVelocity: v(desc.linearVelocity),
    angularVelocity: v(desc.angularVelocity),
    linearDamping: desc.linearDamping,
    angularDamping: desc.angularDamping,
    gravityScale: desc.gravityScale,
    allowSleep: desc.allowSleep,
    awake: desc.awake,
    fixedRotation: desc.fixedRotation,
    isBullet: desc.isBullet,
    enableCCD: desc.enableCCD,
  });
  for (const c of desc.colliders ?? []) {
    body.addCollider({
      shape: shapeFromDesc(c.shape),
      position: v(c.position),
      rotation: q(c.rotation),
      density: c.density,
      friction: c.friction,
      restitution: c.restitution,
      isSensor: c.isSensor,
      filter: c.filter,
    });
  }
  return body;
}

/** 由描述创建关节（尚未加入世界），数组形式的三维向量选项会转换为 Vec3 */
export function jointFromDesc(desc: JointDesc, bodyA: RigidBody, bodyB: RigidBody): Joint {
  const options: Record<string, unknown> = { bodyA, bodyB };
  for (const [key, value] of Object.entries(desc)) {
    if (key === 'type' || key === 'bodyA' || key === 'bodyB') continue;
    options[key] =
      Array.isArray(value) && value.length === 3 && value.every((x) => typeof x === 'number')
        ? new Vec3(value[0], value[1], value[2])
        : value;
  }
  switch (desc.type) {
    case 'ballSocket':
      return new BallSocketJoint(options as never);
    case 'hinge':
      return new HingeJoint(options as never);
    case 'coneTwist':
      return new ConeTwistJoint(options as never);
    case 'distance':
      return new DistanceJoint(options as never);
    case 'fixed':
      return new FixedJoint(options as never);
    case 'slider':
      return new SliderJoint(options as never);
    case 'mouse':
      return new MouseJoint(options as never);
    default:
      throw new Error(`未知的关节类型：${String(desc.type)}`);
  }
}
