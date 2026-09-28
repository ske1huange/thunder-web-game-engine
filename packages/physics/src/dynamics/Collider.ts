import type { ProxyHandle } from '../collision/broadphase/BroadPhase';
import { AABB } from '../math/AABB';
import type { Quat } from '../math/Quat';
import { Transform } from '../math/Transform';
import type { Vec3 } from '../math/Vec3';
import type { Shape } from '../shapes/Shape';
import type { RigidBody } from './RigidBody';

/**
 * 碰撞过滤（与 Box2D 相同的规则）：
 * - group 相同且非 0：正数总是碰撞，负数从不碰撞
 * - 否则需要 (catA & maskB) 且 (catB & maskA)
 */
export interface CollisionFilter {
  categoryBits: number;
  maskBits: number;
  group: number;
}

export const DEFAULT_FILTER: Readonly<CollisionFilter> = Object.freeze({
  categoryBits: 0x0001,
  maskBits: 0xffffffff,
  group: 0,
});

export function shouldFiltersCollide(
  a: Readonly<CollisionFilter>,
  b: Readonly<CollisionFilter>,
): boolean {
  if (a.group === b.group && a.group !== 0) return a.group > 0;
  return (a.categoryBits & b.maskBits) !== 0 && (b.categoryBits & a.maskBits) !== 0;
}

export interface ColliderOptions {
  shape: Shape;
  /** 相对刚体原点的局部偏移 */
  position?: Readonly<Vec3>;
  /** 相对刚体的局部旋转 */
  rotation?: Readonly<Quat>;
  /** 密度（kg/m³），默认 1 */
  density?: number;
  /** 摩擦系数，默认 0.6；两者按几何平均混合 */
  friction?: number;
  /** 弹性系数，默认 0；两者取最大值 */
  restitution?: number;
  /** 传感器只检测重叠、不产生碰撞响应 */
  isSensor?: boolean;
  filter?: Partial<CollisionFilter>;
  userData?: unknown;
}

/** 碰撞体：挂在刚体上的一个形状，带有局部变换与材质参数 */
export class Collider {
  readonly id: number;
  readonly body: RigidBody;
  readonly shape: Shape;
  /** 相对刚体原点的局部变换 */
  readonly localTransform: Transform;
  /** 世界变换（每步更新） */
  readonly worldTransform = new Transform();
  /** 紧致世界包围盒 */
  readonly aabb = new AABB();
  density: number;
  friction: number;
  restitution: number;
  readonly isSensor: boolean;
  readonly filter: CollisionFilter;
  userData: unknown;

  /** @internal 宽相代理句柄 */
  proxy: ProxyHandle | null = null;

  /** @internal 由 RigidBody.addCollider 创建 */
  constructor(id: number, body: RigidBody, options: ColliderOptions) {
    this.id = id;
    this.body = body;
    this.shape = options.shape;
    this.localTransform = new Transform(options.position, options.rotation);
    this.density = options.density ?? 1;
    this.friction = options.friction ?? 0.6;
    this.restitution = options.restitution ?? 0;
    this.isSensor = options.isSensor ?? false;
    this.filter = { ...DEFAULT_FILTER, ...options.filter };
    this.userData = options.userData;
    this.updateWorldTransform();
  }

  /** @internal */
  updateWorldTransform(): void {
    this.worldTransform.multiplyTransforms(this.body.transform, this.localTransform);
    this.shape.computeAABB(this.worldTransform, this.aabb);
  }

  /** 修改过滤条件后调用，使宽相重新配对 */
  setFilter(filter: Partial<CollisionFilter>): void {
    Object.assign(this.filter, filter);
    this.body.world?.refilterCollider(this);
  }
}
