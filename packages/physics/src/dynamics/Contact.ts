import { Manifold, ManifoldPoint } from '../collision/Manifold';
import { type CollideConfig, collideShapes, testOverlap } from '../collision/narrowphase/Collide';
import { Softness } from './solver/Softness';
import type { Collider } from './Collider';

/** 帧间匹配接触点的距离阈值（米） */
const MATCH_DISTANCE_SQ = 0.05 * 0.05;

const oldPoints: ManifoldPoint[] = [];
for (let i = 0; i < 4; i++) oldPoints.push(new ManifoldPoint());

/**
 * 接触：一对宽相重叠的碰撞体。
 * 窄相更新流形；实心接触参与求解，传感器接触只记录是否重叠。
 */
export class Contact {
  readonly colliderA: Collider;
  readonly colliderB: Collider;
  readonly key: number;
  readonly manifold = new Manifold();
  readonly isSensor: boolean;
  /** 当前是否接触（实心：有接触点；传感器：形状重叠） */
  touching = false;
  friction: number;
  restitution: number;
  /** @internal 求解器使用的软约束参数 */
  readonly softness = new Softness();
  /** @internal 在 ContactManager.contacts 中的下标 */
  index = -1;

  constructor(colliderA: Collider, colliderB: Collider, key: number) {
    this.colliderA = colliderA;
    this.colliderB = colliderB;
    this.key = key;
    this.isSensor = colliderA.isSensor || colliderB.isSensor;
    this.friction = Math.sqrt(colliderA.friction * colliderB.friction);
    this.restitution = Math.max(colliderA.restitution, colliderB.restitution);
  }

  get bodyA() {
    return this.colliderA.body;
  }

  get bodyB() {
    return this.colliderB.body;
  }

  /** 给定一个碰撞体，返回另一个 */
  getOther(collider: Collider): Collider {
    return collider === this.colliderA ? this.colliderB : this.colliderA;
  }

  /**
   * @internal 执行窄相并继承上一帧的冲量。返回新的 touching 状态。
   */
  update(config: CollideConfig): boolean {
    const a = this.colliderA;
    const b = this.colliderB;
    if (this.isSensor) {
      this.touching = testOverlap(a.shape, a.worldTransform, b.shape, b.worldTransform);
      return this.touching;
    }
    this.friction = Math.sqrt(a.friction * b.friction);
    this.restitution = Math.max(a.restitution, b.restitution);

    const m = this.manifold;
    const oldCount = m.pointCount;
    for (let i = 0; i < oldCount; i++) {
      const src = m.points[i]!;
      const dst = oldPoints[i]!;
      dst.copyGeometry(src);
      dst.normalImpulse = src.normalImpulse;
      dst.tangentImpulse1 = src.tangentImpulse1;
      dst.tangentImpulse2 = src.tangentImpulse2;
    }

    collideShapes(a.shape, a.worldTransform, b.shape, b.worldTransform, m, config);

    const bodyA = a.body;
    const bodyB = b.body;
    for (let i = 0; i < m.pointCount; i++) {
      const p = m.points[i]!;
      bodyA.transform.inverseTransformPoint(p.point, p.localA);
      bodyB.transform.inverseTransformPoint(p.point, p.localB);
      p.normalImpulse = 0;
      p.tangentImpulse1 = 0;
      p.tangentImpulse2 = 0;
      p.maxNormalImpulse = 0;
      p.persisted = false;
      // 先按特征编号匹配，失败再按局部位置就近匹配
      let match = -1;
      for (let k = 0; k < oldCount; k++) {
        if (oldPoints[k]!.id === p.id) {
          match = k;
          break;
        }
      }
      if (match < 0) {
        let best = MATCH_DISTANCE_SQ;
        for (let k = 0; k < oldCount; k++) {
          const d = oldPoints[k]!.localA.distanceToSq(p.localA);
          if (d < best) {
            best = d;
            match = k;
          }
        }
      }
      if (match >= 0) {
        const o = oldPoints[match]!;
        p.normalImpulse = o.normalImpulse;
        p.tangentImpulse1 = o.tangentImpulse1;
        p.tangentImpulse2 = o.tangentImpulse2;
        p.persisted = true;
      }
    }
    this.touching = m.pointCount > 0;
    return this.touching;
  }
}
