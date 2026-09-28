import { Manifold, ManifoldPoint } from '../collision/Manifold';
import { type CollideConfig, collideShapes, testOverlap } from '../collision/narrowphase/Collide';
import { type ManifoldList, collideMesh } from '../collision/narrowphase/CollideMesh';
import { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import type { MeshShape } from '../shapes/MeshShape';
import { ShapeType } from '../shapes/Shape';
import { Softness } from './solver/Softness';
import type { Collider } from './Collider';

/** 帧间匹配接触点的距离阈值（米） */
const MATCH_DISTANCE_SQ = 0.05 * 0.05;
/** 接触缓存：相对位移小于 1 mm、相对转角小于 2° 时复用上次的流形 */
const CACHE_POSITION_SQ = 1e-3 * 1e-3;
const CACHE_ROTATION_COS = Math.cos((2 * Math.PI) / 180 / 2);
const relXf = new Transform();
const pA = new Vec3();
const pB = new Vec3();
/** 匹配时要求新旧法线足够接近 */
const MATCH_NORMAL_COS = 0.9;

/** 上一帧的接触点（跨所有流形），用于 warm start 匹配 */
const oldPoints: ManifoldPoint[] = [];
const oldNormals: Vec3[] = [];

function ensureOld(n: number): void {
  while (oldPoints.length < n) {
    oldPoints.push(new ManifoldPoint());
    oldNormals.push(new Vec3());
  }
}

function isMeshShape(type: number): boolean {
  return type === ShapeType.TriMesh || type === ShapeType.Heightfield;
}

/**
 * 接触：一对宽相重叠的碰撞体。
 * 窄相更新流形；实心接触参与求解，传感器接触只记录是否重叠。
 *
 * 凸形状之间至多一个流形；与三角网格 / 高度场接触时，法线不同的三角形各自形成流形（manifolds）。
 */
export class Contact implements ManifoldList {
  readonly colliderA: Collider;
  readonly colliderB: Collider;
  readonly key: number;
  /** 接触流形（前 manifoldCount 个有效） */
  readonly manifolds: Manifold[] = [new Manifold()];
  /** 有效流形数量 */
  manifoldCount = 0;
  readonly isSensor: boolean;
  /** 是否有一方是三角网格 / 高度场 */
  readonly isMesh: boolean;
  /** 当前是否接触（实心：有接触点；传感器：形状重叠） */
  touching = false;
  friction: number;
  restitution: number;
  /** @internal 求解器使用的软约束参数 */
  readonly softness = new Softness();
  /** @internal 在 ContactManager.contacts 中的下标 */
  index = -1;
  /** 上一次完整窄相时 B 相对 A 的位姿，用于接触缓存 */
  private readonly cachedRelative = new Transform();
  private cachedSpeculative = 0;
  private cacheValid = false;

  constructor(colliderA: Collider, colliderB: Collider, key: number) {
    this.colliderA = colliderA;
    this.colliderB = colliderB;
    this.key = key;
    this.isSensor = colliderA.isSensor || colliderB.isSensor;
    this.isMesh = isMeshShape(colliderA.shape.type) || isMeshShape(colliderB.shape.type);
    this.friction = Math.sqrt(colliderA.friction * colliderB.friction);
    this.restitution = Math.max(colliderA.restitution, colliderB.restitution);
  }

  get bodyA() {
    return this.colliderA.body;
  }

  get bodyB() {
    return this.colliderB.body;
  }

  /** 第一个流形（凸形状之间唯一的流形） */
  get manifold(): Manifold {
    return this.manifolds[0]!;
  }

  /** 所有流形的接触点总数 */
  get pointCount(): number {
    let n = 0;
    for (let i = 0; i < this.manifoldCount; i++) n += this.manifolds[i]!.pointCount;
    return n;
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
    const bodyA = a.body;
    const bodyB = b.body;

    // ---- 接触缓存（参考 Jolt 的 body pair cache） ----
    // 两者相对位姿几乎没变、且推测距离没有变大时，沿用上次的流形，只更新分离距离
    relXf.multiplyInverseA(bodyA.transform, bodyB.transform);
    const cached = this.cachedRelative;
    if (
      this.cacheValid &&
      config.speculativeDistance <= this.cachedSpeculative &&
      relXf.position.distanceToSq(cached.position) < CACHE_POSITION_SQ &&
      Math.abs(relXf.rotation.dot(cached.rotation)) > CACHE_ROTATION_COS
    ) {
      this.refreshFromCache();
      return this.touching;
    }
    cached.copy(relXf);
    this.cachedSpeculative = config.speculativeDistance;
    this.cacheValid = true;

    // 保存上一帧的接触点
    let oldCount = 0;
    for (let mi = 0; mi < this.manifoldCount; mi++) {
      const m = this.manifolds[mi]!;
      ensureOld(oldCount + m.pointCount);
      for (let i = 0; i < m.pointCount; i++) {
        const src = m.points[i]!;
        const dst = oldPoints[oldCount]!;
        dst.copyGeometry(src);
        dst.normalImpulse = src.normalImpulse;
        dst.tangentImpulse1 = src.tangentImpulse1;
        dst.tangentImpulse2 = src.tangentImpulse2;
        oldNormals[oldCount]!.copy(m.normal);
        oldCount++;
      }
    }

    if (this.isMesh) {
      const aMesh = isMeshShape(a.shape.type);
      if (aMesh) {
        collideMesh(
          a.shape as MeshShape,
          a.worldTransform,
          b.shape,
          b.worldTransform,
          false,
          this,
          config,
        );
      } else {
        collideMesh(
          b.shape as MeshShape,
          b.worldTransform,
          a.shape,
          a.worldTransform,
          true,
          this,
          config,
        );
      }
    } else {
      const m = this.manifolds[0]!;
      collideShapes(a.shape, a.worldTransform, b.shape, b.worldTransform, m, config);
      this.manifoldCount = m.pointCount > 0 ? 1 : 0;
    }

    for (let mi = 0; mi < this.manifoldCount; mi++) {
      const m = this.manifolds[mi]!;
      const n = m.normal;
      bodyA.transform.inverseTransformVector(n, m.localNormal);
      for (let i = 0; i < m.pointCount; i++) {
        const p = m.points[i]!;
        bodyA.transform.inverseTransformPoint(p.point, p.localA);
        bodyB.transform.inverseTransformPoint(p.point, p.localB);
        p.cachedSeparation = p.separation;
        p.normalImpulse = 0;
        p.tangentImpulse1 = 0;
        p.tangentImpulse2 = 0;
        p.maxNormalImpulse = 0;
        p.persisted = false;
        // 先按特征编号匹配，失败再按局部位置就近匹配（都要求法线相近）
        let match = -1;
        for (let k = 0; k < oldCount; k++) {
          if (oldPoints[k]!.id === p.id && oldNormals[k]!.dot(n) > MATCH_NORMAL_COS) {
            match = k;
            break;
          }
        }
        if (match < 0) {
          let best = MATCH_DISTANCE_SQ;
          for (let k = 0; k < oldCount; k++) {
            const d = oldPoints[k]!.localA.distanceToSq(p.localA);
            if (d < best && oldNormals[k]!.dot(n) > MATCH_NORMAL_COS) {
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
    }
    this.touching = this.manifoldCount > 0;
    return this.touching;
  }

  /** 由缓存的局部坐标重建接触点：法线随 A 旋转，分离距离按两锚点的相对位移修正 */
  private refreshFromCache(): void {
    const xfA = this.colliderA.body.transform;
    const xfB = this.colliderB.body.transform;
    for (let mi = 0; mi < this.manifoldCount; mi++) {
      const m = this.manifolds[mi]!;
      const n = xfA.transformVector(m.localNormal, m.normal);
      for (let i = 0; i < m.pointCount; i++) {
        const p = m.points[i]!;
        xfA.transformPoint(p.localA, pA);
        xfB.transformPoint(p.localB, pB);
        p.point.addVectors(pA, pB).scale(0.5);
        p.separation = p.cachedSeparation + pB.sub(pA).dot(n);
        p.maxNormalImpulse = 0;
        p.persisted = true;
      }
    }
  }

  /** 丢弃接触缓存，下一步强制执行完整窄相 */
  invalidateCache(): void {
    this.cacheValid = false;
  }
}
