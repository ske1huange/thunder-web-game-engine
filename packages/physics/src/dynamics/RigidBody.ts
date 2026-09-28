import { AABB } from '../math/AABB';
import { Mat3 } from '../math/Mat3';
import { Quat } from '../math/Quat';
import { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import { MassProperties } from '../shapes/Shape';
import type { World } from '../world/World';
import { Collider, type ColliderOptions } from './Collider';
import type { Contact } from './Contact';
import type { Joint } from './joints/Joint';

/**
 * - static：静止不动，质量无穷大（地面、墙体）
 * - kinematic：按速度运动、不受力影响（移动平台）
 * - dynamic：受力与碰撞影响的普通刚体
 */
export type BodyType = 'static' | 'kinematic' | 'dynamic';

export interface RigidBodyOptions {
  type?: BodyType;
  /** 刚体原点的世界坐标 */
  position?: Readonly<Vec3>;
  rotation?: Readonly<Quat>;
  linearVelocity?: Readonly<Vec3>;
  angularVelocity?: Readonly<Vec3>;
  /** 线性阻尼，默认 0 */
  linearDamping?: number;
  /** 角阻尼，默认 0.05 */
  angularDamping?: number;
  /** 重力缩放，默认 1 */
  gravityScale?: number;
  /** 是否允许休眠，默认 true */
  allowSleep?: boolean;
  /** 初始是否唤醒，默认 true */
  awake?: boolean;
  /** 锁定旋转（例如角色控制器） */
  fixedRotation?: boolean;
  /** 高速物体：连续碰撞也对动态物体生效 */
  isBullet?: boolean;
  /** 是否对该物体做连续碰撞检测，默认 true */
  enableCCD?: boolean;
  userData?: unknown;
}

const tmpV = new Vec3();
const tmpV2 = new Vec3();
const tmpV3 = new Vec3();
const tmpMass = new MassProperties();
const tmpInertia = new Mat3();
const tmpQ = new Quat();

/** 刚体 */
export class RigidBody {
  readonly id: number;
  /** 所属世界（销毁后为 null） */
  world: World | null;
  private _type: BodyType;

  /** 刚体原点的变换（位置 + 旋转） */
  readonly transform = new Transform();
  /** 世界坐标质心 */
  readonly center = new Vec3();
  /** 局部坐标质心 */
  readonly localCenter = new Vec3();

  readonly linearVelocity = new Vec3();
  readonly angularVelocity = new Vec3();
  /** 累积外力（每步清零） */
  readonly force = new Vec3();
  readonly torque = new Vec3();

  mass = 0;
  invMass = 0;
  /** 局部坐标系下关于质心的惯性张量 */
  readonly inertiaLocal = new Mat3().setZero();
  readonly invInertiaLocal = new Mat3().setZero();
  /** 世界坐标系下的逆惯性张量（每步开始时更新） */
  readonly invInertiaWorld = new Mat3().setZero();

  linearDamping: number;
  angularDamping: number;
  gravityScale: number;
  allowSleep: boolean;
  fixedRotation: boolean;
  isBullet: boolean;
  enableCCD: boolean;
  userData: unknown;

  readonly colliders: Collider[] = [];

  // ---- 内部状态 ----
  /** @internal */ awakeFlag: boolean;
  /** @internal */ sleepTime = 0;
  /** @internal 到质心最远的距离，用于休眠判定与 CCD */
  maxExtent = 0;
  /** @internal 最小尺寸，用于 CCD 阈值 */
  minExtent = Infinity;
  /** @internal */ readonly contacts: Contact[] = [];
  /** @internal */ readonly joints: Joint[] = [];
  /** @internal 在 world.bodies 中的下标 */
  index = -1;
  /** @internal 求解器：子步内的质心位移与相对旋转 */
  readonly deltaPosition = new Vec3();
  readonly deltaRotation = new Quat();
  /** @internal 步开始时的质心与旋转 */
  readonly center0 = new Vec3();
  readonly rotation0 = new Quat();
  /** @internal 并查集父节点 */
  islandParent = -1;
  /** 上一步结束时的原点位置与旋转，用于渲染插值 */
  readonly previousPosition = new Vec3();
  readonly previousRotation = new Quat();

  /** @internal 请使用 world.createBody */
  constructor(id: number, world: World, options: RigidBodyOptions = {}) {
    this.id = id;
    this.world = world;
    this._type = options.type ?? 'dynamic';
    if (options.position) this.transform.position.copy(options.position);
    if (options.rotation) this.transform.rotation.copy(options.rotation).normalize();
    if (options.linearVelocity) this.linearVelocity.copy(options.linearVelocity);
    if (options.angularVelocity) this.angularVelocity.copy(options.angularVelocity);
    this.linearDamping = options.linearDamping ?? 0;
    this.angularDamping = options.angularDamping ?? 0.05;
    this.gravityScale = options.gravityScale ?? 1;
    this.allowSleep = options.allowSleep ?? true;
    this.awakeFlag = this._type === 'static' ? false : (options.awake ?? true);
    this.fixedRotation = options.fixedRotation ?? false;
    this.isBullet = options.isBullet ?? false;
    this.enableCCD = options.enableCCD ?? true;
    this.userData = options.userData;
    if (this._type === 'static') {
      this.linearVelocity.setZero();
      this.angularVelocity.setZero();
    }
    this.center.copy(this.transform.position);
    this.previousPosition.copy(this.transform.position);
    this.previousRotation.copy(this.transform.rotation);
    this.updateMassProperties();
  }

  get type(): BodyType {
    return this._type;
  }

  get position(): Readonly<Vec3> {
    return this.transform.position;
  }

  get rotation(): Readonly<Quat> {
    return this.transform.rotation;
  }

  get isAwake(): boolean {
    return this.awakeFlag;
  }

  isStatic(): boolean {
    return this._type === 'static';
  }

  isDynamic(): boolean {
    return this._type === 'dynamic';
  }

  isKinematic(): boolean {
    return this._type === 'kinematic';
  }

  // ------------------------------------------------------------------
  // 碰撞体
  // ------------------------------------------------------------------

  addCollider(options: ColliderOptions): Collider {
    const world = this.requireWorld();
    world.assertUnlocked();
    const collider = new Collider(world.nextColliderId(), this, options);
    this.colliders.push(collider);
    world.registerCollider(collider);
    this.updateMassProperties();
    return collider;
  }

  removeCollider(collider: Collider): void {
    const world = this.requireWorld();
    world.assertUnlocked();
    const i = this.colliders.indexOf(collider);
    if (i < 0) return;
    this.colliders.splice(i, 1);
    world.unregisterCollider(collider);
    this.updateMassProperties();
  }

  /** 根据碰撞体重新计算质量、质心与惯性张量 */
  updateMassProperties(): void {
    const oldCenter = tmpV2.copy(this.center);
    this.mass = 0;
    this.invMass = 0;
    this.inertiaLocal.setZero();
    this.invInertiaLocal.setZero();
    this.localCenter.setZero();
    this.maxExtent = 0;
    this.minExtent = Infinity;

    if (this._type !== 'dynamic') {
      this.center.copy(this.transform.position);
      this.updateExtents();
      this.updateWorldInertia();
      return;
    }

    // 质心
    let totalMass = 0;
    const center = tmpV.setZero();
    for (const c of this.colliders) {
      if (c.density === 0) continue;
      c.shape.computeMass(c.density, tmpMass);
      if (tmpMass.mass === 0) continue;
      totalMass += tmpMass.mass;
      const lc = c.localTransform.transformPoint(tmpMass.center, tmpV3);
      center.addScaled(lc, tmpMass.mass);
    }
    if (totalMass > 0) {
      this.localCenter.copy(center).scale(1 / totalMass);
    } else {
      this.localCenter.setZero();
    }
    // 惯性张量（平行轴定理）
    const lcBody = this.localCenter;
    for (const c of this.colliders) {
      if (c.density === 0) continue;
      c.shape.computeMass(c.density, tmpMass);
      if (tmpMass.mass === 0) continue;
      tmpInertia.setRotated(c.localTransform.rotation, tmpMass.inertia);
      const d = c.localTransform.transformPoint(tmpMass.center, tmpV).sub(lcBody);
      const m = tmpMass.mass;
      const dd = d.lengthSq();
      tmpInertia.m00 += m * (dd - d.x * d.x);
      tmpInertia.m11 += m * (dd - d.y * d.y);
      tmpInertia.m22 += m * (dd - d.z * d.z);
      tmpInertia.m01 -= m * d.x * d.y;
      tmpInertia.m10 -= m * d.x * d.y;
      tmpInertia.m02 -= m * d.x * d.z;
      tmpInertia.m20 -= m * d.x * d.z;
      tmpInertia.m12 -= m * d.y * d.z;
      tmpInertia.m21 -= m * d.y * d.z;
      this.inertiaLocal.add(tmpInertia);
    }

    if (totalMass > 0) {
      this.mass = totalMass;
      this.invMass = 1 / totalMass;
    } else {
      // 没有质量的动态刚体按 1kg 处理，避免除零
      this.mass = 1;
      this.invMass = 1;
    }
    if (!this.fixedRotation) {
      this.invInertiaLocal.copy(this.inertiaLocal);
      if (!this.invInertiaLocal.invert()) this.invInertiaLocal.setZero();
    }

    // 质心移动后保持质心处速度连续：v_new = v + w × (c_new - c_old)
    this.transform.transformPoint(this.localCenter, this.center);
    tmpV.subVectors(this.center, oldCenter);
    this.linearVelocity.add(tmpV2.crossVectors(this.angularVelocity, tmpV));
    this.updateExtents();
    this.updateWorldInertia();
  }

  private updateExtents(): void {
    let maxExtent = 0;
    let minExtent = Infinity;
    for (const c of this.colliders) {
      // 用局部包围盒估计
      c.shape.computeAABB(c.localTransform, bodyLocalAabb);
      const ext = bodyLocalAabb.getExtents(tmpV);
      minExtent = Math.min(minExtent, ext.x, ext.y, ext.z, c.shape.radius > 0 ? c.shape.radius : Infinity);
      for (const s of [bodyLocalAabb.min, bodyLocalAabb.max]) {
        maxExtent = Math.max(maxExtent, tmpV2.subVectors(s, this.localCenter).length());
      }
    }
    this.maxExtent = Number.isFinite(maxExtent) ? maxExtent : 0;
    this.minExtent = minExtent;
  }

  /** @internal */
  updateWorldInertia(): void {
    if (this.invMass === 0 || this.fixedRotation) {
      this.invInertiaWorld.setZero();
      return;
    }
    this.invInertiaWorld.setRotated(this.transform.rotation, this.invInertiaLocal);
  }

  // ------------------------------------------------------------------
  // 变换
  // ------------------------------------------------------------------

  setPosition(position: Readonly<Vec3>, wake = true): void {
    this.setTransform(position, this.transform.rotation, wake);
  }

  setRotation(rotation: Readonly<Quat>, wake = true): void {
    this.setTransform(this.transform.position, rotation, wake);
  }

  /** 直接瞬移刚体（会重新计算包围盒） */
  setTransform(position: Readonly<Vec3>, rotation: Readonly<Quat>, wake = true): void {
    const world = this.world;
    world?.assertUnlocked();
    this.transform.position.copy(position);
    this.transform.rotation.copy(rotation).normalize();
    this.transform.transformPoint(this.localCenter, this.center);
    this.previousPosition.copy(this.transform.position);
    this.previousRotation.copy(this.transform.rotation);
    this.updateWorldInertia();
    world?.synchronizeBody(this, null);
    if (wake && this._type !== 'static') this.wakeUp();
  }

  /** 修改刚体类型 */
  setType(type: BodyType): void {
    if (type === this._type) return;
    const world = this.requireWorld();
    world.assertUnlocked();
    this._type = type;
    if (type === 'static') {
      this.linearVelocity.setZero();
      this.angularVelocity.setZero();
      this.awakeFlag = false;
    }
    this.updateMassProperties();
    world.rebuildBodyProxies(this);
    if (type !== 'static') this.wakeUp();
  }

  // ------------------------------------------------------------------
  // 力与速度
  // ------------------------------------------------------------------

  setLinearVelocity(v: Readonly<Vec3>): void {
    if (this._type === 'static') return;
    this.linearVelocity.copy(v);
    if (v.lengthSq() > 0) this.wakeUp();
  }

  setAngularVelocity(w: Readonly<Vec3>): void {
    if (this._type === 'static') return;
    this.angularVelocity.copy(w);
    if (w.lengthSq() > 0) this.wakeUp();
  }

  /** 在世界点施加力（N）；省略 point 时作用于质心 */
  applyForce(force: Readonly<Vec3>, point?: Readonly<Vec3>, wake = true): void {
    if (this._type !== 'dynamic') return;
    if (wake) this.wakeUp();
    if (!this.awakeFlag) return;
    this.force.add(force);
    if (point) {
      tmpV.subVectors(point, this.center);
      this.torque.add(tmpV.cross(force));
    }
  }

  applyTorque(torque: Readonly<Vec3>, wake = true): void {
    if (this._type !== 'dynamic') return;
    if (wake) this.wakeUp();
    if (!this.awakeFlag) return;
    this.torque.add(torque);
  }

  /** 在世界点施加冲量（N·s）；省略 point 时作用于质心 */
  applyImpulse(impulse: Readonly<Vec3>, point?: Readonly<Vec3>, wake = true): void {
    if (this._type !== 'dynamic') return;
    if (wake) this.wakeUp();
    if (!this.awakeFlag) return;
    this.linearVelocity.addScaled(impulse, this.invMass);
    if (point) {
      tmpV.subVectors(point, this.center).cross(impulse);
      this.invInertiaWorld.transformVector(tmpV, tmpV);
      this.angularVelocity.add(tmpV);
    }
  }

  applyAngularImpulse(impulse: Readonly<Vec3>, wake = true): void {
    if (this._type !== 'dynamic') return;
    if (wake) this.wakeUp();
    if (!this.awakeFlag) return;
    this.invInertiaWorld.transformVector(impulse, tmpV);
    this.angularVelocity.add(tmpV);
  }

  /**
   * 运动学刚体：设置速度，使其在 dt 后到达目标位姿。
   */
  moveKinematic(targetPosition: Readonly<Vec3>, targetRotation: Readonly<Quat>, dt: number): void {
    if (this._type !== 'kinematic' || dt <= 0) return;
    this.linearVelocity.subVectors(targetPosition, this.transform.position).scale(1 / dt);
    tmpQ.multiplyQuats(targetRotation, tmpQ.copy(this.transform.rotation).conjugate());
    tmpQ.toRotationVector(this.angularVelocity).scale(1 / dt);
    this.wakeUp();
  }

  /** 世界点处的速度 */
  getVelocityAtPoint(point: Readonly<Vec3>, out: Vec3): Vec3 {
    tmpV.subVectors(point, this.center);
    out.crossVectors(this.angularVelocity, tmpV);
    return out.add(this.linearVelocity);
  }

  getWorldPoint(localPoint: Readonly<Vec3>, out: Vec3): Vec3 {
    return this.transform.transformPoint(localPoint, out);
  }

  getLocalPoint(worldPoint: Readonly<Vec3>, out: Vec3): Vec3 {
    return this.transform.inverseTransformPoint(worldPoint, out);
  }

  getWorldVector(localVector: Readonly<Vec3>, out: Vec3): Vec3 {
    return this.transform.transformVector(localVector, out);
  }

  getLocalVector(worldVector: Readonly<Vec3>, out: Vec3): Vec3 {
    return this.transform.inverseTransformVector(worldVector, out);
  }

  /** 动能 */
  getKineticEnergy(): number {
    const v = this.linearVelocity;
    const w = this.angularVelocity;
    let e = 0.5 * this.mass * v.lengthSq();
    if (this.invMass > 0) {
      const lw = this.transform.inverseTransformVector(w, tmpV);
      e += 0.5 * this.inertiaLocal.quadraticForm(lw);
    }
    return e;
  }

  /**
   * 按插值系数 alpha 计算用于渲染的位姿（上一步结果与当前结果之间）。
   */
  interpolate(alpha: number, outPosition: Vec3, outRotation?: Quat): void {
    outPosition.lerpVectors(this.previousPosition, this.transform.position, alpha);
    outRotation?.slerpQuats(this.previousRotation, this.transform.rotation, alpha);
  }

  // ------------------------------------------------------------------
  // 休眠
  // ------------------------------------------------------------------

  /** 唤醒刚体（及与其相连、正在休眠的刚体） */
  wakeUp(): void {
    if (this._type === 'static' || this.awakeFlag) return;
    this.world?.wakeIsland(this);
  }

  /** 强制休眠（与其接触的刚体不受影响） */
  sleep(): void {
    if (this._type === 'static' || !this.awakeFlag) return;
    this.world?.putToSleep(this);
  }

  private requireWorld(): World {
    if (!this.world) throw new Error('RigidBody 已被销毁');
    return this.world;
  }
}

const bodyLocalAabb = new AABB();
