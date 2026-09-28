import { BroadPhase, ProxyType } from '../collision/broadphase/BroadPhase';
import type { CollideConfig } from '../collision/narrowphase/Collide';
import { ShapeCastOutput, shapeCast } from '../collision/narrowphase/ShapeCast';
import type { Collider } from '../dynamics/Collider';
import type { Contact } from '../dynamics/Contact';
import { ContactManager } from '../dynamics/ContactManager';
import type { Joint } from '../dynamics/joints/Joint';
import { RigidBody, type RigidBodyOptions } from '../dynamics/RigidBody';
import {
  applyRestitution,
  prepareContacts,
  solveContacts,
  warmStartContacts,
} from '../dynamics/solver/ContactSolver';
import { SolverContext } from '../dynamics/solver/SolverContext';
import { type DebugDrawOptions, type DebugDrawer, debugDrawWorld } from '../debug/DebugDraw';
import { EventEmitter } from '../events/EventEmitter';
import {
  type QueryFilter,
  type RaycastHit,
  type ShapeCastHit,
  castShape,
  overlapShape,
  queryAABB,
  raycastAll,
  raycastClosest,
} from '../query/Queries';
import type { Shape } from '../shapes/Shape';
import { AABB } from '../math/AABB';
import { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import { ShapeType } from '../shapes/Shape';

export interface WorldOptions {
  /** 重力，默认 (0, -9.81, 0) */
  gravity?: Readonly<Vec3>;
  /** 每步子步数，默认 4 */
  subSteps?: number;
  /** advance() 使用的固定步长，默认 1/60 */
  fixedTimeStep?: number;
  /** advance() 每帧最多执行的步数，默认 4 */
  maxStepsPerFrame?: number;
  /** 接触刚度（Hz），默认 30 */
  contactHertz?: number;
  /** 接触阻尼比，默认 10 */
  contactDampingRatio?: number;
  /** 关节阻尼比，默认 2 */
  jointDampingRatio?: number;
  /** 穿透推出速度上限（m/s），默认 3 */
  contactPushMaxVelocity?: number;
  /** 低于该接近速度不产生弹跳（m/s），默认 1 */
  restitutionThreshold?: number;
  /** 线性容差（m），默认 0.005 */
  linearSlop?: number;
  /** 宽相扩展包围盒边距（m），默认 0.1 */
  aabbMargin?: number;
  /** 最大线速度（m/s），默认 400 */
  maxLinearSpeed?: number;
  enableSleep?: boolean;
  /** 静止多久后休眠（s），默认 0.5 */
  timeToSleep?: number;
  /** 休眠速度阈值（m/s），默认 0.05 */
  sleepThreshold?: number;
  /** 连续碰撞检测，默认开启 */
  enableContinuous?: boolean;
  enableWarmStarting?: boolean;
}

export interface CollisionEvent {
  colliderA: Collider;
  colliderB: Collider;
  bodyA: RigidBody;
  bodyB: RigidBody;
  contact: Contact;
  /** 由 A 指向 B 的法线 */
  normal: Vec3;
  /** 第一个接触点（世界坐标） */
  point: Vec3;
  /** 开始接触时沿法线的接近速度（m/s，仅 collisionStart 有意义） */
  approachSpeed: number;
}

export interface SensorEvent {
  sensor: Collider;
  visitor: Collider;
}

export interface BodyEvent {
  body: RigidBody;
}

export interface WorldEvents {
  collisionStart: CollisionEvent;
  collisionEnd: CollisionEvent;
  sensorEnter: SensorEvent;
  sensorExit: SensorEvent;
  sleep: BodyEvent;
  wake: BodyEvent;
}

export interface WorldStats {
  bodies: number;
  awakeBodies: number;
  contacts: number;
  touchingContacts: number;
  joints: number;
  /** 最近一步的耗时（ms） */
  stepTime: number;
}

type QueuedEvent =
  | { type: 'collisionStart' | 'collisionEnd'; event: CollisionEvent }
  | { type: 'sensorEnter' | 'sensorExit'; event: SensorEvent }
  | { type: 'sleep' | 'wake'; event: BodyEvent };

const tmpV = new Vec3();
const tmpV2 = new Vec3();
const tmpAabb = new AABB();
const tmpAabb2 = new AABB();
const castXf = new Transform();
const castOut = new ShapeCastOutput();

/**
 * 物理世界：管理刚体、碰撞体、关节与接触，并推进模拟。
 */
export class World {
  readonly gravity: Vec3;
  subSteps: number;
  fixedTimeStep: number;
  maxStepsPerFrame: number;
  contactHertz: number;
  contactDampingRatio: number;
  jointDampingRatio: number;
  contactPushMaxVelocity: number;
  restitutionThreshold: number;
  readonly linearSlop: number;
  maxLinearSpeed: number;
  enableSleep: boolean;
  timeToSleep: number;
  sleepThreshold: number;
  enableContinuous: boolean;
  enableWarmStarting: boolean;

  readonly bodies: RigidBody[] = [];
  readonly joints: Joint[] = [];
  /** @internal */
  readonly broadPhase: BroadPhase<Collider>;
  /** @internal */
  readonly contactManager: ContactManager;
  readonly events = new EventEmitter<WorldEvents>();
  readonly stats: WorldStats = {
    bodies: 0,
    awakeBodies: 0,
    contacts: 0,
    touchingContacts: 0,
    joints: 0,
    stepTime: 0,
  };

  private readonly collideConfig: CollideConfig;
  private readonly ctx = new SolverContext();
  private locked = false;
  private accumulator = 0;
  private bodyIdCounter = 0;
  private colliderIdCounter = 0;
  private jointIdCounter = 0;
  private readonly eventQueue: QueuedEvent[] = [];
  private readonly awakeBodies: RigidBody[] = [];
  private readonly activeContacts: Contact[] = [];
  private readonly activeJoints: Joint[] = [];

  constructor(options: WorldOptions = {}) {
    this.gravity = options.gravity ? new Vec3().copy(options.gravity) : new Vec3(0, -9.81, 0);
    this.subSteps = options.subSteps ?? 4;
    this.fixedTimeStep = options.fixedTimeStep ?? 1 / 60;
    this.maxStepsPerFrame = options.maxStepsPerFrame ?? 4;
    this.contactHertz = options.contactHertz ?? 30;
    this.contactDampingRatio = options.contactDampingRatio ?? 10;
    this.jointDampingRatio = options.jointDampingRatio ?? 2;
    this.contactPushMaxVelocity = options.contactPushMaxVelocity ?? 3;
    this.restitutionThreshold = options.restitutionThreshold ?? 1;
    this.linearSlop = options.linearSlop ?? 0.005;
    this.maxLinearSpeed = options.maxLinearSpeed ?? 400;
    this.enableSleep = options.enableSleep ?? true;
    this.timeToSleep = options.timeToSleep ?? 0.5;
    this.sleepThreshold = options.sleepThreshold ?? 0.05;
    this.enableContinuous = options.enableContinuous ?? true;
    this.enableWarmStarting = options.enableWarmStarting ?? true;
    this.collideConfig = {
      linearSlop: this.linearSlop,
      speculativeDistance: 4 * this.linearSlop,
    };
    this.broadPhase = new BroadPhase<Collider>(options.aabbMargin ?? 0.1);
    this.contactManager = new ContactManager(this.broadPhase, {
      onBegin: (c) => this.onContactBegin(c),
      onEnd: (c) => this.onContactEnd(c),
    });
  }

  // ------------------------------------------------------------------
  // 事件
  // ------------------------------------------------------------------

  on<K extends keyof WorldEvents>(type: K, listener: (e: WorldEvents[K]) => void): () => void {
    return this.events.on(type, listener);
  }

  off<K extends keyof WorldEvents>(type: K, listener: (e: WorldEvents[K]) => void): void {
    this.events.off(type, listener);
  }

  // ------------------------------------------------------------------
  // 刚体 / 碰撞体 / 关节
  // ------------------------------------------------------------------

  createBody(options: RigidBodyOptions = {}): RigidBody {
    this.assertUnlocked();
    const body = new RigidBody(this.bodyIdCounter++, this, options);
    body.index = this.bodies.length;
    this.bodies.push(body);
    this.resetSolverState(body);
    return body;
  }

  destroyBody(body: RigidBody): void {
    this.assertUnlocked();
    if (body.world !== this) return;
    for (let i = body.joints.length - 1; i >= 0; i--) this.removeJoint(body.joints[i]!);
    for (let i = body.colliders.length - 1; i >= 0; i--) {
      this.unregisterCollider(body.colliders[i]!);
    }
    body.colliders.length = 0;
    const last = this.bodies.pop()!;
    if (last !== body) {
      this.bodies[body.index] = last;
      last.index = body.index;
    }
    body.index = -1;
    body.world = null;
    this.flushEvents();
  }

  addJoint<T extends Joint>(joint: T): T {
    this.assertUnlocked();
    if (joint.world) throw new Error('Joint 已加入某个 World');
    if (joint.bodyA.world !== this || joint.bodyB.world !== this) {
      throw new Error('Joint 的刚体不属于该 World');
    }
    joint.world = this;
    joint.id = this.jointIdCounter++;
    this.joints.push(joint);
    joint.bodyA.joints.push(joint);
    joint.bodyB.joints.push(joint);
    if (!joint.collideConnected) {
      // 销毁两者之间已有的接触
      for (const c of [...joint.bodyA.contacts]) {
        if (!c.isSensor && (c.bodyA === joint.bodyB || c.bodyB === joint.bodyB)) {
          this.contactManager.destroy(c);
        }
      }
    }
    joint.wakeBodies();
    this.flushEvents();
    return joint;
  }

  removeJoint(joint: Joint): void {
    this.assertUnlocked();
    if (joint.world !== this) return;
    const i = this.joints.indexOf(joint);
    if (i >= 0) this.joints.splice(i, 1);
    removeItem(joint.bodyA.joints, joint);
    removeItem(joint.bodyB.joints, joint);
    joint.world = null;
    joint.wakeBodies();
    if (!joint.collideConnected) {
      // 重新配对，恢复两者之间的碰撞
      for (const c of joint.bodyA.colliders) if (c.proxy) this.broadPhase.touchProxy(c.proxy);
    }
  }

  /** @internal */
  nextColliderId(): number {
    return this.colliderIdCounter++;
  }

  /** @internal */
  registerCollider(collider: Collider): void {
    if (collider.shape.type === ShapeType.Plane && !collider.body.isStatic()) {
      throw new Error('PlaneShape 只能用于静态刚体');
    }
    collider.updateWorldTransform();
    const type = collider.body.isStatic() ? ProxyType.Static : ProxyType.Dynamic;
    collider.proxy = this.broadPhase.createProxy(collider.aabb, type, collider);
  }

  /** @internal */
  unregisterCollider(collider: Collider): void {
    this.contactManager.destroyForCollider(collider);
    if (collider.proxy) {
      this.broadPhase.destroyProxy(collider.proxy);
      collider.proxy = null;
    }
  }

  /** @internal 刚体类型改变后重建宽相代理 */
  rebuildBodyProxies(body: RigidBody): void {
    for (const c of body.colliders) {
      this.unregisterCollider(c);
      this.registerCollider(c);
    }
    this.resetSolverState(body);
    this.flushEvents();
  }

  /** @internal 过滤条件改变后重新配对 */
  refilterCollider(collider: Collider): void {
    this.contactManager.destroyForCollider(collider);
    if (collider.proxy) this.broadPhase.touchProxy(collider.proxy);
    this.flushEvents();
  }

  /** @internal 刚体被瞬移后同步碰撞体与宽相 */
  synchronizeBody(body: RigidBody, displacement: Readonly<Vec3> | null): void {
    for (const c of body.colliders) {
      c.updateWorldTransform();
      if (c.proxy) this.broadPhase.moveProxy(c.proxy, c.aabb, displacement ?? undefined);
    }
    this.resetSolverState(body);
  }

  private resetSolverState(body: RigidBody): void {
    body.center0.copy(body.center);
    body.rotation0.copy(body.transform.rotation);
    body.deltaPosition.setZero();
    body.deltaRotation.identity();
  }

  /** @internal */
  assertUnlocked(): void {
    if (this.locked) throw new Error('World 正在 step 中，不能修改（请在事件回调之外或 step 之后修改）');
  }

  get isLocked(): boolean {
    return this.locked;
  }

  // ------------------------------------------------------------------
  // 休眠
  // ------------------------------------------------------------------

  /** @internal 唤醒刚体及与之相连的休眠刚体 */
  wakeIsland(body: RigidBody): void {
    if (body.isStatic() || body.isAwake) return;
    const stack = [body];
    while (stack.length > 0) {
      const b = stack.pop()!;
      if (b.isAwake || b.isStatic()) continue;
      b.awakeFlag = true;
      b.sleepTime = 0;
      this.resetSolverState(b);
      this.queueEvent({ type: 'wake', event: { body: b } });
      for (const c of b.contacts) {
        if (!c.touching || c.isSensor) continue;
        const other = c.bodyA === b ? c.bodyB : c.bodyA;
        if (!other.isAwake && !other.isStatic()) stack.push(other);
      }
      for (const j of b.joints) {
        const other = j.bodyA === b ? j.bodyB : j.bodyA;
        if (!other.isAwake && !other.isStatic()) stack.push(other);
      }
    }
    this.flushEvents();
  }

  /** @internal */
  putToSleep(body: RigidBody): void {
    body.awakeFlag = false;
    body.sleepTime = 0;
    body.previousPosition.copy(body.transform.position);
    body.previousRotation.copy(body.transform.rotation);
    body.linearVelocity.setZero();
    body.angularVelocity.setZero();
    body.force.setZero();
    body.torque.setZero();
    this.queueEvent({ type: 'sleep', event: { body } });
    this.flushEvents();
  }

  // ------------------------------------------------------------------
  // 模拟
  // ------------------------------------------------------------------

  /**
   * 以固定步长累加推进，返回渲染插值系数 alpha ∈ [0, 1)。
   * 渲染时可用 body.interpolate(alpha, ...) 得到平滑的位姿。
   */
  advance(frameTime: number): number {
    const dt = this.fixedTimeStep;
    this.accumulator += Math.min(Math.max(frameTime, 0), dt * this.maxStepsPerFrame);
    while (this.accumulator >= dt) {
      this.step(dt);
      this.accumulator -= dt;
    }
    return this.accumulator / dt;
  }

  /** 推进 dt 秒（建议使用固定步长） */
  step(dt: number): void {
    if (!(dt > 0)) return;
    const start = now();
    this.locked = true;
    try {
      // 1. 宽相：新的潜在配对
      this.broadPhase.updatePairs((a, b) => this.contactManager.addPair(a, b));
      // 2. 窄相
      this.contactManager.update(this.collideConfig, dt, this.broadPhase.dynamicTree.margin);
      // 3. 求解
      this.solve(dt);
    } finally {
      this.locked = false;
    }
    this.stats.stepTime = now() - start;
    this.updateStats();
    this.flushEvents();
  }

  private solve(dt: number): void {
    const ctx = this.ctx;
    const subSteps = Math.max(1, Math.floor(this.subSteps));
    ctx.dt = dt;
    ctx.subSteps = subSteps;
    ctx.h = dt / subSteps;
    ctx.invH = 1 / ctx.h;
    ctx.maxBiasVelocity = this.contactPushMaxVelocity;
    ctx.restitutionThreshold = this.restitutionThreshold;
    ctx.enableWarmStarting = this.enableWarmStarting;
    const contactHertz = Math.min(this.contactHertz, 0.25 * subSteps / dt);
    ctx.contactSoftness.set(contactHertz, this.contactDampingRatio, ctx.h);
    ctx.staticSoftness.set(2 * contactHertz, this.contactDampingRatio, ctx.h);
    ctx.jointSoftness.set(2 * contactHertz, this.jointDampingRatio, ctx.h);

    // ---- 收集参与求解的对象 ----
    const bodies = this.awakeBodies;
    bodies.length = 0;
    for (const b of this.bodies) {
      if (!b.isAwake) continue;
      bodies.push(b);
      b.previousPosition.copy(b.transform.position);
      b.previousRotation.copy(b.transform.rotation);
      b.updateWorldInertia();
      this.resetSolverState(b);
    }
    const contacts = this.activeContacts;
    contacts.length = 0;
    for (const c of this.contactManager.contacts) {
      if (c.isSensor || !c.touching) continue;
      if (c.bodyA.isAwake || c.bodyB.isAwake) contacts.push(c);
    }
    const joints = this.activeJoints;
    joints.length = 0;
    for (const j of this.joints) {
      if (j.bodyA.isAwake || j.bodyB.isAwake) joints.push(j);
    }

    // ---- 准备 ----
    for (const j of joints) j.prepare(ctx);
    prepareContacts(contacts, ctx);

    const h = ctx.h;
    const gravity = this.gravity;
    const maxAngular = 0.25 * Math.PI * ctx.invH;
    const maxLinear = this.maxLinearSpeed;

    for (let sub = 0; sub < subSteps; sub++) {
      // 积分速度
      for (const b of bodies) {
        if (!b.isDynamic()) continue;
        const v = b.linearVelocity;
        const w = b.angularVelocity;
        v.addScaled(b.force, h * b.invMass).addScaled(gravity, h * b.gravityScale);
        if (!b.fixedRotation) {
          b.invInertiaWorld.transformVector(b.torque, tmpV);
          w.addScaled(tmpV, h);
        } else {
          w.setZero();
        }
        if (b.linearDamping > 0) v.scale(1 / (1 + h * b.linearDamping));
        if (b.angularDamping > 0) w.scale(1 / (1 + h * b.angularDamping));
        const vl = v.lengthSq();
        if (vl > maxLinear * maxLinear) v.scale(maxLinear / Math.sqrt(vl));
        const wl = w.lengthSq();
        if (wl > maxAngular * maxAngular) w.scale(maxAngular / Math.sqrt(wl));
      }
      // warm start
      for (const j of joints) j.warmStart(ctx);
      warmStartContacts(contacts);
      // 求解（带偏置）
      for (const j of joints) j.solve(ctx, true);
      solveContacts(contacts, ctx, true);
      // 积分位置
      for (const b of bodies) {
        b.deltaPosition.addScaled(b.linearVelocity, h);
        if (!b.fixedRotation) b.deltaRotation.integrate(b.angularVelocity, h);
      }
      // relax（不带偏置）
      for (const j of joints) j.solve(ctx, false);
      solveContacts(contacts, ctx, false);
    }
    applyRestitution(contacts, ctx);

    // ---- 写回位姿 ----
    const enableSleep = this.enableSleep;
    for (const b of bodies) {
      b.center.addVectors(b.center0, b.deltaPosition);
      b.transform.rotation.multiplyQuats(b.deltaRotation, b.rotation0).normalize();
      b.transform.rotation.rotate(b.localCenter, tmpV);
      b.transform.position.subVectors(b.center, tmpV);
      b.force.setZero();
      b.torque.setZero();
      if (enableSleep && b.allowSleep) {
        const speed = b.linearVelocity.length() + b.maxExtent * b.angularVelocity.length();
        if (speed > this.sleepThreshold) b.sleepTime = 0;
        else b.sleepTime += dt;
      } else {
        b.sleepTime = 0;
      }
    }

    // ---- 连续碰撞 ----
    if (this.enableContinuous) {
      for (const b of bodies) {
        if (b.isDynamic() && b.enableCCD) this.solveContinuous(b);
      }
    }

    // ---- 更新宽相（扩展包围盒沿速度方向预测一步） ----
    for (const b of bodies) {
      tmpV.copy(b.linearVelocity).scale(dt);
      for (const c of b.colliders) {
        c.updateWorldTransform();
        this.broadPhase.moveProxy(c.proxy!, c.aabb, tmpV);
      }
      b.updateWorldInertia();
    }

    // ---- 休眠 ----
    if (enableSleep) this.updateSleep(bodies, contacts, joints);
  }

  /** 连续碰撞：高速刚体沿本步位移对静态物体（子弹还包括动态物体）做形状投射 */
  private solveContinuous(body: RigidBody): void {
    const dp = body.deltaPosition;
    const distance = dp.length();
    if (distance === 0 || distance < 0.5 * body.minExtent) return;

    let minT = 1;
    const slop = this.linearSlop;
    for (const collider of body.colliders) {
      if (collider.isSensor) continue;
      // 终点位姿下的碰撞体，平移回起点后沿 dp 投射
      collider.updateWorldTransform();
      castXf.copy(collider.worldTransform);
      castXf.position.sub(dp);
      collider.shape.computeAABB(castXf, tmpAabb);
      tmpAabb2.union(tmpAabb, collider.aabb);
      const check = (other: Collider): boolean => {
        if (other.body === body || other.isSensor) return true;
        if (!ContactManager.shouldCollide(collider, other)) return true;
        if (other.body.isDynamic() && (!body.isBullet || other.body.isBullet)) return true;
        if (shapeCast(collider.shape, castXf, dp, other.shape, other.worldTransform, minT, slop, castOut)) {
          if (castOut.t < minT) minT = castOut.t;
        }
        return true;
      };
      this.broadPhase.staticTree.query(tmpAabb2, (_id, other) => check(other));
      if (body.isBullet) this.broadPhase.dynamicTree.query(tmpAabb2, (_id, other) => check(other));
    }
    if (minT < 1) {
      // 回退到撞击时刻（保留速度，由推测接触处理后续响应）
      tmpV2.copy(dp).scale(minT - 1);
      body.center.add(tmpV2);
      body.transform.position.add(tmpV2);
      body.deltaPosition.scale(minT);
    }
  }

  /** 用并查集把接触/关节相连的刚体分成岛屿，整岛静止足够久则休眠 */
  private updateSleep(
    bodies: readonly RigidBody[],
    contacts: readonly Contact[],
    joints: readonly Joint[],
  ): void {
    const n = bodies.length;
    if (n === 0) return;
    const parent = new Int32Array(n);
    for (let i = 0; i < n; i++) {
      parent[i] = i;
      bodies[i]!.islandParent = i;
    }
    const find = (i: number): number => {
      while (parent[i] !== i) {
        parent[i] = parent[parent[i]!]!;
        i = parent[i]!;
      }
      return i;
    };
    const union = (a: RigidBody, b: RigidBody): void => {
      if (!a.isAwake || !b.isAwake || a.isStatic() || b.isStatic()) return;
      const ra = find(a.islandParent);
      const rb = find(b.islandParent);
      if (ra !== rb) parent[ra] = rb;
    };
    for (const c of contacts) union(c.bodyA, c.bodyB);
    for (const j of joints) union(j.bodyA, j.bodyB);

    const minSleep = new Float64Array(n).fill(Infinity);
    for (let i = 0; i < n; i++) {
      const b = bodies[i]!;
      const r = find(i);
      const t = b.allowSleep ? b.sleepTime : 0;
      if (t < minSleep[r]!) minSleep[r] = t;
    }
    for (let i = 0; i < n; i++) {
      const b = bodies[i]!;
      if (minSleep[find(i)]! >= this.timeToSleep) {
        b.awakeFlag = false;
        b.sleepTime = 0;
        b.linearVelocity.setZero();
        b.angularVelocity.setZero();
        b.previousPosition.copy(b.transform.position);
        b.previousRotation.copy(b.transform.rotation);
        this.queueEvent({ type: 'sleep', event: { body: b } });
      }
    }
    for (const b of bodies) b.islandParent = -1;
  }

  private updateStats(): void {
    let awake = 0;
    for (const b of this.bodies) if (b.isAwake) awake++;
    let touching = 0;
    for (const c of this.contactManager.contacts) if (c.touching) touching++;
    this.stats.bodies = this.bodies.length;
    this.stats.awakeBodies = awake;
    this.stats.contacts = this.contactManager.count;
    this.stats.touchingContacts = touching;
    this.stats.joints = this.joints.length;
  }

  // ------------------------------------------------------------------
  // 事件派发
  // ------------------------------------------------------------------

  private onContactBegin(c: Contact): void {
    if (c.isSensor) {
      const sensor = c.colliderA.isSensor ? c.colliderA : c.colliderB;
      this.queueEvent({ type: 'sensorEnter', event: { sensor, visitor: c.getOther(sensor) } });
      return;
    }
    if (!this.events.hasListeners('collisionStart')) return;
    const m = c.manifold;
    const point = m.pointCount > 0 ? m.points[0]!.point.clone() : new Vec3();
    const vA = c.bodyA.getVelocityAtPoint(point, new Vec3());
    const vB = c.bodyB.getVelocityAtPoint(point, new Vec3());
    const approachSpeed = -vB.sub(vA).dot(m.normal);
    this.queueEvent({
      type: 'collisionStart',
      event: {
        colliderA: c.colliderA,
        colliderB: c.colliderB,
        bodyA: c.bodyA,
        bodyB: c.bodyB,
        contact: c,
        normal: m.normal.clone(),
        point,
        approachSpeed,
      },
    });
  }

  private onContactEnd(c: Contact): void {
    if (c.isSensor) {
      const sensor = c.colliderA.isSensor ? c.colliderA : c.colliderB;
      this.queueEvent({ type: 'sensorExit', event: { sensor, visitor: c.getOther(sensor) } });
      return;
    }
    if (!this.events.hasListeners('collisionEnd')) return;
    this.queueEvent({
      type: 'collisionEnd',
      event: {
        colliderA: c.colliderA,
        colliderB: c.colliderB,
        bodyA: c.bodyA,
        bodyB: c.bodyB,
        contact: c,
        normal: c.manifold.normal.clone(),
        point: new Vec3(),
        approachSpeed: 0,
      },
    });
  }

  private queueEvent(e: QueuedEvent): void {
    this.eventQueue.push(e);
  }

  private flushEvents(): void {
    if (this.locked || this.eventQueue.length === 0) return;
    const queue = this.eventQueue.splice(0, this.eventQueue.length);
    for (const e of queue) {
      this.events.emit(e.type, e.event as never);
    }
  }

  // ------------------------------------------------------------------
  // 查询
  // ------------------------------------------------------------------

  /** 射线检测，返回最近的命中（direction 无需归一化） */
  raycast(
    origin: Readonly<Vec3>,
    direction: Readonly<Vec3>,
    maxDistance = Infinity,
    filter?: QueryFilter,
  ): RaycastHit | null {
    return raycastClosest(this.broadPhase, origin, direction, maxDistance, filter);
  }

  /** 射线检测，返回全部命中（按距离排序） */
  raycastAll(
    origin: Readonly<Vec3>,
    direction: Readonly<Vec3>,
    maxDistance = Infinity,
    filter?: QueryFilter,
  ): RaycastHit[] {
    return raycastAll(this.broadPhase, origin, direction, maxDistance, filter);
  }

  /** 查询与 AABB 相交的碰撞体 */
  queryAABB(aabb: Readonly<AABB>, filter?: QueryFilter): Collider[] {
    return queryAABB(this.broadPhase, aabb, filter);
  }

  /** 查询与给定形状重叠的碰撞体 */
  overlapShape(shape: Shape, transform: Readonly<Transform>, filter?: QueryFilter): Collider[] {
    return overlapShape(this.broadPhase, shape, transform, filter);
  }

  /** 形状投射：形状沿 translation 平移，返回最先命中的碰撞体 */
  castShape(
    shape: Shape,
    transform: Readonly<Transform>,
    translation: Readonly<Vec3>,
    filter?: QueryFilter,
  ): ShapeCastHit | null {
    return castShape(this.broadPhase, shape, transform, translation, filter, this.linearSlop);
  }

  // ------------------------------------------------------------------
  // 调试绘制
  // ------------------------------------------------------------------

  /** 把世界绘制为线段（供渲染端调试显示） */
  debugDraw(drawer: DebugDrawer, options?: DebugDrawOptions): void {
    debugDrawWorld(this, drawer, options);
  }

  // ------------------------------------------------------------------
  // 访问器
  // ------------------------------------------------------------------

  get contacts(): readonly Contact[] {
    return this.contactManager.contacts;
  }
}

function removeItem<T>(arr: T[], item: T): void {
  const i = arr.indexOf(item);
  if (i >= 0) arr.splice(i, 1);
}

const perf = (globalThis as { performance?: { now(): number } }).performance;

function now(): number {
  return perf ? perf.now() : Date.now();
}
