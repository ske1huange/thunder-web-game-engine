import { EventEmitter } from '../events/EventEmitter';
import { Quat } from '../math/Quat';
import { Vec3 } from '../math/Vec3';
import type { BodyType } from '../dynamics/RigidBody';
import type { WorldStats } from '../world/World';
import type { BodyDesc, JointDesc, Vec3Tuple } from './descriptors';
import {
  type BodyOp,
  type EventRecord,
  FLAG_ALIVE,
  FLAG_AWAKE,
  type FromWorker,
  type MessagePortLike,
  type RaycastRecord,
  SLOT_STRIDE,
  type ToWorker,
  type WorldDesc,
} from './protocol';

type Vec3Like = Readonly<{ x: number; y: number; z: number }>;
type QuatLike = Readonly<{ x: number; y: number; z: number; w: number }>;

const tuple = (v: Vec3Like): Vec3Tuple => [v.x, v.y, v.z];

/**
 * 主线程上的刚体代理：位姿来自 Worker 最近一次回传的快照，操作以消息发给 Worker。
 */
export class BodyProxy {
  readonly id: number;
  /** @internal 快照中的槽位 */
  readonly slot: number;
  readonly desc: BodyDesc;
  readonly position = new Vec3();
  readonly rotation = new Quat();
  readonly previousPosition = new Vec3();
  readonly previousRotation = new Quat();
  isAwake = true;
  userData: unknown;
  /** @internal */
  world: WorkerWorld | null;

  /** @internal */
  constructor(world: WorkerWorld, id: number, slot: number, desc: BodyDesc) {
    this.world = world;
    this.id = id;
    this.slot = slot;
    this.desc = desc;
    if (desc.position) this.position.set(desc.position[0], desc.position[1], desc.position[2]);
    if (desc.rotation) {
      this.rotation.set(desc.rotation[0], desc.rotation[1], desc.rotation[2], desc.rotation[3]);
    }
    this.previousPosition.copy(this.position);
    this.previousRotation.copy(this.rotation);
  }

  get type(): BodyType {
    return this.desc.type ?? 'dynamic';
  }

  /** 渲染插值：alpha 一般取 workerWorld.alpha */
  interpolate(alpha: number, outPosition: Vec3, outRotation?: Quat): void {
    outPosition.lerpVectors(this.previousPosition, this.position, alpha);
    outRotation?.slerpQuats(this.previousRotation, this.rotation, alpha);
  }

  private op(op: BodyOp, args: readonly unknown[]): void {
    this.world?.post({ t: 'body', id: this.id, op, args });
  }

  setLinearVelocity(v: Vec3Like): void {
    this.op('setLinearVelocity', tuple(v));
  }

  setAngularVelocity(w: Vec3Like): void {
    this.op('setAngularVelocity', tuple(w));
  }

  applyImpulse(impulse: Vec3Like, point?: Vec3Like): void {
    this.op('applyImpulse', point ? [...tuple(impulse), ...tuple(point)] : tuple(impulse));
  }

  applyForce(force: Vec3Like, point?: Vec3Like): void {
    this.op('applyForce', point ? [...tuple(force), ...tuple(point)] : tuple(force));
  }

  applyTorque(torque: Vec3Like): void {
    this.op('applyTorque', tuple(torque));
  }

  applyAngularImpulse(impulse: Vec3Like): void {
    this.op('applyAngularImpulse', tuple(impulse));
  }

  setPosition(position: Vec3Like): void {
    this.op('setPosition', tuple(position));
  }

  setTransform(position: Vec3Like, rotation: QuatLike): void {
    this.op('setTransform', [...tuple(position), rotation.x, rotation.y, rotation.z, rotation.w]);
  }

  moveKinematic(position: Vec3Like, rotation: QuatLike, dt: number): void {
    this.op('moveKinematic', [
      ...tuple(position),
      rotation.x,
      rotation.y,
      rotation.z,
      rotation.w,
      dt,
    ]);
  }

  wakeUp(): void {
    this.op('wakeUp', []);
  }

  sleep(): void {
    this.op('sleep', []);
  }

  setType(type: BodyType): void {
    this.op('setType', [type]);
  }
}

export interface WorkerCollisionEvent {
  bodyA: BodyProxy | null;
  bodyB: BodyProxy | null;
  colliderA: number;
  colliderB: number;
  normal: Vec3;
  point: Vec3;
  approachSpeed: number;
}

export interface WorkerSensorEvent {
  sensorBody: BodyProxy | null;
  sensorCollider: number;
  visitorBody: BodyProxy | null;
  visitorCollider: number;
}

export interface WorkerWorldEvents {
  collisionStart: WorkerCollisionEvent;
  collisionEnd: WorkerCollisionEvent;
  sensorEnter: WorkerSensorEvent;
  sensorExit: WorkerSensorEvent;
  /** 收到新快照（位姿已更新） */
  snapshot: { steps: number; time: number };
  /** Worker 端未被请求捕获的错误 */
  error: { message: string };
}

export interface WorkerRaycastHit {
  body: BodyProxy | null;
  colliderIndex: number;
  point: Vec3;
  normal: Vec3;
  distance: number;
}

interface Pending {
  resolve(value: unknown): void;
  reject(err: Error): void;
}

/**
 * 在 Web Worker 中运行的物理世界的主线程端。
 *
 * - 命令（创建刚体、施加冲量等）立即以消息发送，按顺序在 Worker 中执行
 * - 每帧调用 advance(dt)：请求 Worker 按固定步长推进；上一次请求未完成时累积时间，下次一起发送
 * - Worker 回传的位姿快照是一块来回转移的 Float64Array，不复制、不产生垃圾
 * - 渲染使用最近一次快照与其插值系数 alpha（比同步模拟晚一帧）
 *
 * ```ts
 * const world = new WorkerWorld(new Worker(new URL('./physics.worker.ts', import.meta.url), { type: 'module' }));
 * const box = world.createBody({ position: [0, 5, 0], colliders: [{ shape: { type: 'box', halfExtents: [0.5, 0.5, 0.5] } }] });
 * // 每帧
 * const alpha = world.advance(dt);
 * box.interpolate(alpha, mesh.position, mesh.quaternion);
 * ```
 */
export class WorkerWorld {
  readonly bodies = new Map<number, BodyProxy>();
  readonly events = new EventEmitter<WorkerWorldEvents>();
  /** 最近一次快照的插值系数 */
  alpha = 1;
  /** Worker 中已模拟的时间（s） */
  time = 0;
  /** 最近一次快照的统计（stepTime 为 Worker 处理该次请求的总耗时） */
  stats: WorldStats = {
    bodies: 0,
    awakeBodies: 0,
    contacts: 0,
    touchingContacts: 0,
    joints: 0,
    stepTime: 0,
  };

  private buffer: ArrayBuffer | null = new ArrayBuffer(64 * SLOT_STRIDE * 8);
  private pendingDt = 0;
  private nextBodyId = 1;
  private nextJointId = 1;
  private nextQid = 1;
  private nextSlot = 0;
  private readonly freeSlots: number[] = [];
  private releasedSlots: number[] = [];
  private inFlightReleased: number[] = [];
  private readonly pending = new Map<number, Pending>();
  private readonly snapshotWaiters: (() => void)[] = [];
  private stepQueue: { dt: number; resolve: () => void }[] = [];

  constructor(
    private readonly port: MessagePortLike,
    options: WorldDesc = {},
  ) {
    port.onmessage = (e) => this.receive(e.data as FromWorker);
    this.post({ t: 'init', world: options });
  }

  /** @internal */
  post(msg: ToWorker, transfer?: unknown[]): void {
    this.port.postMessage(msg, transfer);
  }

  on<K extends keyof WorkerWorldEvents>(
    type: K,
    listener: (e: WorkerWorldEvents[K]) => void,
  ): () => void {
    return this.events.on(type, listener);
  }

  /** 是否有推进请求尚未返回 */
  get busy(): boolean {
    return this.buffer === null;
  }

  createBody(desc: BodyDesc): BodyProxy {
    const id = this.nextBodyId++;
    const slot = this.freeSlots.length > 0 ? this.freeSlots.pop()! : this.nextSlot++;
    const proxy = new BodyProxy(this, id, slot, desc);
    this.bodies.set(id, proxy);
    this.post({ t: 'createBody', id, slot, desc });
    return proxy;
  }

  destroyBody(body: BodyProxy): void {
    if (body.world !== this) return;
    this.post({ t: 'destroyBody', id: body.id });
    this.bodies.delete(body.id);
    body.world = null;
    // 槽位要等到不再包含该刚体的快照返回后才能复用
    this.releasedSlots.push(body.slot);
  }

  /** 添加关节，返回关节编号 */
  addJoint(desc: JointDesc & { bodyA: BodyProxy; bodyB: BodyProxy }): number {
    const id = this.nextJointId++;
    const { bodyA, bodyB, ...rest } = desc;
    this.post({ t: 'addJoint', id, bodyA: bodyA.id, bodyB: bodyB.id, desc: rest as JointDesc });
    return id;
  }

  removeJoint(id: number): void {
    this.post({ t: 'removeJoint', id });
  }

  setGravity(gravity: Vec3Like): void {
    this.post({ t: 'setGravity', gravity: tuple(gravity) });
  }

  /**
   * 请求 Worker 推进 frameTime 秒（内部按固定步长），返回用于渲染的插值系数。
   * 上一次请求尚未返回时只累积时间。
   */
  advance(frameTime: number): number {
    this.pendingDt += Math.max(0, frameTime);
    if (this.buffer && this.stepQueue.length === 0 && this.pendingDt > 0) {
      const buffer = this.takeBuffer();
      this.post({ t: 'advance', dt: this.pendingDt, buffer }, [buffer]);
      this.pendingDt = 0;
    }
    return this.alpha;
  }

  /** 精确推进一步 dt（锁步模式 / 测试用），快照返回后 resolve */
  step(dt: number): Promise<void> {
    return new Promise((resolve) => {
      this.stepQueue.push({ dt, resolve });
      this.flushStepQueue();
    });
  }

  /** 等待下一次快照 */
  nextSnapshot(): Promise<void> {
    return new Promise((resolve) => this.snapshotWaiters.push(resolve));
  }

  raycast(
    origin: Vec3Like,
    direction: Vec3Like,
    maxDistance = Infinity,
    exclude?: BodyProxy,
  ): Promise<WorkerRaycastHit | null> {
    return this.query('raycast', origin, direction, maxDistance, exclude).then((v) =>
      v ? this.toHit(v as RaycastRecord) : null,
    );
  }

  raycastAll(
    origin: Vec3Like,
    direction: Vec3Like,
    maxDistance = Infinity,
    exclude?: BodyProxy,
  ): Promise<WorkerRaycastHit[]> {
    return this.query('raycastAll', origin, direction, maxDistance, exclude).then((v) =>
      (v as RaycastRecord[]).map((r) => this.toHit(r)),
    );
  }

  /** 发送自定义命令（Worker 端用 host.on(name, handler) 处理），不等待结果 */
  send(name: string, payload?: unknown): void {
    this.post({ t: 'custom', name, payload });
  }

  /** 发送自定义命令并等待处理函数的返回值 */
  request<T = unknown>(name: string, payload?: unknown): Promise<T> {
    const qid = this.nextQid++;
    return new Promise<T>((resolve, reject) => {
      this.pending.set(qid, { resolve: resolve as (v: unknown) => void, reject });
      this.post({ t: 'custom', name, payload, qid });
    });
  }

  /** 结束 Worker（端点有 terminate 方法时） */
  terminate(): void {
    (this.port as { terminate?: () => void }).terminate?.();
    this.port.onmessage = null;
  }

  // ------------------------------------------------------------------

  private takeBuffer(): ArrayBuffer {
    const buffer = this.buffer!;
    this.buffer = null;
    this.inFlightReleased = this.releasedSlots;
    this.releasedSlots = [];
    return buffer;
  }

  private flushStepQueue(): void {
    const next = this.stepQueue[0];
    if (!next || !this.buffer) return;
    const buffer = this.takeBuffer();
    this.post({ t: 'step', dt: next.dt, buffer }, [buffer]);
  }

  private query(
    kind: 'raycast' | 'raycastAll',
    origin: Vec3Like,
    direction: Vec3Like,
    maxDistance: number,
    exclude?: BodyProxy,
  ): Promise<unknown> {
    const qid = this.nextQid++;
    return new Promise((resolve, reject) => {
      this.pending.set(qid, { resolve, reject });
      this.post({
        t: 'query',
        qid,
        kind,
        origin: tuple(origin),
        direction: tuple(direction),
        maxDistance,
        exclude: exclude?.id,
      });
    });
  }

  private toHit(r: RaycastRecord): WorkerRaycastHit {
    return {
      body: this.bodies.get(r.body) ?? null,
      colliderIndex: r.collider,
      point: new Vec3(r.point[0], r.point[1], r.point[2]),
      normal: new Vec3(r.normal[0], r.normal[1], r.normal[2]),
      distance: r.distance,
    };
  }

  private receive(msg: FromWorker): void {
    switch (msg.t) {
      case 'snapshot': {
        this.buffer = msg.buffer;
        for (const slot of this.inFlightReleased) this.freeSlots.push(slot);
        this.inFlightReleased = [];
        this.readSnapshot(new Float64Array(msg.buffer));
        this.alpha = msg.alpha;
        this.time = msg.time;
        this.stats = msg.stats;
        for (const e of msg.events) this.emitEvent(e);
        this.events.emit('snapshot', { steps: msg.steps, time: msg.time });
        const step = this.stepQueue.shift();
        step?.resolve();
        const waiters = this.snapshotWaiters.splice(0, this.snapshotWaiters.length);
        for (const w of waiters) w();
        this.flushStepQueue();
        break;
      }
      case 'result': {
        const p = this.pending.get(msg.qid);
        if (!p) break;
        this.pending.delete(msg.qid);
        if (msg.error !== undefined) p.reject(new Error(msg.error));
        else p.resolve(msg.value);
        break;
      }
      case 'error':
        if (this.events.hasListeners('error')) this.events.emit('error', { message: msg.message });
        else logError('[physics worker]', msg.message);
        break;
    }
  }

  private readSnapshot(data: Float64Array): void {
    const slots = data.length / SLOT_STRIDE;
    for (const body of this.bodies.values()) {
      if (body.slot >= slots) continue;
      const o = body.slot * SLOT_STRIDE;
      const flags = data[o + 14]!;
      if (!(flags & FLAG_ALIVE)) continue;
      body.position.set(data[o]!, data[o + 1]!, data[o + 2]!);
      body.rotation.set(data[o + 3]!, data[o + 4]!, data[o + 5]!, data[o + 6]!);
      body.previousPosition.set(data[o + 7]!, data[o + 8]!, data[o + 9]!);
      body.previousRotation.set(data[o + 10]!, data[o + 11]!, data[o + 12]!, data[o + 13]!);
      body.isAwake = (flags & FLAG_AWAKE) !== 0;
    }
  }

  private emitEvent(e: EventRecord): void {
    const get = (id: number): BodyProxy | null => this.bodies.get(id) ?? null;
    if (!('sensorBody' in e)) {
      if (!this.events.hasListeners(e.type)) return;
      this.events.emit(e.type, {
        bodyA: get(e.bodyA),
        bodyB: get(e.bodyB),
        colliderA: e.colliderA,
        colliderB: e.colliderB,
        normal: new Vec3(e.normal[0], e.normal[1], e.normal[2]),
        point: new Vec3(e.point[0], e.point[1], e.point[2]),
        approachSpeed: e.approachSpeed,
      });
    } else {
      if (!this.events.hasListeners(e.type)) return;
      this.events.emit(e.type, {
        sensorBody: get(e.sensorBody),
        sensorCollider: e.sensorCollider,
        visitorBody: get(e.visitorBody),
        visitorCollider: e.visitorCollider,
      });
    }
  }
}

function logError(...args: unknown[]): void {
  (globalThis as { console?: { error(...a: unknown[]): void } }).console?.error(...args);
}
