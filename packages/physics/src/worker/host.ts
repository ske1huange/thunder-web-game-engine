import type { Collider } from '../dynamics/Collider';
import type { Joint } from '../dynamics/joints/Joint';
import type { RigidBody } from '../dynamics/RigidBody';
import { Quat } from '../math/Quat';
import { Vec3 } from '../math/Vec3';
import { World } from '../world/World';
import { createBodyFromDesc, jointFromDesc } from './descriptors';
import {
  type EventRecord,
  FLAG_ALIVE,
  FLAG_AWAKE,
  type FromWorker,
  type MessagePortLike,
  type RaycastRecord,
  SLOT_STRIDE,
  type ToWorker,
} from './protocol';

export interface PhysicsWorkerOptions {
  /** 世界创建后调用：可在 Worker 内搭建场景、注册自定义命令（例如角色 / 车辆输入） */
  setup?(world: World, host: PhysicsWorkerHost): void;
}

/** 自定义命令处理函数：返回值（可以是 Promise）会作为 request 的结果传回主线程 */
export type CustomHandler = (payload: unknown, world: World, host: PhysicsWorkerHost) => unknown;

const tmpV = new Vec3();
const tmpV2 = new Vec3();
const tmpQ = new Quat();

const perf = (globalThis as { performance?: { now(): number } }).performance;
function now(): number {
  return perf ? perf.now() : Date.now();
}

/**
 * Worker 端：持有真正的 World，执行主线程发来的命令并回传位姿快照、事件与查询结果。
 *
 * ```ts
 * // physics.worker.ts
 * import { runPhysicsWorker } from '@thunder/physics';
 * runPhysicsWorker();
 * ```
 */
export class PhysicsWorkerHost {
  world: World = new World();
  private readonly bodies = new Map<number, RigidBody>();
  private readonly ids = new Map<RigidBody, number>();
  private readonly slots = new Map<number, number>();
  private readonly joints = new Map<number, Joint>();
  private readonly handlers = new Map<string, CustomHandler>();
  private events: EventRecord[] = [];
  private time = 0;
  private readonly unsubscribe: (() => void)[] = [];

  constructor(
    private readonly port: MessagePortLike,
    private readonly options: PhysicsWorkerOptions = {},
  ) {
    port.onmessage = (e) => this.handle(e.data as ToWorker);
    this.bindEvents();
  }

  /** 注册自定义命令（主线程用 workerWorld.send / request 调用） */
  on(name: string, handler: CustomHandler): void {
    this.handlers.set(name, handler);
  }

  /** 刚体编号 → 刚体 */
  getBody(id: number): RigidBody | undefined {
    return this.bodies.get(id);
  }

  /** 刚体 → 编号（Worker 内创建、未登记的刚体返回 -1） */
  idOf(body: RigidBody): number {
    return this.ids.get(body) ?? -1;
  }

  /**
   * 登记一个在 Worker 内创建的刚体，使其出现在快照中。
   * 编号与槽位由调用方保证不与主线程分配的冲突（建议使用负数编号与较大的槽位）。
   */
  registerBody(body: RigidBody, id: number, slot: number): void {
    this.bodies.set(id, body);
    this.ids.set(body, id);
    this.slots.set(id, slot);
  }

  private bindEvents(): void {
    for (const off of this.unsubscribe) off();
    this.unsubscribe.length = 0;
    const w = this.world;
    const colliderIndex = (c: Collider): number => c.body.colliders.indexOf(c);
    const collision =
      (type: 'collisionStart' | 'collisionEnd') =>
      (e: {
        colliderA: Collider;
        colliderB: Collider;
        normal: Vec3;
        point: Vec3;
        approachSpeed: number;
      }) => {
        this.events.push({
          type,
          bodyA: this.idOf(e.colliderA.body),
          bodyB: this.idOf(e.colliderB.body),
          colliderA: colliderIndex(e.colliderA),
          colliderB: colliderIndex(e.colliderB),
          normal: [e.normal.x, e.normal.y, e.normal.z],
          point: [e.point.x, e.point.y, e.point.z],
          approachSpeed: e.approachSpeed,
        });
      };
    const sensor =
      (type: 'sensorEnter' | 'sensorExit') => (e: { sensor: Collider; visitor: Collider }) => {
        this.events.push({
          type,
          sensorBody: this.idOf(e.sensor.body),
          sensorCollider: colliderIndex(e.sensor),
          visitorBody: this.idOf(e.visitor.body),
          visitorCollider: colliderIndex(e.visitor),
        });
      };
    this.unsubscribe.push(
      w.on('collisionStart', collision('collisionStart')),
      w.on('collisionEnd', collision('collisionEnd')),
      w.on('sensorEnter', sensor('sensorEnter')),
      w.on('sensorExit', sensor('sensorExit')),
    );
  }

  private post(msg: FromWorker, transfer?: unknown[]): void {
    this.port.postMessage(msg, transfer);
  }

  private handle(msg: ToWorker): void {
    try {
      this.dispatch(msg);
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err);
      if ((msg.t === 'query' || msg.t === 'custom') && msg.qid !== undefined) {
        this.post({ t: 'result', qid: msg.qid, value: null, error: message });
      } else {
        this.post({ t: 'error', message });
      }
    }
  }

  private dispatch(msg: ToWorker): void {
    const world = this.world;
    switch (msg.t) {
      case 'init': {
        const { gravity, ...rest } = msg.world;
        this.world = new World({
          ...rest,
          gravity: gravity ? new Vec3(gravity[0], gravity[1], gravity[2]) : undefined,
        });
        this.bodies.clear();
        this.ids.clear();
        this.slots.clear();
        this.joints.clear();
        this.events = [];
        this.time = 0;
        this.bindEvents();
        this.options.setup?.(this.world, this);
        break;
      }
      case 'createBody': {
        const body = createBodyFromDesc(world, msg.desc);
        this.registerBody(body, msg.id, msg.slot);
        break;
      }
      case 'destroyBody': {
        const body = this.bodies.get(msg.id);
        if (!body) break;
        world.destroyBody(body);
        this.bodies.delete(msg.id);
        this.ids.delete(body);
        this.slots.delete(msg.id);
        break;
      }
      case 'addJoint': {
        const a = this.bodies.get(msg.bodyA);
        const b = this.bodies.get(msg.bodyB);
        if (!a || !b) throw new Error('addJoint：刚体不存在');
        this.joints.set(msg.id, world.addJoint(jointFromDesc(msg.desc, a, b)));
        break;
      }
      case 'removeJoint': {
        const j = this.joints.get(msg.id);
        if (j) world.removeJoint(j);
        this.joints.delete(msg.id);
        break;
      }
      case 'body':
        this.bodyOp(msg.id, msg.op, msg.args);
        break;
      case 'setGravity':
        world.gravity.set(msg.gravity[0], msg.gravity[1], msg.gravity[2]);
        for (const b of world.bodies) b.wakeUp();
        break;
      case 'advance':
      case 'step': {
        const before = now();
        const startCount = world.stepCount;
        let alpha = 1;
        if (msg.t === 'advance') {
          alpha = world.advance(msg.dt);
        } else {
          world.step(msg.dt);
        }
        this.time += msg.dt;
        const steps = world.stepCount - startCount;
        const buffer = this.writeSnapshot(msg.buffer);
        const events = this.events;
        this.events = [];
        // stepTime 为本次处理的总耗时（可能包含多步）
        const stats = { ...world.stats, stepTime: now() - before };
        this.post({ t: 'snapshot', buffer, alpha, steps, time: this.time, stats, events }, [
          buffer,
        ]);
        break;
      }
      case 'query': {
        const origin = tmpV.set(msg.origin[0], msg.origin[1], msg.origin[2]);
        const dir = tmpV2.set(msg.direction[0], msg.direction[1], msg.direction[2]);
        const exclude = msg.exclude !== undefined ? this.bodies.get(msg.exclude) : undefined;
        const filter = exclude ? { excludeBody: exclude } : undefined;
        const toRecord = (h: {
          body: RigidBody;
          collider: Collider;
          point: Vec3;
          normal: Vec3;
          distance: number;
        }): RaycastRecord => ({
          body: this.idOf(h.body),
          collider: h.body.colliders.indexOf(h.collider),
          point: [h.point.x, h.point.y, h.point.z],
          normal: [h.normal.x, h.normal.y, h.normal.z],
          distance: h.distance,
        });
        let value: unknown;
        if (msg.kind === 'raycast') {
          const hit = world.raycast(origin, dir, msg.maxDistance, filter);
          value = hit ? toRecord(hit) : null;
        } else {
          value = world.raycastAll(origin, dir, msg.maxDistance, filter).map(toRecord);
        }
        this.post({ t: 'result', qid: msg.qid, value });
        break;
      }
      case 'custom': {
        const handler = this.handlers.get(msg.name);
        if (!handler) throw new Error(`未注册的自定义命令：${msg.name}`);
        const result = handler(msg.payload, world, this);
        if (msg.qid !== undefined) {
          const qid = msg.qid;
          Promise.resolve(result).then(
            (value) => this.post({ t: 'result', qid, value }),
            (err: unknown) =>
              this.post({
                t: 'result',
                qid,
                value: null,
                error: err instanceof Error ? err.message : String(err),
              }),
          );
        }
        break;
      }
    }
  }

  private bodyOp(id: number, op: string, args: readonly unknown[]): void {
    const body = this.bodies.get(id);
    if (!body) return;
    const n = args as readonly number[];
    const vec = (i: number): Vec3 => new Vec3(n[i]!, n[i + 1]!, n[i + 2]!);
    switch (op) {
      case 'setLinearVelocity':
        body.setLinearVelocity(vec(0));
        break;
      case 'setAngularVelocity':
        body.setAngularVelocity(vec(0));
        break;
      case 'applyImpulse':
        body.applyImpulse(vec(0), n.length >= 6 ? vec(3) : undefined);
        break;
      case 'applyForce':
        body.applyForce(vec(0), n.length >= 6 ? vec(3) : undefined);
        break;
      case 'applyTorque':
        body.applyTorque(vec(0));
        break;
      case 'applyAngularImpulse':
        body.applyAngularImpulse(vec(0));
        break;
      case 'setPosition':
        body.setPosition(vec(0));
        break;
      case 'setTransform':
        body.setTransform(vec(0), tmpQ.set(n[3]!, n[4]!, n[5]!, n[6]!));
        break;
      case 'moveKinematic':
        body.moveKinematic(vec(0), tmpQ.set(n[3]!, n[4]!, n[5]!, n[6]!), n[7]!);
        break;
      case 'wakeUp':
        body.wakeUp();
        break;
      case 'sleep':
        body.sleep();
        break;
      case 'setType':
        body.setType(args[0] as never);
        break;
      default:
        throw new Error(`未知的刚体操作：${op}`);
    }
  }

  /** 把所有登记刚体的位姿写入快照缓冲区（容量不足时新建） */
  private writeSnapshot(buffer: ArrayBuffer): ArrayBuffer {
    let maxSlot = -1;
    for (const slot of this.slots.values()) if (slot > maxSlot) maxSlot = slot;
    const needed = (maxSlot + 1) * SLOT_STRIDE * 8;
    const out = buffer.byteLength >= needed ? buffer : new ArrayBuffer(needed);
    const data = new Float64Array(out);
    data.fill(0);
    for (const [id, slot] of this.slots) {
      const body = this.bodies.get(id)!;
      const o = slot * SLOT_STRIDE;
      const p = body.transform.position;
      const r = body.transform.rotation;
      const pp = body.previousPosition;
      const pr = body.previousRotation;
      data[o] = p.x;
      data[o + 1] = p.y;
      data[o + 2] = p.z;
      data[o + 3] = r.x;
      data[o + 4] = r.y;
      data[o + 5] = r.z;
      data[o + 6] = r.w;
      data[o + 7] = pp.x;
      data[o + 8] = pp.y;
      data[o + 9] = pp.z;
      data[o + 10] = pr.x;
      data[o + 11] = pr.y;
      data[o + 12] = pr.z;
      data[o + 13] = pr.w;
      data[o + 14] = FLAG_ALIVE | (body.isAwake ? FLAG_AWAKE : 0);
    }
    return out;
  }
}

/**
 * 在 Worker 中启动物理宿主（默认使用全局 self 作为消息端点）。
 */
export function runPhysicsWorker(
  options: PhysicsWorkerOptions = {},
  port: MessagePortLike = globalThis as unknown as MessagePortLike,
): PhysicsWorkerHost {
  return new PhysicsWorkerHost(port, options);
}
