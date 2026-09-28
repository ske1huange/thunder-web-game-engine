import type { WorldOptions, WorldStats } from '../world/World';
import type { BodyDesc, JointDesc, Vec3Tuple } from './descriptors';

/**
 * 主线程与物理 Worker 之间的消息协议。
 *
 * 位姿快照是一块 Float64Array：每个刚体槽位 SLOT_STRIDE 个数——
 * 位置 (3)、朝向 (4)、上一步位置 (3)、上一步朝向 (4)、标志 (1)。
 * 缓冲区在两端之间来回转移（transfer），不复制、不分配。
 */
export const SLOT_STRIDE = 15;
export const FLAG_ALIVE = 1;
export const FLAG_AWAKE = 2;

/** 消息回调（写成方法签名的双变形式，使 DOM 的 Worker / MessagePort 可直接赋值） */
export type MessageHandler = {
  bivarianceHack(event: { data: unknown }): void;
}['bivarianceHack'];

/** 可以 postMessage 的端点：Worker、Worker 内的 self、MessagePort 都满足 */
export interface MessagePortLike {
  postMessage(message: unknown, transfer?: unknown[]): void;
  onmessage: MessageHandler | null;
}

/** World 构造参数（gravity 写成数组） */
export type WorldDesc = Omit<WorldOptions, 'gravity'> & { gravity?: Vec3Tuple };

/** 刚体上的操作：方法名与参数（向量展开为数字） */
export type BodyOp =
  | 'setLinearVelocity'
  | 'setAngularVelocity'
  | 'applyImpulse'
  | 'applyForce'
  | 'applyTorque'
  | 'applyAngularImpulse'
  | 'setPosition'
  | 'setTransform'
  | 'moveKinematic'
  | 'wakeUp'
  | 'sleep'
  | 'setType';

export type QueryKind = 'raycast' | 'raycastAll';

export type ToWorker =
  | { t: 'init'; world: WorldDesc }
  | { t: 'createBody'; id: number; slot: number; desc: BodyDesc }
  | { t: 'destroyBody'; id: number }
  | { t: 'addJoint'; id: number; bodyA: number; bodyB: number; desc: JointDesc }
  | { t: 'removeJoint'; id: number }
  | { t: 'body'; id: number; op: BodyOp; args: readonly unknown[] }
  | { t: 'setGravity'; gravity: Vec3Tuple }
  | { t: 'advance'; dt: number; buffer: ArrayBuffer }
  | { t: 'step'; dt: number; buffer: ArrayBuffer }
  | {
      t: 'query';
      qid: number;
      kind: QueryKind;
      origin: Vec3Tuple;
      direction: Vec3Tuple;
      maxDistance: number;
      exclude?: number;
    }
  | { t: 'custom'; name: string; payload: unknown; qid?: number };

export interface CollisionRecord {
  type: 'collisionStart' | 'collisionEnd';
  bodyA: number;
  bodyB: number;
  colliderA: number;
  colliderB: number;
  normal: Vec3Tuple;
  point: Vec3Tuple;
  approachSpeed: number;
}

export interface SensorRecord {
  type: 'sensorEnter' | 'sensorExit';
  sensorBody: number;
  sensorCollider: number;
  visitorBody: number;
  visitorCollider: number;
}

export type EventRecord = CollisionRecord | SensorRecord;

export interface RaycastRecord {
  body: number;
  collider: number;
  point: Vec3Tuple;
  normal: Vec3Tuple;
  distance: number;
}

export type FromWorker =
  | {
      t: 'snapshot';
      buffer: ArrayBuffer;
      alpha: number;
      steps: number;
      time: number;
      stats: WorldStats;
      events: EventRecord[];
    }
  | { t: 'result'; qid: number; value: unknown; error?: string }
  | { t: 'error'; message: string };
