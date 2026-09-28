import { afterEach, describe, expect, it } from 'vitest';
import {
  type BodyDesc,
  type MessagePortLike,
  type PhysicsWorkerOptions,
  ShapeType,
  Vec3,
  World,
  WorkerWorld,
  createBodyFromDesc,
  runPhysicsWorker,
  shapeFromDesc,
} from '../src';

declare const MessageChannel: new () => {
  port1: MessagePortLike & { close(): void };
  port2: MessagePortLike & { close(): void };
};

const DT = 1 / 60;
const closers: (() => void)[] = [];
afterEach(() => {
  for (const c of closers.splice(0)) c();
});

/** 用 MessageChannel 模拟 Worker：port1 为 Worker 端，port2 为主线程端 */
function pair(options?: PhysicsWorkerOptions) {
  const ch = new MessageChannel();
  const host = runPhysicsWorker(options, ch.port1);
  const world = new WorkerWorld(ch.port2);
  closers.push(() => {
    ch.port1.close();
    ch.port2.close();
  });
  return { host, world };
}

const box = (y: number, extra: Partial<BodyDesc> = {}): BodyDesc => ({
  position: [0, y, 0],
  colliders: [{ shape: { type: 'box', halfExtents: [0.5, 0.5, 0.5] }, friction: 0.6 }],
  ...extra,
});
const ground: BodyDesc = { type: 'static', colliders: [{ shape: { type: 'plane' } }] };

describe('描述与构建', () => {
  it('各类形状描述', () => {
    expect(shapeFromDesc({ type: 'sphere', radius: 1 }).type).toBe(ShapeType.Sphere);
    expect(shapeFromDesc({ type: 'capsule', radius: 0.3, halfHeight: 0.5 }).type).toBe(
      ShapeType.Capsule,
    );
    expect(shapeFromDesc({ type: 'cylinder', radius: 1, halfHeight: 1 }).type).toBe(
      ShapeType.Cylinder,
    );
    expect(
      shapeFromDesc({
        type: 'convexHull',
        points: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
      }).type,
    ).toBe(ShapeType.ConvexHull);
    expect(
      shapeFromDesc({ type: 'trimesh', positions: [0, 0, 0, 0, 0, 1, 1, 0, 0], indices: [0, 1, 2] })
        .type,
    ).toBe(ShapeType.TriMesh);
    expect(
      shapeFromDesc({ type: 'heightfield', heights: new Float64Array(4), rows: 2, cols: 2 }).type,
    ).toBe(ShapeType.Heightfield);
    const w = new World();
    const b = createBodyFromDesc(w, {
      position: [1, 2, 3],
      colliders: [
        { shape: { type: 'sphere', radius: 0.5 }, position: [0, 1, 0], restitution: 0.5 },
        { shape: { type: 'box', halfExtents: [1, 1, 1] }, isSensor: true },
      ],
    });
    expect(b.colliders).toHaveLength(2);
    expect(b.colliders[0]!.restitution).toBe(0.5);
    expect(b.colliders[1]!.isSensor).toBe(true);
    expect(b.position.z).toBe(3);
  });
});

describe('WorkerWorld', () => {
  it('锁步推进的结果与本地 World 逐位相同', async () => {
    const { world } = pair();
    world.createBody(ground);
    const proxy = world.createBody(box(3, { rotation: [0.1, 0.2, 0, 0.97467943448] }));
    for (let i = 0; i < 90; i++) await world.step(DT);

    const local = new World();
    createBodyFromDesc(local, ground);
    const body = createBodyFromDesc(local, box(3, { rotation: [0.1, 0.2, 0, 0.97467943448] }));
    for (let i = 0; i < 90; i++) local.step(DT);

    expect(proxy.position.x).toBe(body.position.x);
    expect(proxy.position.y).toBe(body.position.y);
    expect(proxy.rotation.w).toBe(body.rotation.w);
    expect(proxy.position.y).toBeLessThan(0.55);
    expect(world.stats.bodies).toBe(2);
  });

  it('advance 按固定步长推进，请求未返回时累积时间', async () => {
    const { world } = pair();
    world.createBody(ground);
    const proxy = world.createBody(box(5));
    world.advance(0.05);
    expect(world.busy).toBe(true);
    world.advance(0.05); // 累积
    await world.nextSnapshot();
    expect(world.time).toBeCloseTo(0.05, 12);
    world.advance(0);
    await world.nextSnapshot();
    expect(world.time).toBeCloseTo(0.1, 12);
    expect(world.alpha).toBeGreaterThanOrEqual(0);
    expect(world.alpha).toBeLessThan(1);
    expect(proxy.position.y).toBeLessThan(5);
    const p = new Vec3();
    proxy.interpolate(world.alpha, p);
    expect(p.y).toBeLessThanOrEqual(proxy.previousPosition.y);
  });

  it('碰撞事件、射线查询、刚体操作', async () => {
    const { world } = pair();
    const floor = world.createBody(ground);
    const a = world.createBody(box(1.5));
    const hits: string[] = [];
    world.on('collisionStart', (e) => {
      if (e.bodyA === floor || e.bodyB === floor) hits.push('floor');
    });
    for (let i = 0; i < 60; i++) await world.step(DT);
    expect(hits).toContain('floor');

    const hit = await world.raycast(new Vec3(0, 10, 0), new Vec3(0, -1, 0));
    expect(hit!.body).toBe(a);
    expect(hit!.point.y).toBeCloseTo(1, 1);
    const all = await world.raycastAll(new Vec3(0, 10, 0), new Vec3(0, -1, 0));
    expect(all.map((h) => h.body)).toEqual([a, floor]);
    const excluded = await world.raycast(new Vec3(0, 10, 0), new Vec3(0, -1, 0), 100, a);
    expect(excluded!.body).toBe(floor);

    a.applyImpulse(new Vec3(0, 0, 5));
    for (let i = 0; i < 30; i++) await world.step(DT);
    expect(a.position.z).toBeGreaterThan(0.5);
    a.setTransform(new Vec3(3, 4, 0), { x: 0, y: 0, z: 0, w: 1 });
    await world.step(DT);
    expect(a.position.x).toBeCloseTo(3, 4);
  });

  it('关节与销毁刚体（槽位延迟复用）', async () => {
    const { world } = pair();
    const anchor = world.createBody({ type: 'static', position: [0, 5, 0] });
    const bob = world.createBody({
      position: [1, 5, 0],
      colliders: [{ shape: { type: 'sphere', radius: 0.2 } }],
    });
    world.addJoint({
      type: 'distance',
      bodyA: anchor,
      bodyB: bob,
      anchorA: [0, 5, 0],
      anchorB: [1, 5, 0],
    });
    // 摆长 1 m，周期约 2 s：0.5 s 时摆到最低点
    for (let i = 0; i < 30; i++) await world.step(DT);
    expect(bob.position.distanceTo(new Vec3(0, 5, 0))).toBeCloseTo(1, 2);
    expect(bob.position.y).toBeLessThan(4.5);

    const slot = bob.slot;
    world.destroyBody(bob);
    const c = world.createBody(box(8));
    expect(c.slot).not.toBe(slot); // 快照返回前不复用
    await world.step(DT);
    const d = world.createBody(box(9));
    expect(d.slot).toBe(slot);
    await world.step(DT);
    expect(d.position.y).toBeGreaterThan(8.9);
    expect(world.stats.bodies).toBe(3);
  });

  it('大量刚体：快照缓冲区自动扩容', async () => {
    const { world } = pair();
    world.createBody(ground);
    const bodies = [];
    for (let i = 0; i < 300; i++) {
      bodies.push(
        world.createBody({
          position: [(i % 10) * 1.1, 1 + Math.floor(i / 100) * 1.1, Math.floor(i / 10) % 10],
          colliders: [{ shape: { type: 'sphere', radius: 0.5 } }],
        }),
      );
    }
    await world.step(DT);
    expect(bodies[299]!.position.y).toBeGreaterThan(3);
    expect(bodies[299]!.position.y).toBeLessThan(3.3);
    expect(world.stats.bodies).toBe(301);
  });

  it('自定义命令：Worker 内的逻辑与请求 / 应答', async () => {
    const { world } = pair({
      setup(w, host) {
        host.on('count', () => w.bodies.length);
        host.on('gravity', (payload) => {
          w.gravity.set(0, payload as number, 0);
          return w.gravity.y;
        });
        host.on('fail', () => {
          throw new Error('boom');
        });
      },
    });
    world.createBody(box(1));
    expect(await world.request('count')).toBe(1);
    expect(await world.request('gravity', -1.62)).toBe(-1.62);
    await expect(world.request('fail')).rejects.toThrow('boom');
    await expect(world.request('missing')).rejects.toThrow();
  });
});
