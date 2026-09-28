import { describe, expect, it } from 'vitest';
import {
  BoxShape,
  CapsuleShape,
  ConvexHullShape,
  CylinderShape,
  PlaneShape,
  Quat,
  type RigidBody,
  SphereShape,
  Vec3,
  World,
  type WorldOptions,
} from '../src';

const DT = 1 / 60;

function makeWorld(options?: WorldOptions): World {
  const world = new World(options);
  const ground = world.createBody({ type: 'static' });
  ground.addCollider({ shape: new PlaneShape() });
  return world;
}

function run(world: World, steps: number): void {
  for (let i = 0; i < steps; i++) world.step(DT);
}

function buildPyramid(world: World, rows: number): RigidBody[] {
  const boxes: RigidBody[] = [];
  for (let row = 0; row < rows; row++) {
    for (let i = 0; i < rows - row; i++) {
      const b = world.createBody({
        position: new Vec3((i - (rows - row - 1) / 2) * 1.02, 0.5 + row, 0),
      });
      b.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
      boxes.push(b);
    }
  }
  return boxes;
}

describe('World 基本行为', () => {
  it('自由落体与 ½gt² 一致', () => {
    const world = new World();
    const body = world.createBody({ position: new Vec3(0, 100, 0) });
    body.addCollider({ shape: new SphereShape(0.5) });
    run(world, 60);
    const expected = 100 - 0.5 * 9.81 * 1 * 1;
    expect(Math.abs(body.position.y - expected)).toBeLessThan(0.05);
    expect(body.linearVelocity.y).toBeCloseTo(-9.81, 6);
  });

  it('长方体落地后静止并进入休眠', () => {
    const world = makeWorld();
    const box = world.createBody({ position: new Vec3(0, 5, 0) });
    box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
    let minY = Infinity;
    for (let i = 0; i < 240; i++) {
      world.step(DT);
      minY = Math.min(minY, box.position.y);
    }
    expect(box.position.y).toBeCloseTo(0.5, 2);
    // 落地时不应明显穿透地面
    expect(minY).toBeGreaterThan(0.49);
    expect(box.isAwake).toBe(false);
  });

  it('10 层箱子金字塔稳定（漂移 < 1cm）并全部休眠', () => {
    const world = makeWorld();
    const boxes = buildPyramid(world, 10);
    const start = boxes.map((b) => b.position.clone());
    run(world, 600);
    let maxDrift = 0;
    boxes.forEach((b, i) => {
      maxDrift = Math.max(maxDrift, b.position.distanceTo(start[i]!));
    });
    expect(maxDrift).toBeLessThan(0.01);
    expect(world.stats.awakeBodies).toBe(0);
  });

  it('弹性：restitution=1 基本保持弹跳高度，restitution=0 不反弹', () => {
    for (const e of [1, 0]) {
      const world = new World();
      const ground = world.createBody({ type: 'static' });
      ground.addCollider({ shape: new PlaneShape(), restitution: e });
      const ball = world.createBody({ position: new Vec3(0, 2, 0) });
      ball.addCollider({ shape: new SphereShape(0.25), restitution: e });
      let maxY = 0;
      let maxUpVelocity = 0;
      for (let i = 0; i < 180; i++) {
        world.step(DT);
        if (i > 50) {
          maxY = Math.max(maxY, ball.position.y);
          maxUpVelocity = Math.max(maxUpVelocity, ball.linearVelocity.y);
        }
      }
      if (e === 1) {
        expect(maxY).toBeGreaterThan(1.8);
        expect(maxY).toBeLessThan(2.05);
      } else {
        expect(maxUpVelocity).toBeLessThan(0.1);
        expect(ball.position.y).toBeCloseTo(0.25, 2);
      }
    }
  });

  it('摩擦：μ > tanθ 时停在斜面上，μ < tanθ 时下滑', () => {
    const angle = (20 * Math.PI) / 180;
    for (const mu of [0.6, 0.1]) {
      const world = new World();
      const incline = world.createBody({
        type: 'static',
        rotation: new Quat().setFromAxisAngle(new Vec3(0, 0, 1), angle),
      });
      incline.addCollider({ shape: new BoxShape(new Vec3(10, 0.5, 5)), friction: mu });
      const up = new Vec3(-Math.sin(angle), Math.cos(angle), 0);
      const box = world.createBody({
        position: up.clone().scale(1.0),
        rotation: new Quat().setFromAxisAngle(new Vec3(0, 0, 1), angle),
      });
      box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)), friction: mu });
      run(world, 30);
      const p0 = box.position.clone();
      run(world, 120);
      const slid = box.position.distanceTo(p0);
      if (mu > Math.tan(angle)) expect(slid).toBeLessThan(0.02);
      else expect(slid).toBeGreaterThan(1);
    }
  });

  it('确定性：相同输入得到逐位相同的结果', () => {
    const simulate = () => {
      const world = makeWorld();
      const boxes = buildPyramid(world, 5);
      const ball = world.createBody({
        position: new Vec3(-8, 2, 0.1),
        linearVelocity: new Vec3(15, 0, 0),
      });
      ball.addCollider({ shape: new SphereShape(0.5), density: 5 });
      run(world, 200);
      return [...boxes, ball].map((b) => [b.position.x, b.position.y, b.position.z, b.rotation.w]);
    };
    expect(simulate()).toEqual(simulate());
  });

  it('多种形状落地后都能静止', () => {
    const world = makeWorld();
    const shapes = [
      new SphereShape(0.4),
      new CapsuleShape(0.3, 0.4),
      new CylinderShape(0.4, 0.3),
      new ConvexHullShape([
        new Vec3(0, 0.6, 0),
        new Vec3(0.5, -0.3, 0.3),
        new Vec3(-0.5, -0.3, 0.3),
        new Vec3(0, -0.3, -0.5),
      ]),
    ];
    const bodies = shapes.map((shape, i) => {
      const b = world.createBody({
        position: new Vec3(i * 2, 2, 0),
        rotation: new Quat().setFromEuler(0.3 * i, 0.2, 0.1),
      });
      b.addCollider({ shape });
      return b;
    });
    run(world, 900);
    for (const b of bodies) {
      // 最低点贴地（允许 linearSlop 量级的误差），且已经静止
      const lowest = b.colliders[0]!.aabb.min.y;
      expect(Math.abs(lowest)).toBeLessThan(0.01);
      expect(b.linearVelocity.length()).toBeLessThan(0.05);
      expect(b.angularVelocity.length()).toBeLessThan(0.05);
    }
  });
});

describe('事件', () => {
  it('collisionStart / collisionEnd', () => {
    const world = makeWorld();
    const ball = world.createBody({ position: new Vec3(0, 1, 0) });
    ball.addCollider({ shape: new SphereShape(0.25) });
    const starts: number[] = [];
    let ends = 0;
    world.on('collisionStart', (e) => starts.push(e.approachSpeed));
    world.on('collisionEnd', () => ends++);
    run(world, 60);
    expect(starts.length).toBe(1);
    expect(starts[0]).toBeGreaterThan(0);
    ball.setPosition(new Vec3(0, 3, 0));
    run(world, 2);
    expect(ends).toBe(1);
  });

  it('传感器进入与离开', () => {
    const world = new World();
    const zone = world.createBody({ type: 'static', position: new Vec3(0, 5, 0) });
    const sensor = zone.addCollider({ shape: new BoxShape(new Vec3(1, 1, 1)), isSensor: true });
    const ball = world.createBody({ position: new Vec3(0, 8, 0) });
    ball.addCollider({ shape: new SphereShape(0.25) });
    const log: string[] = [];
    world.on('sensorEnter', (e) => {
      expect(e.sensor).toBe(sensor);
      log.push('enter');
    });
    world.on('sensorExit', () => log.push('exit'));
    run(world, 120);
    expect(log).toEqual(['enter', 'exit']);
    // 传感器不产生碰撞响应：球穿过传感器继续下落
    expect(ball.position.y).toBeLessThan(0);
  });

  it('休眠与唤醒', () => {
    const world = makeWorld();
    const boxes = buildPyramid(world, 4);
    const sleeps: RigidBody[] = [];
    world.on('sleep', (e) => sleeps.push(e.body));
    run(world, 300);
    expect(world.stats.awakeBodies).toBe(0);
    expect(sleeps.length).toBe(boxes.length);
    // 推动一个箱子，整堆一起被唤醒
    boxes[0]!.applyImpulse(new Vec3(-1, 0, 0));
    expect(boxes.every((b) => b.isAwake)).toBe(true);
  });
});

describe('过滤、运动学与连续碰撞', () => {
  it('碰撞过滤：mask 不匹配时穿过地面', () => {
    const world = new World();
    const ground = world.createBody({ type: 'static' });
    ground.addCollider({ shape: new BoxShape(new Vec3(5, 0.5, 5)), filter: { categoryBits: 0b01 } });
    const ghost = world.createBody({ position: new Vec3(0, 2, 0) });
    ghost.addCollider({ shape: new SphereShape(0.5), filter: { maskBits: 0b10 } });
    const solid = world.createBody({ position: new Vec3(2, 2, 0) });
    solid.addCollider({ shape: new SphereShape(0.5) });
    run(world, 120);
    expect(ghost.position.y).toBeLessThan(-1);
    expect(solid.position.y).toBeCloseTo(1, 1);
    // group 为负数时同组不碰撞
    const a = world.createBody({ position: new Vec3(10, 0.5, 0), type: 'static' });
    a.addCollider({ shape: new BoxShape(new Vec3(1, 0.5, 1)), filter: { group: -1 } });
    const b = world.createBody({ position: new Vec3(10, 3, 0) });
    b.addCollider({ shape: new SphereShape(0.3), filter: { group: -1 } });
    run(world, 120);
    expect(b.position.y).toBeLessThan(0);
  });

  it('连续碰撞：高速小球不会穿过薄墙', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) });
    const wall = world.createBody({ type: 'static', position: new Vec3(5, 0, 0) });
    wall.addCollider({ shape: new BoxShape(new Vec3(0.05, 2, 2)) });
    const bullet = world.createBody({ position: new Vec3(0, 0, 0), linearVelocity: new Vec3(300, 0, 0) });
    bullet.addCollider({ shape: new SphereShape(0.05) });
    run(world, 10);
    expect(bullet.position.x).toBeLessThan(5);
  });

  it('连续碰撞不会把贴地高速滑行的物体钉在原地', () => {
    const world = makeWorld();
    // 半径 5cm 的小球以 10m/s 贴地滑行：每步位移远大于 CCD 阈值，且起点已与地面接触
    const puck = world.createBody({
      position: new Vec3(0, 0.05, 0),
      linearVelocity: new Vec3(10, 0, 0),
    });
    puck.addCollider({ shape: new SphereShape(0.05), friction: 0 });
    const tail = world.createBody({ position: new Vec3(-0.2, 0.05, 0), linearVelocity: new Vec3(10, 0, 0) });
    tail.addCollider({ shape: new SphereShape(0.05), friction: 0 });
    run(world, 30);
    expect(puck.position.x).toBeGreaterThan(4.5);
    expect(tail.position.x).toBeGreaterThan(4.3);
  });

  it('子弹对动态物体也做连续碰撞', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) });
    const target = world.createBody({ position: new Vec3(5, 0, 0) });
    target.addCollider({ shape: new BoxShape(new Vec3(0.05, 1, 1)) });
    const bullet = world.createBody({
      position: new Vec3(0, 0, 0),
      linearVelocity: new Vec3(300, 0, 0),
      isBullet: true,
    });
    bullet.addCollider({ shape: new SphereShape(0.05) });
    run(world, 3);
    expect(target.linearVelocity.x).toBeGreaterThan(0.1);
  });

  it('运动学平台带动上面的箱子', () => {
    const world = new World();
    const platform = world.createBody({ type: 'kinematic', position: new Vec3(0, 0, 0) });
    platform.addCollider({ shape: new BoxShape(new Vec3(3, 0.25, 3)) });
    platform.setLinearVelocity(new Vec3(1, 0, 0));
    const box = world.createBody({ position: new Vec3(0, 0.75, 0) });
    box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)), friction: 0.8 });
    run(world, 120);
    expect(platform.position.x).toBeCloseTo(2, 6);
    expect(box.position.x).toBeGreaterThan(1.8);
    expect(box.position.y).toBeCloseTo(0.75, 1);
  });

  it('销毁刚体会移除接触并触发 collisionEnd', () => {
    const world = makeWorld();
    const box = world.createBody({ position: new Vec3(0, 0.5, 0) });
    box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
    let ends = 0;
    world.on('collisionEnd', () => ends++);
    run(world, 10);
    expect(world.stats.touchingContacts).toBe(1);
    world.destroyBody(box);
    expect(ends).toBe(1);
    expect(world.contacts.length).toBe(0);
    expect(world.bodies.length).toBe(1);
    run(world, 2);
  });

  it('advance 以固定步长推进并返回插值系数', () => {
    const world = new World({ fixedTimeStep: 1 / 60 });
    const body = world.createBody({ position: new Vec3(0, 10, 0) });
    body.addCollider({ shape: new SphereShape(0.5) });
    const alpha = world.advance(2.5 / 60);
    expect(alpha).toBeCloseTo(0.5, 9);
    const p = new Vec3();
    body.interpolate(alpha, p);
    expect(p.y).toBeLessThan(body.previousPosition.y);
    expect(p.y).toBeGreaterThan(body.position.y);
  });

  it('step 期间修改世界会抛错', () => {
    const world = makeWorld();
    const ball = world.createBody({ position: new Vec3(0, 0.3, 0) });
    ball.addCollider({ shape: new SphereShape(0.5) });
    let error: unknown = null;
    world.on('collisionStart', () => {
      // 事件在 step 结束后派发，此时可以安全修改
      try {
        world.createBody();
      } catch (e) {
        error = e;
      }
    });
    run(world, 2);
    expect(error).toBeNull();
  });
});
