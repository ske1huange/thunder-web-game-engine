import { describe, expect, it } from 'vitest';
import {
  AABB,
  BallSocketJoint,
  BoxShape,
  CapsuleShape,
  type DebugDrawer,
  PlaneShape,
  Quat,
  SphereShape,
  Transform,
  Vec3,
  World,
} from '../src';

function scene() {
  const world = new World();
  const ground = world.createBody({ type: 'static' });
  ground.addCollider({ shape: new PlaneShape() });
  const box = world.createBody({ position: new Vec3(0, 0.5, 0) });
  box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
  const ball = world.createBody({ position: new Vec3(3, 0.5, 0) });
  ball.addCollider({ shape: new SphereShape(0.5), filter: { categoryBits: 0b10 } });
  const zone = world.createBody({ type: 'static', position: new Vec3(-3, 1, 0) });
  zone.addCollider({ shape: new BoxShape(new Vec3(1, 1, 1)), isSensor: true });
  return { world, ground, box, ball, zone };
}

describe('射线检测', () => {
  it('返回最近命中，含点、法线与距离', () => {
    const { world, box } = scene();
    const hit = world.raycast(new Vec3(0, 10, 0), new Vec3(0, -2, 0), 100);
    expect(hit).not.toBeNull();
    expect(hit!.body).toBe(box);
    expect(hit!.distance).toBeCloseTo(9, 9);
    expect(hit!.point.equals(new Vec3(0, 1, 0), 1e-9)).toBe(true);
    expect(hit!.normal.equals(new Vec3(0, 1, 0), 1e-9)).toBe(true);
  });

  it('maxDistance 截断、raycastAll 排序', () => {
    const { world } = scene();
    expect(world.raycast(new Vec3(0, 10, 0), new Vec3(0, -1, 0), 5)).toBeNull();
    const hits = world.raycastAll(new Vec3(-10, 0.5, 0), new Vec3(1, 0, 0), 100);
    expect(hits.map((h) => h.distance.toFixed(3))).toEqual(['9.500', '12.500']);
  });

  it('过滤：传感器默认忽略、类别掩码、排除刚体', () => {
    const { world, ball, zone, box } = scene();
    const origin = new Vec3(-3, 5, 0);
    const down = new Vec3(0, -1, 0);
    expect(world.raycast(origin, down)!.body.isStatic()).toBe(true);
    expect(world.raycast(origin, down)!.collider.isSensor).toBe(false);
    expect(world.raycast(origin, down, 100, { includeSensors: true })!.body).toBe(zone);
    const fromLeft = new Vec3(-10, 0.5, 0);
    const right = new Vec3(1, 0, 0);
    expect(world.raycast(fromLeft, right, 100, { maskBits: 0b10 })!.body).toBe(ball);
    expect(world.raycast(fromLeft, right, 100, { excludeBody: box })!.body).toBe(ball);
  });

  it('旋转的胶囊体', () => {
    const world = new World();
    const cap = world.createBody({
      type: 'static',
      rotation: new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2),
    });
    cap.addCollider({ shape: new CapsuleShape(0.5, 2) });
    const hit = world.raycast(new Vec3(2, 5, 0), new Vec3(0, -1, 0));
    expect(hit!.distance).toBeCloseTo(4.5, 9);
    const end = world.raycast(new Vec3(10, 0, 0), new Vec3(-1, 0, 0));
    expect(end!.distance).toBeCloseTo(7.5, 9);
  });
});

describe('区域与形状查询', () => {
  it('queryAABB', () => {
    const { world, box, ball } = scene();
    const found = world.queryAABB(new AABB(new Vec3(-1, 0.2, -1), new Vec3(4, 1, 1)));
    expect(found.map((c) => c.body)).toEqual(expect.arrayContaining([box, ball]));
    expect(found.some((c) => c.isSensor)).toBe(false);
  });

  it('overlapShape', () => {
    const { world, box } = scene();
    const probe = new SphereShape(0.3);
    const hits = world.overlapShape(probe, new Transform(new Vec3(0.7, 0.5, 0)));
    expect(hits.map((c) => c.body)).toEqual([box]);
    expect(world.overlapShape(probe, new Transform(new Vec3(1.5, 3, 0)))).toEqual([]);
  });

  it('castShape 找到最先命中的物体', () => {
    const { world, box } = scene();
    const probe = new SphereShape(0.25);
    const hit = world.castShape(probe, new Transform(new Vec3(-2, 0.5, 0)), new Vec3(4, 0, 0));
    expect(hit).not.toBeNull();
    expect(hit!.body).toBe(box);
    // 球心停在 x ≈ -0.5 - 0.25 处：位移 ≈ 1.25，比例 ≈ 0.3125
    expect(hit!.fraction).toBeCloseTo(1.25 / 4, 2);
    expect(hit!.normal.x).toBeCloseTo(1, 6);
    // 向下投射命中地面平面
    const down = world.castShape(probe, new Transform(new Vec3(6, 3, 0)), new Vec3(0, -5, 0));
    expect(down!.body.isStatic()).toBe(true);
    expect(down!.fraction).toBeCloseTo(2.75 / 5, 2);
  });
});

describe('调试绘制', () => {
  it('输出形状、接触与关节线段', () => {
    const { world, box, ball } = scene();
    world.addJoint(new BallSocketJoint({ bodyA: box, bodyB: ball, anchor: new Vec3(1.5, 0.5, 0) }));
    world.step(1 / 60);
    let lines = 0;
    let points = 0;
    const colors = new Set<number>();
    const drawer: DebugDrawer = {
      drawLine(from, to, color) {
        expect(from.isFinite() && to.isFinite()).toBe(true);
        colors.add(color);
        lines++;
      },
      drawPoint() {
        points++;
      },
    };
    world.debugDraw(drawer, { contacts: true, aabbs: true, centerOfMass: true });
    expect(lines).toBeGreaterThan(50);
    expect(points).toBeGreaterThan(0);
    expect(colors.size).toBeGreaterThan(4);
  });
});
