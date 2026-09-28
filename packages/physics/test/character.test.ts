import { describe, expect, it } from 'vitest';
import {
  BoxShape,
  CharacterController,
  type CharacterControllerOptions,
  HeightfieldShape,
  PlaneShape,
  Quat,
  TriMeshShape,
  Vec3,
  World,
} from '../src';

const DT = 1 / 60;
/** 默认胶囊：半径 0.3、半高 0.6 → 中心离脚底 0.9 */
const FOOT = 0.9;

function flatWorld(): World {
  const world = new World();
  const ground = world.createBody({ type: 'static' });
  ground.addCollider({ shape: new PlaneShape() });
  return world;
}

function staticBox(world: World, pos: Vec3, half: Vec3, rotation?: Quat) {
  const b = world.createBody({ type: 'static', position: pos, rotation });
  b.addCollider({ shape: new BoxShape(half) });
  return b;
}

/** 模拟游戏里的角色：期望水平速度 + 重力，每步 move 后 world.step */
class Player {
  readonly velocity = new Vec3();
  readonly c: CharacterController;
  private readonly tmp = new Vec3();

  constructor(
    readonly world: World,
    options: CharacterControllerOptions,
  ) {
    this.c = new CharacterController(world, options);
  }

  run(steps: number, walk: Vec3, jump = false): void {
    for (let i = 0; i < steps; i++) this.step(walk, jump && i === 0);
  }

  step(walk: Vec3, jump = false): void {
    const c = this.c;
    if (c.isGrounded) this.velocity.y = jump ? 5 : 0;
    else this.velocity.y -= 9.81 * DT;
    if (c.hitCeiling && this.velocity.y > 0) this.velocity.y = 0;
    this.velocity.x = walk.x;
    this.velocity.z = walk.z;
    c.move(this.tmp.copy(this.velocity).scale(DT), DT);
    this.world.step(DT);
  }
}

describe('CharacterController', () => {
  it('落地后站在地面上，平地行走保持高度', () => {
    const world = flatWorld();
    const p = new Player(world, { position: new Vec3(0, 2, 0) });
    p.run(60, new Vec3());
    expect(p.c.isGrounded).toBe(true);
    expect(p.c.position.y).toBeGreaterThan(FOOT);
    expect(p.c.position.y).toBeLessThan(FOOT + 0.05);
    p.run(60, new Vec3(3, 0, 0));
    expect(p.c.position.x).toBeCloseTo(3, 1);
    expect(p.c.position.y).toBeLessThan(FOOT + 0.05);
    expect(p.c.velocity.x).toBeCloseTo(3, 3);
  });

  it('撞墙停下，斜着撞墙时沿墙滑动；墙角不会穿过', () => {
    const world = flatWorld();
    staticBox(world, new Vec3(2, 1, 0), new Vec3(0.2, 1, 5));
    staticBox(world, new Vec3(0, 1, 3), new Vec3(5, 1, 0.2));
    const p = new Player(world, { position: new Vec3(0, FOOT + 0.02, 0) });
    p.run(120, new Vec3(3, 0, 0));
    expect(p.c.position.x).toBeLessThan(2 - 0.2 - 0.3 + 0.001);
    expect(p.c.position.x).toBeGreaterThan(2 - 0.2 - 0.3 - 0.05);
    expect(p.c.hits.length).toBeGreaterThan(0);
    // 斜向：沿墙滑向墙角并停在墙角
    p.run(120, new Vec3(3, 0, 3));
    expect(p.c.position.z).toBeLessThan(3 - 0.2 - 0.3 + 0.001);
    expect(p.c.position.z).toBeGreaterThan(3 - 0.2 - 0.3 - 0.05);
    expect(p.c.position.x).toBeLessThan(1.5 + 0.001);
  });

  it('能走上缓坡，走不上陡坡', () => {
    for (const [deg, climbs] of [
      [25, true],
      [60, false],
    ] as const) {
      const world = flatWorld();
      const a = (deg * Math.PI) / 180;
      // 从 x = 2 开始向上的斜坡
      const len = 6;
      staticBox(
        world,
        new Vec3(2 + (Math.cos(a) * len) / 2, (Math.sin(a) * len) / 2 - 0.1, 0),
        new Vec3(len / 2, 0.1, 2),
        new Quat().setFromAxisAngle(new Vec3(0, 0, 1), a),
      );
      const p = new Player(world, { position: new Vec3(0, FOOT + 0.02, 0) });
      p.run(150, new Vec3(3, 0, 0));
      if (climbs) {
        expect(p.c.position.y).toBeGreaterThan(FOOT + 1);
        expect(p.c.isGrounded).toBe(true);
      } else {
        expect(p.c.position.y).toBeLessThan(FOOT + 0.3);
      }
    }
  });

  it('自动迈上矮台阶，高台阶挡住', () => {
    for (const [h, climbs] of [
      [0.25, true],
      [0.5, false],
    ] as const) {
      const world = flatWorld();
      staticBox(world, new Vec3(3, h / 2, 0), new Vec3(1, h / 2, 2));
      const p = new Player(world, { position: new Vec3(0, FOOT + 0.02, 0), stepHeight: 0.3 });
      p.run(60, new Vec3(3, 0, 0));
      if (climbs) {
        expect(p.c.position.x).toBeGreaterThan(2.5);
        expect(p.c.position.y).toBeGreaterThan(FOOT + h - 0.01);
        expect(p.c.position.y).toBeLessThan(FOOT + h + 0.05);
        expect(p.c.isGrounded).toBe(true);
      } else {
        expect(p.c.position.x).toBeLessThan(2 - 0.3 + 0.001);
      }
    }
  });

  it('上下楼梯（8 级，每级 0.2 m 高、0.35 m 深）', () => {
    const world = flatWorld();
    for (let i = 0; i < 8; i++) {
      const h = 0.2 * (i + 1);
      staticBox(world, new Vec3(2 + i * 0.35 + 0.175, h / 2, 0), new Vec3(0.175, h / 2, 1.5));
    }
    // 顶部平台
    staticBox(world, new Vec3(2 + 8 * 0.35 + 1.5, 0.8, 0), new Vec3(1.5, 0.8, 1.5));
    const p = new Player(world, { position: new Vec3(0, FOOT + 0.02, 0) });
    // 2.5 m/s × 2.4 s = 6 m
    p.run(144, new Vec3(2.5, 0, 0));
    expect(p.c.position.x).toBeGreaterThan(5.5);
    expect(p.c.position.y).toBeGreaterThan(1.6 + FOOT - 0.01);
    expect(p.c.isGrounded).toBe(true);
    // 走下楼梯全程着地
    let airborne = 0;
    for (let i = 0; i < 144; i++) {
      p.step(new Vec3(-2.5, 0, 0));
      if (!p.c.isGrounded) airborne++;
    }
    expect(p.c.position.x).toBeLessThan(0.5);
    expect(p.c.position.y).toBeLessThan(FOOT + 0.05);
    expect(airborne).toBeLessThan(3);
  });

  it('走下台阶与下坡时贴地（不会腾空）', () => {
    const world = flatWorld();
    staticBox(world, new Vec3(0, 0.1, 0), new Vec3(2, 0.1, 2));
    const p = new Player(world, { position: new Vec3(0, 0.2 + FOOT + 0.02, 0) });
    p.run(10, new Vec3());
    let airborne = 0;
    for (let i = 0; i < 90; i++) {
      p.step(new Vec3(3, 0, 0));
      if (!p.c.isGrounded) airborne++;
    }
    expect(p.c.position.x).toBeGreaterThan(4);
    expect(p.c.position.y).toBeLessThan(FOOT + 0.05);
    expect(airborne).toBe(0);
  });

  it('起跳离地，撞到天花板', () => {
    const world = flatWorld();
    staticBox(world, new Vec3(0, 2.5, 0), new Vec3(2, 0.1, 2));
    const p = new Player(world, { position: new Vec3(0, FOOT + 0.02, 0) });
    p.run(5, new Vec3());
    p.step(new Vec3(), true);
    expect(p.c.isGrounded).toBe(false);
    let ceiling = false;
    let maxY = 0;
    for (let i = 0; i < 60; i++) {
      p.step(new Vec3());
      ceiling ||= p.c.hitCeiling;
      maxY = Math.max(maxY, p.c.position.y);
    }
    expect(ceiling).toBe(true);
    expect(maxY).toBeLessThan(2.4 - FOOT + 0.001);
    expect(p.c.isGrounded).toBe(true);
  });

  it('站在移动的运动学平台上随之移动', () => {
    const world = flatWorld();
    const platform = world.createBody({ type: 'kinematic', position: new Vec3(0, 1, 0) });
    platform.addCollider({ shape: new BoxShape(new Vec3(1.5, 0.1, 1.5)) });
    const p = new Player(world, { position: new Vec3(0, 1.1 + FOOT + 0.02, 0) });
    p.run(20, new Vec3());
    expect(p.c.groundBody).toBe(platform);
    for (let i = 0; i < 60; i++) {
      platform.setLinearVelocity(new Vec3(1, 0, 0));
      p.step(new Vec3());
    }
    expect(platform.position.x).toBeCloseTo(1, 2);
    expect(p.c.position.x).toBeGreaterThan(0.95);
    expect(p.c.isGrounded).toBe(true);
  });

  it('推动动态箱子，动态物体会被角色的运动学刚体挡住', () => {
    const world = flatWorld();
    const crate = world.createBody({ position: new Vec3(1.5, 0.4, 0) });
    crate.addCollider({ shape: new BoxShape(new Vec3(0.4, 0.4, 0.4)), density: 50, friction: 0.3 });
    const p = new Player(world, { position: new Vec3(0, FOOT + 0.02, 0) });
    p.run(90, new Vec3(2, 0, 0));
    expect(crate.position.x).toBeGreaterThan(2.3);
    expect(p.c.position.x).toBeLessThan(crate.position.x - 0.6);

    // 从上方落下的箱子停在角色头顶
    const falling = world.createBody({ position: new Vec3(p.c.position.x, 4, 0) });
    falling.addCollider({ shape: new BoxShape(new Vec3(0.2, 0.2, 0.2)) });
    p.run(60, new Vec3());
    expect(falling.position.y).toBeGreaterThan(p.c.position.y + 0.8);
  });

  it('穿透恢复：出生在箱子里会被推出', () => {
    const world = flatWorld();
    staticBox(world, new Vec3(0.4, 1, 0), new Vec3(0.5, 1, 0.5));
    const p = new Player(world, { position: new Vec3(-0.25, FOOT + 0.02, 0) });
    p.run(3, new Vec3());
    expect(p.c.position.x).toBeLessThan(0.4 - 0.5 - 0.3 + 0.01);
  });

  it('在三角网格与高度场上行走', () => {
    const world = new World();
    const heights = new Float64Array(33 * 33);
    for (let r = 0; r < 33; r++) {
      for (let c = 0; c < 33; c++) heights[r * 33 + c] = 0.3 * Math.sin(c * 0.4) + 0.2 * r * 0.05;
    }
    const field = new HeightfieldShape({ heights, rows: 33, cols: 33, cellSize: 0.5 });
    const ground = world.createBody({ type: 'static' });
    ground.addCollider({ shape: field });
    // 一段三角网格桥（单个倾斜四边形）
    const bridge = world.createBody({ type: 'static', position: new Vec3(0, 0, 5) });
    bridge.addCollider({
      shape: new TriMeshShape([-3, 1, -1, -3, 1, 1, 3, 1.5, 1, 3, 1.5, -1], [0, 1, 2, 0, 2, 3]),
    });
    const p = new Player(world, { position: new Vec3(-6, 2, 0) });
    let airborne = 0;
    p.run(30, new Vec3());
    // 2.5 m/s 走 4 秒 ≈ 10 m
    for (let i = 0; i < 240; i++) {
      p.step(new Vec3(2.5, 0, 0));
      if (!p.c.isGrounded) airborne++;
      const h = field.heightAt(p.c.position.x, p.c.position.z);
      expect(p.c.position.y - h).toBeGreaterThan(FOOT - 0.05);
    }
    expect(p.c.position.x).toBeGreaterThan(3.5);
    expect(airborne).toBeLessThan(5);
    // 走上桥
    p.c.teleport(new Vec3(-2.5, 3, 5));
    p.run(40, new Vec3());
    expect(p.c.groundBody).toBe(bridge);
    p.run(60, new Vec3(2, 0, 0));
    expect(p.c.position.y).toBeGreaterThan(1.2 + FOOT);
    expect(p.c.groundBody).toBe(bridge);
  });
});
