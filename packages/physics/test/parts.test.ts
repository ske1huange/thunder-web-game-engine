import { describe, expect, it } from 'vitest';
import { BoxShape, type RigidBody, Vec3, VehicleParts, World } from '../src';

const DT = 1 / 60;

function run(world: World, steps: number, each?: (i: number) => void): void {
  for (let i = 0; i < steps; i++) {
    each?.(i);
    world.step(DT);
  }
}

/**
 * 车身（type 可选）+ 左侧车门：车门在 x = -1 处，从铰链（前沿 z = -1）向后延伸到 z = 0，
 * 绕竖直轴向外（-X 方向）打开。车头朝 -Z。
 */
function carWithDoor(chassisType: 'static' | 'kinematic' = 'static') {
  const world = new World({ enableSleep: false });
  const chassis = world.createBody({ type: chassisType, position: new Vec3(0, 1, 0) });
  chassis.addCollider({
    shape: new BoxShape(new Vec3(0.9, 0.5, 2)),
    filter: { group: -1 },
  });
  const door = world.createBody({ position: new Vec3(-1, 1, -0.5) });
  door.addCollider({
    shape: new BoxShape(new Vec3(0.04, 0.45, 0.5)),
    density: 250,
    filter: { group: -1 },
  });
  const parts = new VehicleParts(world);
  // 绕 +Y 轴旋转负角度时，车门后端（+Z 方向）转向 -X（向外）
  const part = parts.addHinged({
    name: 'door',
    chassis,
    body: door,
    anchor: new Vec3(-1, 1, -1),
    axis: new Vec3(0, 1, 0),
    openAngle: (-70 * Math.PI) / 180,
  });
  return { world, chassis, door, parts, part };
}

describe('HingedPart', () => {
  it('关闭时锁止，打开由马达推动到位，关闭后重新锁止', () => {
    const { world, door, part } = carWithDoor();
    door.applyAngularImpulse(new Vec3(0, 20, 0));
    run(world, 30);
    expect(Math.abs(part.angle)).toBeLessThan(0.01);
    expect(part.state).toBe('closed');

    part.open();
    expect(part.state).toBe('opening');
    run(world, 90);
    expect(part.state).toBe('open');
    // 自由模式：撞到打开限位后会略微回弹
    expect(part.angle).toBeLessThan(-0.9);
    expect(part.angle).toBeGreaterThan(part.openAngle - 0.05);
    // 车门后端转到了车身外侧
    const rear = door.getWorldPoint(new Vec3(0, 0, 0.5), new Vec3());
    expect(rear.x).toBeLessThan(-1.5);

    part.close();
    run(world, 120);
    expect(part.state).toBe('closed');
    expect(Math.abs(part.angle)).toBeLessThan(0.02);
    door.applyAngularImpulse(new Vec3(0, 20, 0));
    run(world, 30);
    expect(Math.abs(part.angle)).toBeLessThan(0.02);
  });

  it('打开时被障碍物挡住', () => {
    const { world, part } = carWithDoor();
    const wall = world.createBody({ type: 'static', position: new Vec3(-1.6, 1, 0) });
    wall.addCollider({ shape: new BoxShape(new Vec3(0.1, 1, 1)) });
    part.open();
    run(world, 120);
    expect(part.state).toBe('opening');
    expect(part.angle).toBeGreaterThan(-0.9);
    expect(part.angle).toBeLessThan(-0.3);
  });

  it('自由摆动：车身加速时打开的车门被甩上并锁止', () => {
    const { world, chassis, part } = carWithDoor('kinematic');
    part.open();
    run(world, 90);
    expect(part.state).toBe('open');
    // 车身向前（-Z）加速
    let v = 0;
    run(world, 120, () => {
      v += 8 * DT;
      chassis.setLinearVelocity(new Vec3(0, 0, -v));
    });
    expect(part.state).toBe('closed');
    expect(Math.abs(part.angle)).toBeLessThan(0.03);
  });

  it('保持打开：引擎盖克服重力被顶在打开位置', () => {
    const world = new World({ enableSleep: false });
    const chassis = world.createBody({ type: 'static', position: new Vec3(0, 1, 0) });
    chassis.addCollider({ shape: new BoxShape(new Vec3(0.9, 0.5, 2)), filter: { group: -1 } });
    // 引擎盖在车头上方（-Z 侧），铰链在靠近挡风玻璃的后沿 z = -0.8
    const hood = world.createBody({ position: new Vec3(0, 1.54, -1.4) });
    hood.addCollider({
      shape: new BoxShape(new Vec3(0.8, 0.03, 0.6)),
      density: 150,
      filter: { group: -1 },
    });
    const parts = new VehicleParts(world);
    // 绕 +X 旋转正角度时，车头方向（-Z）的前沿抬起
    const part = parts.addHinged({
      name: 'hood',
      chassis,
      body: hood,
      anchor: new Vec3(0, 1.54, -0.8),
      axis: new Vec3(1, 0, 0),
      openAngle: 1,
      holdOpen: 'hold',
    });
    part.open();
    run(world, 120);
    expect(part.state).toBe('open');
    run(world, 120);
    expect(part.angle).toBeCloseTo(1, 1);
    const front = hood.getWorldPoint(new Vec3(0, 0, -0.6), new Vec3());
    expect(front.y).toBeGreaterThan(2);
    // 自由模式下重力会让它落回关闭位置并锁上
    part.holdOpen = 'free';
    part.close();
    part.open();
    run(world, 60);
    part.close();
    run(world, 180);
    expect(part.state).toBe('closed');
  });
});

describe('SlidingPart 与 WiperPart', () => {
  function base(): { world: World; chassis: RigidBody; parts: VehicleParts } {
    const world = new World({ enableSleep: false });
    const chassis = world.createBody({ type: 'static', position: new Vec3(0, 1, 0) });
    chassis.addCollider({ shape: new BoxShape(new Vec3(0.9, 0.5, 2)), filter: { group: -1 } });
    return { world, chassis, parts: new VehicleParts(world) };
  }

  it('天窗沿车顶向后滑开再关上', () => {
    const { world, chassis, parts } = base();
    const roof = world.createBody({ position: new Vec3(0, 1.52, 0) });
    roof.addCollider({
      shape: new BoxShape(new Vec3(0.4, 0.01, 0.3)),
      density: 500,
      filter: { group: -1 },
    });
    const sunroof = parts.addSliding({
      name: 'sunroof',
      chassis,
      body: roof,
      anchor: new Vec3(0, 1.52, 0),
      axis: new Vec3(0, 0, 1),
      travel: 0.45,
    });
    run(world, 30);
    expect(Math.abs(sunroof.position)).toBeLessThan(0.005);
    sunroof.open();
    run(world, 150);
    expect(sunroof.state).toBe('open');
    expect(roof.position.z).toBeCloseTo(0.45, 2);
    sunroof.close();
    run(world, 150);
    expect(sunroof.state).toBe('closed');
    expect(Math.abs(roof.position.z)).toBeLessThan(0.01);
  });

  it('雨刮在摆动范围内往复，停止后回到停放位置', () => {
    const { world, chassis, parts } = base();
    const blade = world.createBody({ position: new Vec3(0.3, 1.55, -1.2) });
    blade.addCollider({
      shape: new BoxShape(new Vec3(0.3, 0.01, 0.01)),
      density: 800,
      filter: { group: -1 },
    });
    const wiper = parts.addWiper({
      name: 'wiper',
      chassis,
      body: blade,
      anchor: new Vec3(0, 1.55, -1.2),
      axis: new Vec3(0, 0, 1),
      sweep: 1.4,
    });
    wiper.start();
    const angles: number[] = [];
    run(world, 180, () => angles.push(wiper.angle));
    const max = Math.max(...angles);
    const min = Math.min(...angles);
    expect(max).toBeGreaterThan(1.3);
    expect(max).toBeLessThan(1.45);
    // 至少完成了两次往返
    let reversals = 0;
    for (let i = 2; i < angles.length; i++) {
      const d1 = angles[i - 1]! - angles[i - 2]!;
      const d2 = angles[i]! - angles[i - 1]!;
      if (d1 * d2 < 0 && Math.abs(d1) > 1e-3) reversals++;
    }
    expect(reversals).toBeGreaterThanOrEqual(3);
    expect(min).toBeGreaterThan(-0.05);
    wiper.stop();
    run(world, 120);
    expect(Math.abs(wiper.angle)).toBeLessThan(0.05);
  });
});

describe('RigidBody 质量覆盖与质心偏移', () => {
  it('mass 缩放质量与惯性，centerOfMassOffset 降低质心', () => {
    const world = new World();
    const a = world.createBody();
    a.addCollider({ shape: new BoxShape(new Vec3(1, 0.5, 2)) });
    const inertia = a.inertiaLocal.m00;
    a.setMass(1200);
    expect(a.mass).toBeCloseTo(1200, 9);
    expect(a.inertiaLocal.m00).toBeCloseTo((inertia * 1200) / 8, 6);
    a.setCenterOfMassOffset(new Vec3(0, -0.4, 0));
    expect(a.localCenter.y).toBeCloseTo(-0.4, 9);
    const b = world.createBody({ mass: 50, centerOfMassOffset: new Vec3(0, -0.2, 0) });
    b.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
    expect(b.mass).toBeCloseTo(50, 9);
    expect(b.center.y).toBeCloseTo(-0.2, 9);
  });
});
