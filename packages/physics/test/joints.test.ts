import { describe, expect, it } from 'vitest';
import {
  BallSocketJoint,
  BoxShape,
  DistanceJoint,
  FixedJoint,
  HingeJoint,
  MouseJoint,
  PlaneShape,
  type RigidBody,
  SliderJoint,
  SphereShape,
  Vec3,
  World,
} from '../src';

const DT = 1 / 60;

function run(world: World, steps: number, each?: () => void): void {
  for (let i = 0; i < steps; i++) {
    world.step(DT);
    each?.();
  }
}

function sphere(world: World, pos: Vec3, r = 0.1): RigidBody {
  const b = world.createBody({ position: pos });
  b.addCollider({ shape: new SphereShape(r) });
  return b;
}

function box(world: World, pos: Vec3, half: Vec3, type: 'dynamic' | 'static' = 'dynamic'): RigidBody {
  const b = world.createBody({ position: pos, type });
  b.addCollider({ shape: new BoxShape(half) });
  return b;
}

describe('BallSocketJoint', () => {
  it('单摆：锚点保持重合，能量基本守恒', () => {
    const world = new World({ enableSleep: false });
    const pivot = world.createBody({ type: 'static', position: new Vec3(0, 5, 0) });
    const bob = sphere(world, new Vec3(2, 5, 0));
    const joint = world.addJoint(
      new BallSocketJoint({ bodyA: pivot, bodyB: bob, anchor: new Vec3(0, 5, 0) }),
    );
    let maxError = 0;
    let lowest = Infinity;
    const a = new Vec3();
    const b = new Vec3();
    run(world, 300, () => {
      maxError = Math.max(maxError, joint.getAnchorA(a).distanceTo(joint.getAnchorB(b)));
      lowest = Math.min(lowest, bob.position.y);
    });
    expect(maxError).toBeLessThan(0.02);
    expect(lowest).toBeCloseTo(3, 1);
    // 能量守恒：摆回另一侧时高度接近初始高度
    let maxLeftHeight = -Infinity;
    run(world, 120, () => {
      if (bob.position.x < 0) maxLeftHeight = Math.max(maxLeftHeight, bob.position.y);
    });
    expect(maxLeftHeight).toBeGreaterThan(4.6);
  });

  it('10 节链条：摆动中误差 < 3cm，静止悬挂时 < 3mm 并休眠', () => {
    for (const vertical of [false, true]) {
      const world = new World();
      const anchor = world.createBody({ type: 'static', position: new Vec3(0, 10, 0) });
      const joints: BallSocketJoint[] = [];
      let prev = anchor;
      for (let i = 0; i < 10; i++) {
        const link = vertical
          ? box(world, new Vec3(0, 9.5 - i, 0), new Vec3(0.1, 0.5, 0.1))
          : box(world, new Vec3(0.5 + i, 10, 0), new Vec3(0.5, 0.1, 0.1));
        const at = vertical ? new Vec3(0, 10 - i, 0) : new Vec3(i, 10, 0);
        joints.push(world.addJoint(new BallSocketJoint({ bodyA: prev, bodyB: link, anchor: at })));
        prev = link;
      }
      const a = new Vec3();
      const b = new Vec3();
      let maxError = 0;
      run(world, 600, () => {
        for (const j of joints) maxError = Math.max(maxError, j.getAnchorA(a).distanceTo(j.getAnchorB(b)));
      });
      if (vertical) {
        expect(maxError).toBeLessThan(0.003);
        expect(world.stats.awakeBodies).toBe(0);
      } else {
        expect(maxError).toBeLessThan(0.03);
        expect(prev.position.y).toBeLessThan(5);
      }
    }
  });
});

describe('HingeJoint', () => {
  it('只能绕轴转动，并遵守角度限制', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) });
    const frame = world.createBody({ type: 'static' });
    const door = box(world, new Vec3(0.5, 1, 0), new Vec3(0.5, 1, 0.05));
    const hinge = world.addJoint(
      new HingeJoint({
        bodyA: frame,
        bodyB: door,
        anchor: new Vec3(0, 1, 0),
        axis: new Vec3(0, 1, 0),
        enableLimit: true,
        lowerAngle: -Math.PI / 4,
        upperAngle: Math.PI / 4,
      }),
    );
    // 施加一个偏离轴的角冲量与线冲量
    door.applyImpulse(new Vec3(0, 0, -3), new Vec3(1, 1.5, 0));
    let maxAngle = 0;
    const axisWorld = new Vec3();
    run(world, 120, () => {
      maxAngle = Math.max(maxAngle, Math.abs(hinge.getAngle()));
      door.getWorldVector(new Vec3(0, 1, 0), axisWorld);
      expect(axisWorld.y).toBeGreaterThan(0.999);
    });
    expect(maxAngle).toBeGreaterThan(Math.PI / 4 - 0.05);
    expect(maxAngle).toBeLessThan(Math.PI / 4 + 0.05);
    const a = new Vec3();
    const b = new Vec3();
    expect(hinge.getAnchorA(a).distanceTo(hinge.getAnchorB(b))).toBeLessThan(0.01);
  });

  it('马达达到目标角速度', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) });
    const base = world.createBody({ type: 'static' });
    const wheel = box(world, new Vec3(0, 0, 0), new Vec3(0.5, 0.1, 0.5));
    const hinge = world.addJoint(
      new HingeJoint({
        bodyA: base,
        bodyB: wheel,
        anchor: new Vec3(0, 0, 0),
        axis: new Vec3(0, 1, 0),
        enableMotor: true,
        motorSpeed: 2,
        maxMotorTorque: 100,
      }),
    );
    run(world, 60);
    expect(hinge.getSpeed()).toBeCloseTo(2, 2);
    expect(wheel.angularVelocity.y).toBeCloseTo(2, 2);
  });

  it('重力下的水平轴摆动：轴保持对齐', () => {
    const world = new World({ enableSleep: false });
    const frame = world.createBody({ type: 'static', position: new Vec3(0, 3, 0) });
    const arm = box(world, new Vec3(1, 3, 0), new Vec3(1, 0.1, 0.1));
    const hinge = world.addJoint(
      new HingeJoint({ bodyA: frame, bodyB: arm, anchor: new Vec3(0, 3, 0), axis: new Vec3(0, 0, 1) }),
    );
    const axisWorld = new Vec3();
    run(world, 240, () => {
      arm.getWorldVector(new Vec3(0, 0, 1), axisWorld);
      expect(axisWorld.z).toBeGreaterThan(0.999);
    });
    expect(Math.abs(hinge.getAngle())).toBeGreaterThan(0.1);
  });
});

describe('DistanceJoint', () => {
  it('刚性杆保持长度', () => {
    const world = new World({ enableSleep: false });
    const a = world.createBody({ type: 'static', position: new Vec3(0, 5, 0) });
    const b = sphere(world, new Vec3(1.5, 5, 0));
    const joint = world.addJoint(new DistanceJoint({ bodyA: a, bodyB: b, anchorA: a.position, anchorB: b.position }));
    let maxErr = 0;
    run(world, 240, () => {
      maxErr = Math.max(maxErr, Math.abs(joint.getCurrentLength() - 1.5));
    });
    expect(maxErr).toBeLessThan(0.02);
  });

  it('弹簧围绕静止长度振荡并最终稳定', () => {
    const world = new World({ enableSleep: false });
    const a = world.createBody({ type: 'static', position: new Vec3(0, 5, 0) });
    const b = sphere(world, new Vec3(0, 4, 0));
    const joint = world.addJoint(
      new DistanceJoint({
        bodyA: a,
        bodyB: b,
        anchorA: a.position,
        anchorB: b.position,
        enableSpring: true,
        hertz: 2,
        dampingRatio: 0.5,
      }),
    );
    const lengths: number[] = [];
    run(world, 600, () => lengths.push(joint.getCurrentLength()));
    // 重力使弹簧伸长，然后稳定
    expect(Math.max(...lengths)).toBeGreaterThan(1.05);
    const tail = lengths.slice(-30);
    expect(Math.max(...tail) - Math.min(...tail)).toBeLessThan(0.005);
  });

  it('绳索：只限制最大长度', () => {
    const world = new World({ enableSleep: false });
    const a = world.createBody({ type: 'static', position: new Vec3(0, 5, 0) });
    const b = sphere(world, new Vec3(0.5, 5, 0));
    const joint = world.addJoint(
      new DistanceJoint({
        bodyA: a,
        bodyB: b,
        anchorA: a.position,
        anchorB: b.position,
        enableLimit: true,
        minLength: 0,
        maxLength: 2,
        enableSpring: true,
        hertz: 0,
      }),
    );
    let maxLen = 0;
    run(world, 300, () => {
      maxLen = Math.max(maxLen, joint.getCurrentLength());
    });
    expect(maxLen).toBeGreaterThan(1.9);
    expect(maxLen).toBeLessThan(2.03);
  });
});

describe('FixedJoint 与 SliderJoint', () => {
  it('焊接的悬臂几乎不下垂、不转动', () => {
    const world = new World({ enableSleep: false });
    const wall = box(world, new Vec3(0, 5, 0), new Vec3(0.5, 0.5, 0.5), 'static');
    const beam = box(world, new Vec3(1.5, 5, 0), new Vec3(1, 0.1, 0.1));
    world.addJoint(new FixedJoint({ bodyA: wall, bodyB: beam, anchor: new Vec3(0.5, 5, 0) }));
    run(world, 240);
    expect(Math.abs(beam.position.y - 5)).toBeLessThan(0.02);
    expect(beam.rotation.getAngle()).toBeLessThan(0.02);
  });

  it('滑动关节：只沿轴平移，遵守限制，马达可驱动', () => {
    const world = new World({ enableSleep: false });
    const rail = world.createBody({ type: 'static', position: new Vec3(0, 3, 0) });
    const cart = box(world, new Vec3(0, 3, 0), new Vec3(0.3, 0.2, 0.2));
    const slider = world.addJoint(
      new SliderJoint({
        bodyA: rail,
        bodyB: cart,
        anchor: new Vec3(0, 3, 0),
        axis: new Vec3(1, 0, 0),
        enableLimit: true,
        lowerTranslation: -1,
        upperTranslation: 2,
        enableMotor: true,
        motorSpeed: 1,
        maxMotorForce: 100,
      }),
    );
    run(world, 60);
    // 重力不应让小车下落
    expect(Math.abs(cart.position.y - 3)).toBeLessThan(0.01);
    expect(cart.linearVelocity.x).toBeCloseTo(1, 1);
    expect(slider.getTranslation()).toBeCloseTo(1, 1);
    run(world, 180);
    expect(slider.getTranslation()).toBeLessThan(2.02);
    expect(slider.getTranslation()).toBeGreaterThan(1.95);
    expect(cart.rotation.getAngle()).toBeLessThan(0.01);
  });
});

describe('MouseJoint 与碰撞设置', () => {
  it('鼠标关节把刚体拖向目标点', () => {
    const world = new World();
    const ground = world.createBody({ type: 'static' });
    ground.addCollider({ shape: new PlaneShape() });
    const b = box(world, new Vec3(0, 0.5, 0), new Vec3(0.5, 0.5, 0.5));
    const mouse = world.addJoint(
      new MouseJoint({ bodyA: ground, bodyB: b, anchor: new Vec3(0, 0.5, 0), target: new Vec3(0, 3, 0) }),
    );
    run(world, 120);
    expect(b.position.y).toBeGreaterThan(2.8);
    mouse.setTarget(new Vec3(2, 3, 0));
    run(world, 120);
    expect(b.position.x).toBeGreaterThan(1.8);
    world.removeJoint(mouse);
    run(world, 180);
    expect(b.position.y).toBeLessThan(0.6);
  });

  it('collideConnected=false 时相连刚体互不碰撞，移除关节后恢复碰撞', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) });
    const a = box(world, new Vec3(0, 0, 0), new Vec3(0.5, 0.5, 0.5));
    const b = box(world, new Vec3(0.8, 0, 0), new Vec3(0.5, 0.5, 0.5));
    const joint = world.addJoint(new BallSocketJoint({ bodyA: a, bodyB: b, anchor: new Vec3(0.4, 0, 0) }));
    run(world, 10);
    expect(world.stats.touchingContacts).toBe(0);
    world.removeJoint(joint);
    run(world, 2);
    expect(world.stats.touchingContacts).toBe(1);
  });
});
