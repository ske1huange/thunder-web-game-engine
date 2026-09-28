import { describe, expect, it } from 'vitest';
import {
  BoxShape,
  ConeTwistJoint,
  PlaneShape,
  RaycastVehicle,
  SphereShape,
  Vec3,
  World,
} from '../src';

const DT = 1 / 60;

function makeCar(world: World, position = new Vec3(0, 1.2, 0)) {
  const chassis = world.createBody({ position, angularDamping: 0.1 });
  // 车身 + 较低的配重，降低质心
  chassis.addCollider({ shape: new BoxShape(new Vec3(0.9, 0.3, 2)), density: 150 });
  chassis.addCollider({
    shape: new BoxShape(new Vec3(0.8, 0.1, 1.6)),
    position: new Vec3(0, -0.3, 0),
    density: 400,
  });
  const car = new RaycastVehicle(world, { chassis });
  for (const [x, z] of [
    [-0.85, -1.3],
    [0.85, -1.3],
    [-0.85, 1.3],
    [0.85, 1.3],
  ] as const) {
    car.addWheel({
      position: new Vec3(x, -0.2, z),
      radius: 0.4,
      suspensionRestLength: 0.35,
      suspensionStiffness: 40,
      frictionSlip: 3,
    });
  }
  return { chassis, car };
}

function flat(): World {
  const world = new World();
  const ground = world.createBody({ type: 'static' });
  ground.addCollider({ shape: new PlaneShape(), friction: 0.8 });
  return world;
}

function run(world: World, steps: number, each?: (i: number) => void): void {
  for (let i = 0; i < steps; i++) {
    each?.(i);
    world.step(DT);
  }
}

describe('RaycastVehicle', () => {
  it('悬挂支撑车身静止在地面上方，四轮着地', () => {
    const world = flat();
    const { chassis, car } = makeCar(world);
    run(world, 180);
    expect(car.wheelsInContact).toBe(4);
    // 车身不接触地面（只由悬挂支撑）
    expect(world.contacts.filter((c) => c.touching).length).toBe(0);
    const w = car.wheels[0]!;
    // 静态压缩量 = g / (4 × 刚度)（每个轮子承担 1/4 车重）
    const expected = w.suspensionRestLength - 9.81 / (4 * w.suspensionStiffness);
    expect(w.suspensionLength).toBeCloseTo(expected, 1);
    expect(chassis.linearVelocity.length()).toBeLessThan(0.01);
    run(world, 60);
    expect(chassis.isAwake).toBe(false);
    // 车身水平
    const up = chassis.getWorldVector(new Vec3(0, 1, 0), new Vec3());
    expect(up.y).toBeGreaterThan(0.999);
  });

  it('后轮驱动向前直行，刹车后停下', () => {
    const world = flat();
    const { chassis, car } = makeCar(world);
    run(world, 60);
    run(world, 180, () => {
      car.applyEngineForce(3000, 2);
      car.applyEngineForce(3000, 3);
    });
    const speed = car.forwardSpeed;
    expect(speed).toBeGreaterThan(8);
    expect(chassis.position.z).toBeLessThan(-8); // 车头朝 -Z
    expect(Math.abs(chassis.position.x)).toBeLessThan(0.1);
    expect(car.wheels[2]!.rotation).toBeGreaterThan(10);
    const zBrake = chassis.position.z;
    run(world, 240, () => {
      for (let i = 0; i < 4; i++) {
        car.applyEngineForce(0, i);
        car.setBrake(4000, i);
      }
    });
    expect(Math.abs(car.forwardSpeed)).toBeLessThan(0.1);
    // 刹车距离合理（v² / 2a，a 至少 ~3 m/s²）
    expect(zBrake - chassis.position.z).toBeLessThan((speed * speed) / (2 * 3));
  });

  it('前轮转向使车辆转弯', () => {
    const world = flat();
    const { chassis, car } = makeCar(world);
    run(world, 60);
    run(world, 240, () => {
      car.setSteering(0.4, 0);
      car.setSteering(0.4, 1);
      car.applyEngineForce(1500, 2);
      car.applyEngineForce(1500, 3);
    });
    // 正转向角向左（-X）转；稳态转弯半径 ≈ 轴距 / tan(转向角)
    expect(chassis.position.x).toBeLessThan(-2);
    const radius = car.forwardSpeed / chassis.angularVelocity.y;
    expect(radius).toBeGreaterThan((2.6 / Math.tan(0.4)) * 0.9);
    expect(radius).toBeLessThan((2.6 / Math.tan(0.4)) * 1.3);
    const up = chassis.getWorldVector(new Vec3(0, 1, 0), new Vec3());
    expect(up.y).toBeGreaterThan(0.9);
  });

  it('离地时悬挂伸到最长，落地后恢复', () => {
    const world = flat();
    const { car } = makeCar(world, new Vec3(0, 4, 0));
    run(world, 5);
    expect(car.wheelsInContact).toBe(0);
    const w = car.wheels[0]!;
    expect(w.suspensionLength).toBeCloseTo(w.suspensionRestLength + w.maxSuspensionTravel, 9);
    run(world, 180);
    expect(car.wheelsInContact).toBe(4);
  });

  it('压在动态物体上时对其施加反作用力', () => {
    const world = flat();
    const plate = world.createBody({ position: new Vec3(0, 0.1, 0) });
    plate.addCollider({ shape: new BoxShape(new Vec3(3, 0.1, 3)), density: 10, friction: 0.8 });
    const { car } = makeCar(world, new Vec3(0, 1.4, 0));
    run(world, 120);
    expect(car.wheelsInContact).toBe(4);
    expect(car.wheels[0]!.groundBody).toBe(plate);
    // 板被车压住，仍然平放在地面上
    expect(plate.position.y).toBeGreaterThan(0.08);
    expect(plate.position.y).toBeLessThan(0.12);
  });
});

describe('ConeTwistJoint', () => {
  function pendulum(options: { swingSpan?: number; friction?: number } = {}) {
    const world = new World({ gravity: new Vec3(0, -9.81, 0) });
    const anchor = world.createBody({ type: 'static', position: new Vec3(0, 5, 0) });
    const bob = world.createBody({ position: new Vec3(0, 4, 0) });
    bob.addCollider({ shape: new SphereShape(0.2) });
    const joint = world.addJoint(
      new ConeTwistJoint({
        bodyA: anchor,
        bodyB: bob,
        anchor: new Vec3(0, 5, 0),
        twistAxis: new Vec3(0, -1, 0),
        swingSpan: options.swingSpan ?? Math.PI / 6,
        twistLower: -0.2,
        twistUpper: 0.2,
        maxFrictionTorque: options.friction ?? 0,
      }),
    );
    return { world, bob, joint };
  }

  it('摆动角限制在圆锥内，锚点保持重合', () => {
    const { world, bob, joint } = pendulum();
    bob.setLinearVelocity(new Vec3(6, 0, 3));
    let maxSwing = 0;
    let maxGap = 0;
    const a = new Vec3();
    const b = new Vec3();
    run(world, 240, () => {
      maxSwing = Math.max(maxSwing, joint.getSwingAngle());
      maxGap = Math.max(maxGap, joint.getAnchorA(a).distanceTo(joint.getAnchorB(b)));
    });
    expect(maxSwing).toBeLessThan(Math.PI / 6 + 0.1);
    expect(maxSwing).toBeGreaterThan(Math.PI / 6 - 0.05);
    // 高速撞上限制的瞬间点约束会被拉开几厘米，随后恢复
    expect(maxGap).toBeLessThan(0.1);
    expect(joint.getAnchorA(a).distanceTo(joint.getAnchorB(b))).toBeLessThan(0.01);
  });

  it('扭转角限制', () => {
    const { world, bob, joint } = pendulum();
    bob.setAngularVelocity(new Vec3(0, 8, 0));
    let maxTwist = 0;
    run(world, 120, () => {
      maxTwist = Math.max(maxTwist, Math.abs(joint.getTwistAngle()));
    });
    expect(maxTwist).toBeLessThan(0.25);
    expect(maxTwist).toBeGreaterThan(0.15);
  });

  it('关节摩擦让摆动更快停下', () => {
    const energy = (friction: number): number => {
      const { world, bob } = pendulum({ swingSpan: Math.PI / 2, friction });
      bob.setLinearVelocity(new Vec3(3, 0, 0));
      run(world, 180);
      return bob.linearVelocity.length();
    };
    expect(energy(5)).toBeLessThan(energy(0) * 0.5);
  });
});
