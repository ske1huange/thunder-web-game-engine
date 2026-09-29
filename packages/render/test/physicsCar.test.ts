import { type HingedPart, PlaneShape, type SlidingPart, World } from '@thunder/physics';
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import { isHingedPart, isSlidingPart, PhysicsCar, type PhysicsCarOptions } from '../src';

const DT = 1 / 60;

function box(name: string, size: [number, number, number], pos: [number, number, number]) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(...size),
    new THREE.MeshStandardMaterial({ color: 0x3366cc }),
  );
  mesh.name = name;
  mesh.position.set(...pos);
  return mesh;
}

/**
 * 由方块拼成的简易车模，车头朝 z 轴的 dir 方向（-1 为默认的 -Z，+1 为 glTF 常见的 +Z），
 * 车轮底部在 y = 0。
 */
function buildCar(dir: 1 | -1 = -1): THREE.Group {
  const car = new THREE.Group();
  car.name = 'Car';
  const body = box('Body', [1.8, 0.6, 4.2], [0, 0.7, 0]);
  const wheelShape = new THREE.CylinderGeometry(0.35, 0.35, 0.25, 16).rotateZ(Math.PI / 2);
  const steering = new THREE.Mesh(new THREE.TorusGeometry(0.18, 0.03, 6, 16));
  steering.name = 'SteeringWheel';
  steering.position.set(-0.4, 0.3, dir * 0.4);
  body.add(steering);
  car.add(body);
  for (const [name, x, z] of [
    ['Wheel_FL', -0.85, 1.3],
    ['Wheel_FR', 0.85, 1.3],
    ['Wheel_RL', -0.85, -1.3],
    ['Wheel_RR', 0.85, -1.3],
  ] as const) {
    const wheel = new THREE.Mesh(wheelShape);
    wheel.name = name;
    wheel.position.set(x, 0.35, dir * z);
    car.add(wheel);
  }
  // 左侧车门：前沿在车头一侧的 z = dir * 0.6
  car.add(box('Door', [0.06, 0.5, 1.0], [-0.93, 0.75, dir * 0.1]));
  car.add(box('Hood', [1.6, 0.04, 1.0], [0, 1.02, dir * 1.5]));
  car.add(box('Spoiler', [1.4, 0.05, 0.3], [0, 1.1, -dir * 1.9]));
  return car;
}

function binding(dir: 1 | -1 = -1): PhysicsCarOptions {
  return {
    forward: [0, 0, dir],
    wheels: [
      { node: 'Wheel_FL' },
      { node: 'Wheel_FR' },
      { node: 'Wheel_RL' },
      { node: 'Wheel_RR' },
    ],
    hinges: [
      { node: 'Door', edge: 'front', angle: 70 },
      { node: 'Hood', edge: 'rear', angle: 50 },
    ],
    sliders: [{ node: 'Spoiler', direction: 'up', travel: 0.15, autoAbove: 4 }],
    steeringWheel: { node: 'SteeringWheel' },
  };
}

function setup(dir: 1 | -1 = -1) {
  const world = new World({ enableSleep: false });
  const ground = world.createBody({ type: 'static' });
  ground.addCollider({ shape: new PlaneShape() });
  const scene = new THREE.Scene();
  const car = new PhysicsCar(world, buildCar(dir), binding(dir));
  scene.add(car.root);
  const run = (steps: number) => {
    for (let i = 0; i < steps; i++) world.step(DT);
    car.sync(1);
  };
  return { world, scene, car, run };
}

const worldPos = (o: THREE.Object3D) => o.getWorldPosition(new THREE.Vector3());

function hinged(car: PhysicsCar, name: string): HingedPart {
  const p = car.part(name);
  if (!isHingedPart(p)) throw new Error(`不是铰链部件：${name}`);
  return p;
}

function sliding(car: PhysicsCar, name: string): SlidingPart {
  const p = car.part(name);
  if (!isSlidingPart(p)) throw new Error(`不是滑动部件：${name}`);
  return p;
}

describe('PhysicsCar', () => {
  it('四轮着地停稳，车身停在模型原位附近', () => {
    const { world, car, run } = setup();
    // 车身 + 车门 + 引擎盖 + 尾翼
    expect(car.bodies.length).toBe(4);
    expect(world.bodies.length).toBe(5);
    expect(car.collisionGroup).toBeLessThan(0);
    for (const c of car.bodies.flatMap((b) => b.colliders)) {
      expect(c.filter.group).toBe(car.collisionGroup);
    }
    // 质心降到车轮中心上方 0.1 m
    expect(car.chassis.center.y).toBeCloseTo(0.45, 2);
    run(180);
    expect(car.vehicle.wheelsInContact).toBe(4);
    expect(Math.abs(car.chassis.position.y)).toBeLessThan(0.05);
    expect(Math.abs(car.chassis.linearVelocity.length())).toBeLessThan(0.05);
    // 车轮节点跟随悬挂：底部仍贴着地面
    const wheel = car.root.getObjectByName('Wheel_FL')!;
    expect(worldPos(wheel).y).toBeCloseTo(0.35, 1);
    // 部件都保持关闭
    expect(hinged(car, 'Door').state).toBe('closed');
    expect(hinged(car, 'Hood').state).toBe('closed');
    expect(sliding(car, 'Spoiler').state).toBe('closed');
  });

  it('车门向外打开，引擎盖顶在打开位置，关上后重新锁止', () => {
    const { car, run } = setup();
    run(60);
    const door = hinged(car, 'Door');
    const hood = hinged(car, 'Hood');
    const doorX = door.body.position.x;
    const hoodY = hood.body.position.y;
    car.open('Door');
    car.open('Hood');
    run(120);
    expect(door.state).toBe('open');
    expect(hood.state).toBe('open');
    // 车门后端转到车身外侧（-X），引擎盖前沿抬起
    expect(door.body.position.x).toBeLessThan(doorX - 0.2);
    expect(hood.body.position.y).toBeGreaterThan(hoodY + 0.25);
    // 渲染节点跟着部件刚体
    const doorNode = car.root.getObjectByName('Door')!;
    expect(worldPos(doorNode).x).toBeCloseTo(door.body.position.x, 3);
    run(60);
    expect(hood.state).toBe('open');
    car.toggle('Door');
    car.toggle('Hood');
    run(180);
    expect(door.state).toBe('closed');
    expect(hood.state).toBe('closed');
    expect(Math.abs(door.body.position.x - doorX)).toBeLessThan(0.03);
  });

  it('glTF 朝向（车头 +Z）同样自动推算铰链方向', () => {
    const { car, run } = setup(1);
    run(60);
    const door = hinged(car, 'Door');
    const x0 = door.body.position.x;
    car.open('Door');
    run(120);
    expect(door.state).toBe('open');
    expect(door.body.position.x).toBeLessThan(x0 - 0.2);
    // 车头方向为 +Z：加油门向 +Z 行驶
    car.setInput({ throttle: 1 });
    run(60);
    expect(car.speed).toBeGreaterThan(1);
    expect(car.chassis.linearVelocity.z).toBeGreaterThan(1);
  });

  it('驾驶：车轮滚动、前轮与方向盘随转向转动，尾翼按车速自动升降', () => {
    const { car, run } = setup();
    run(60);
    const spoiler = sliding(car, 'Spoiler');
    const wheel = car.root.getObjectByName('Wheel_RL')!;
    const front = car.root.getObjectByName('Wheel_FL')!;
    const steering = car.root.getObjectByName('SteeringWheel')!;
    const q0 = wheel.getWorldQuaternion(new THREE.Quaternion());
    const s0 = steering.getWorldQuaternion(new THREE.Quaternion());

    car.setInput({ throttle: 1, steer: 1 });
    run(30);
    expect(car.steering).toBeGreaterThan(0.2);
    // 前轮绕竖直轴转向，方向盘转得更多
    const frontQ = front.getWorldQuaternion(new THREE.Quaternion());
    const frontAxle = new THREE.Vector3(1, 0, 0).applyQuaternion(frontQ);
    expect(Math.abs(frontAxle.z)).toBeGreaterThan(0.15);
    const sAngle = steering.getWorldQuaternion(new THREE.Quaternion()).angleTo(s0);
    expect(sAngle).toBeGreaterThan(1);
    expect(wheel.getWorldQuaternion(new THREE.Quaternion()).angleTo(q0)).toBeGreaterThan(0.1);

    car.setInput({ throttle: 1, steer: 0 });
    run(180);
    expect(car.speed).toBeGreaterThan(4);
    expect(spoiler.state === 'opening' || spoiler.state === 'open').toBe(true);
    // 高速时手动收起：不会被自动逻辑立刻重新打开
    car.close('Spoiler');
    run(30);
    expect(spoiler.state === 'closing' || spoiler.state === 'closed').toBe(true);

    // 刹停：刹车灯状态、低于阈值后回到“慢速”，再次加速会重新升起
    car.setInput({ throttle: 0, brake: 1 });
    run(210);
    expect(car.braking).toBe(true);
    expect(Math.abs(car.speed)).toBeLessThan(0.5);
    car.setInput({ throttle: 1, brake: 0 });
    run(240);
    expect(spoiler.state === 'opening' || spoiler.state === 'open').toBe(true);
  });

  it('destroy 移除刚体、关节与控制器，并把 root 移出场景', () => {
    const { world, scene, car } = setup();
    expect(world.joints.length).toBe(3);
    expect(world.controllers.length).toBe(3);
    car.destroy();
    expect(world.bodies.length).toBe(1);
    expect(world.joints.length).toBe(0);
    expect(world.controllers.length).toBe(0);
    expect(car.root.parent).toBeNull();
    expect(scene.children.length).toBe(0);
    // 世界仍可继续步进
    for (let i = 0; i < 10; i++) world.step(DT);
    expect(world.bodies[0]!.position.lengthSq()).toBe(0);
  });
});
