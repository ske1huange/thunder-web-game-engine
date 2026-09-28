import { BoxShape, DistanceJoint, HingeJoint, SliderJoint, Vec3 } from '@thunder/physics';
import type { Demo } from './types';
import { addBox, addSphere, v } from './helpers';

export const machines: Demo = {
  id: 'machines',
  name: '机械装置',
  description: '马达驱动的风车（复合刚体）、滑动关节电梯、弹簧悬挂平台与带回位弹簧的门。',
  camera: { position: [0, 8, 18], target: [0, 3, 0] },
  setup({ world, setInfo }) {
    // ---- 风车：铰链马达 + 复合碰撞体 ----
    const post = addBox(world, v(-7, 2.5, 0), v(0.2, 2.5, 0.2), { body: { type: 'static' } });
    const rotor = world.createBody({ position: v(-7, 5, 0.5) });
    rotor.addCollider({ shape: new BoxShape(new Vec3(2.2, 0.15, 0.1)) });
    rotor.addCollider({ shape: new BoxShape(new Vec3(0.15, 2.2, 0.1)) });
    world.addJoint(
      new HingeJoint({
        bodyA: post,
        bodyB: rotor,
        anchor: v(-7, 5, 0.5),
        axis: v(0, 0, 1),
        enableMotor: true,
        motorSpeed: 1.2,
        maxMotorTorque: 500,
      }),
    );
    for (let i = 0; i < 6; i++) addSphere(world, v(-8 + (i % 3) * 0.8, 8 + i, 0.5), 0.25);

    // ---- 电梯：滑动关节 + 往返马达 ----
    const rail = world.createBody({ type: 'static', position: v(-1, 0, 0) });
    const lift = addBox(world, v(-1, 0.3, 0), v(1.2, 0.1, 1.2), { body: { allowSleep: false } });
    const slider = world.addJoint(
      new SliderJoint({
        bodyA: rail,
        bodyB: lift,
        anchor: v(-1, 0.3, 0),
        axis: v(0, 1, 0),
        enableLimit: true,
        lowerTranslation: 0,
        upperTranslation: 4,
        enableMotor: true,
        motorSpeed: 1.5,
        maxMotorForce: 2000,
      }),
    );
    for (let i = 0; i < 4; i++)
      addBox(world, v(-1.5 + (i % 2), 0.8 + Math.floor(i / 2) * 0.6, 0), v(0.25, 0.25, 0.25));

    // ---- 弹簧悬挂平台 ----
    const frame = world.createBody({ type: 'static', position: v(4, 7, 0) });
    const platform = addBox(world, v(4, 3.5, 0), v(1.5, 0.1, 1.5));
    for (const [dx, dz] of [
      [-1.4, -1.4],
      [1.4, -1.4],
      [-1.4, 1.4],
      [1.4, 1.4],
    ] as const) {
      world.addJoint(
        new DistanceJoint({
          bodyA: frame,
          bodyB: platform,
          anchorA: v(4 + dx, 7, dz),
          anchorB: v(4 + dx, 3.6, dz),
          enableSpring: true,
          hertz: 1.5,
          dampingRatio: 0.2,
        }),
      );
    }
    for (let i = 0; i < 5; i++)
      addBox(world, v(3.5 + (i % 2) * 0.8, 5 + i * 0.7, 0), v(0.3, 0.3, 0.3));

    // ---- 门：限制 + 回位弹簧 ----
    const doorFrame = addBox(world, v(8, 1.5, 0), v(0.1, 1.5, 0.1), { body: { type: 'static' } });
    const door = addBox(world, v(8.8, 1.5, 0), v(0.7, 1.4, 0.05));
    world.addJoint(
      new HingeJoint({
        bodyA: doorFrame,
        bodyB: door,
        anchor: v(8.1, 1.5, 0),
        axis: v(0, 1, 0),
        enableLimit: true,
        lowerAngle: -Math.PI / 2,
        upperAngle: Math.PI / 2,
        enableSpring: true,
        springHertz: 0.8,
        springDampingRatio: 0.3,
      }),
    );

    setInfo(
      '风车由铰链马达驱动；电梯每 3 秒换向；平台由 4 根弹簧（距离关节）吊着；拖拽门会自动回位。',
    );
    let direction = 1;
    let timer = 0;
    return (dt) => {
      timer += dt;
      if (timer > 3) {
        timer = 0;
        direction = -direction;
        slider.motorSpeed = 1.5 * direction;
        lift.wakeUp();
      }
    };
  },
};
