import {
  ConeTwistJoint,
  HingeJoint,
  Quat,
  type RigidBody,
  Vec3,
  type World,
} from '@thunder/physics';
import type { Demo } from './types';
import { addBox, addCapsule, addSphere, v } from './helpers';

const ARM = new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2);

/**
 * 由胶囊、球与长方体组成的布娃娃：
 * 颈、肩、髋为锥形扭转关节（摆动锥 + 扭转范围 + 关节摩擦），肘、膝为带角度限制的铰链。
 */
export function createRagdoll(world: World, o: Vec3): RigidBody[] {
  const at = (x: number, y: number, z: number) => v(o.x + x, o.y + y, o.z + z);
  const torso = addBox(world, at(0, 0, 0), v(0.2, 0.3, 0.12));
  const head = addSphere(world, at(0, 0.48, 0), 0.13);
  world.addJoint(
    new ConeTwistJoint({
      bodyA: torso,
      bodyB: head,
      anchor: at(0, 0.33, 0),
      twistAxis: v(0, 1, 0),
      swingSpan: 0.6,
      twistLower: -0.8,
      twistUpper: 0.8,
      maxFrictionTorque: 0.4,
    }),
  );
  const parts = [torso, head];
  for (const side of [-1, 1]) {
    const upperArm = addCapsule(world, at(side * 0.43, 0.22, 0), 0.06, 0.15, {
      body: { rotation: ARM },
    });
    const lowerArm = addCapsule(world, at(side * 0.87, 0.22, 0), 0.055, 0.15, {
      body: { rotation: ARM },
    });
    world.addJoint(
      new ConeTwistJoint({
        bodyA: torso,
        bodyB: upperArm,
        anchor: at(side * 0.21, 0.22, 0),
        twistAxis: v(side, 0, 0),
        swingSpan: 1.4,
        twistLower: -1,
        twistUpper: 1,
        maxFrictionTorque: 0.5,
      }),
    );
    world.addJoint(
      new HingeJoint({
        bodyA: upperArm,
        bodyB: lowerArm,
        anchor: at(side * 0.65, 0.22, 0),
        axis: v(0, 0, 1),
        enableLimit: true,
        lowerAngle: side < 0 ? -2.3 : 0,
        upperAngle: side < 0 ? 0 : 2.3,
      }),
    );
    const upperLeg = addCapsule(world, at(side * 0.1, -0.57, 0), 0.075, 0.18);
    const lowerLeg = addCapsule(world, at(side * 0.1, -1.09, 0), 0.065, 0.18);
    world.addJoint(
      new ConeTwistJoint({
        bodyA: torso,
        bodyB: upperLeg,
        anchor: at(side * 0.1, -0.31, 0),
        twistAxis: v(0, -1, 0),
        swingSpan: 1.1,
        twistLower: -0.5,
        twistUpper: 0.5,
        maxFrictionTorque: 1,
      }),
    );
    world.addJoint(
      new HingeJoint({
        bodyA: upperLeg,
        bodyB: lowerLeg,
        anchor: at(side * 0.1, -0.83, 0),
        axis: v(1, 0, 0),
        enableLimit: true,
        lowerAngle: 0,
        upperAngle: 2.3,
      }),
    );
    parts.push(upperArm, lowerArm, upperLeg, lowerLeg);
  }
  return parts;
}

export const ragdoll: Demo = {
  id: 'ragdoll',
  name: '布娃娃',
  description:
    '胶囊 + 锥形扭转 / 铰链关节组成的布娃娃从楼梯上滚落：颈、肩、髋有摆动锥与扭转范围并带关节摩擦，肘、膝是带角度限制的铰链。',
  camera: { position: [8, 6, 6], target: [0, 1.5, -2.5] },
  setup({ world, setInfo }) {
    // 楼梯
    for (let i = 0; i < 8; i++) {
      const h = 0.4 * (8 - i);
      addBox(world, v(0, h / 2, -6 + i * 0.8), v(3, h / 2, 0.4), { body: { type: 'static' } });
    }
    for (let i = 0; i < 4; i++) {
      // 给一个向楼梯下方的初速度，让布娃娃沿台阶翻滚
      for (const part of createRagdoll(world, v(-1.5 + i, 5 + i * 1.2, -5.5))) {
        part.setLinearVelocity(v(0, 0, 3));
      }
    }
    setInfo('拖拽布娃娃的任意部位，关节不会扭到不自然的角度。');
  },
};
