import { BallSocketJoint, CapsuleShape, HingeJoint, Quat, Vec3 } from '@thunder/physics';
import type { Demo } from './types';
import { addBox, addSphere, v } from './helpers';

export const chain: Demo = {
  id: 'chain',
  name: '链条与吊桥',
  description: '球窝关节连接的 20 节链条挂着重球；铰链关节连接的木板吊桥。',
  camera: { position: [12, 8, 16], target: [2, 5, 0] },
  setup({ world, setInfo }) {
    // ---- 链条 ----
    const top = world.createBody({ type: 'static', position: v(-4, 12, 0) });
    let prev = top;
    const linkHalf = 0.25;
    const lying = new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2);
    const x0 = -4;
    for (let i = 0; i < 20; i++) {
      // 每节沿 X 放置，首尾相接；关节位于相邻两节的连接处
      const link = world.createBody({
        position: v(x0 + linkHalf + i * 2 * linkHalf, 12, 0),
        rotation: lying,
      });
      link.addCollider({ shape: new CapsuleShape(0.08, linkHalf), density: 2 });
      world.addJoint(
        new BallSocketJoint({ bodyA: prev, bodyB: link, anchor: v(x0 + i * 2 * linkHalf, 12, 0) }),
      );
      prev = link;
    }
    const endX = x0 + 20 * 2 * linkHalf;
    const ball = addSphere(world, v(endX + 0.6, 12, 0), 0.6, { density: 3 });
    world.addJoint(new BallSocketJoint({ bodyA: prev, bodyB: ball, anchor: v(endX, 12, 0) }));

    // ---- 吊桥 ----
    const planks = 14;
    const plankHalf = 0.3;
    const z = 6;
    const startX = -4;
    const y = 4;
    const left = addBox(world, v(startX - 0.5, y / 2, z), v(0.5, y / 2, 1.2), {
      body: { type: 'static' },
    });
    let prevBody = left;
    for (let i = 0; i < planks; i++) {
      const x = startX + plankHalf + i * 2 * plankHalf;
      const plank = addBox(world, v(x, y, z), v(plankHalf - 0.02, 0.05, 1));
      world.addJoint(
        new HingeJoint({
          bodyA: prevBody,
          bodyB: plank,
          anchor: v(x - plankHalf, y, z),
          axis: v(0, 0, 1),
        }),
      );
      prevBody = plank;
    }
    const endPost = startX + planks * 2 * plankHalf;
    const right = addBox(world, v(endPost + 0.5, y / 2, z), v(0.5, y / 2, 1.2), {
      body: { type: 'static' },
    });
    world.addJoint(
      new HingeJoint({ bodyA: prevBody, bodyB: right, anchor: v(endPost, y, z), axis: v(0, 0, 1) }),
    );
    for (let i = 0; i < 5; i++)
      addBox(world, v(startX + 1.5 + i * 0.8, y + 1 + i, z), v(0.25, 0.25, 0.25));
    setInfo('拖拽重球或桥上的箱子试试。');
  },
};
