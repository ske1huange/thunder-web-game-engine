import { BoxShape, type RigidBody, Vec3 } from '@thunder/physics';
import type { Demo } from './types';
import { addBox, addSphere, q, rng, v } from './helpers';

export const sensor: Demo = {
  id: 'sensor',
  name: '传感器（触发区）',
  description: '半透明绿色区域是传感器，只检测重叠不产生碰撞。进入区域的物体被染成绿色。',
  camera: { position: [-2, 9, 15], target: [0, 1, 0] },
  setup({ world, view, setInfo }) {
    // 斜坡
    addBox(world, v(-6, 3, 0), v(4, 0.2, 2), {
      body: { type: 'static', rotation: q(0, 0, -0.35) },
      friction: 0.3,
    });
    const zoneBody = world.createBody({ type: 'static', position: v(2, 1, 0) });
    const zone = zoneBody.addCollider({ shape: new BoxShape(new Vec3(2, 1, 2.5)), isSensor: true });
    const inside = new Set<RigidBody>();
    world.on('sensorEnter', (e) => {
      if (e.sensor !== zone) return;
      inside.add(e.visitor.body);
      view.colorOverride.set(e.visitor.body, 0x22c55e);
    });
    world.on('sensorExit', (e) => {
      if (e.sensor !== zone) return;
      inside.delete(e.visitor.body);
      view.colorOverride.delete(e.visitor.body);
    });
    const rand = rng(11);
    let timer = 0;
    const balls: RigidBody[] = [];
    return (dt) => {
      timer += dt;
      if (timer > 0.4) {
        timer = 0;
        const b =
          rand() < 0.5
            ? addSphere(world, v(-9, 6, (rand() - 0.5) * 2), 0.25 + rand() * 0.2, { friction: 0.3 })
            : addBox(world, v(-9, 6, (rand() - 0.5) * 2), v(0.25, 0.25, 0.25), { friction: 0.2 });
        balls.push(b);
      }
      for (let i = balls.length - 1; i >= 0; i--) {
        const b = balls[i]!;
        if (b.position.x > 14 || b.position.y < -5) {
          inside.delete(b);
          view.colorOverride.delete(b);
          world.destroyBody(b);
          balls.splice(i, 1);
        }
      }
      setInfo(`区域内物体数量：${inside.size}（sensorEnter / sensorExit 事件驱动）`);
    };
  },
};
