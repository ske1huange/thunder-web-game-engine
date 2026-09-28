import { Quat, Vec3 } from '@thunder/physics';
import type { Demo } from './types';
import { addBox } from './helpers';

export const dominoes: Demo = {
  id: 'dominoes',
  name: '多米诺骨牌',
  description: '沿螺线排列的 120 块骨牌，推倒第一块后依次倒下。',
  camera: { position: [0, 14, 18], target: [0, 0, 0] },
  setup({ world, setInfo }) {
    const count = 120;
    let angle = 0;
    let radius = 2.5;
    let first = null as ReturnType<typeof addBox> | null;
    for (let i = 0; i < count; i++) {
      const x = Math.cos(angle) * radius;
      const z = Math.sin(angle) * radius;
      // 骨牌正面朝向切线方向
      const rot = new Quat().setFromAxisAngle(new Vec3(0, 1, 0), -angle);
      const d = addBox(world, new Vec3(x, 0.5, z), new Vec3(0.25, 0.5, 0.05), {
        friction: 0.5,
        body: { rotation: rot },
      });
      if (i === 0) first = d;
      const spacing = 0.55;
      angle += spacing / radius;
      radius += 0.035;
    }
    let pushed = false;
    setInfo('1 秒后自动推倒第一块骨牌。');
    return (_dt, time) => {
      if (!pushed && time > 1 && first) {
        pushed = true;
        const tangent = new Vec3(0, 0, 1);
        first.applyImpulse(tangent.scale(0.2), first.position.clone().add(new Vec3(0, 0.4, 0)));
      }
    };
  },
};
