import {
  BoxShape,
  CapsuleShape,
  ConvexHullShape,
  CylinderShape,
  type Shape,
  SphereShape,
  Vec3,
} from '@thunder/physics';
import type { Demo } from './types';
import { addBox, q, rng, v } from './helpers';

export const shapes: Demo = {
  id: 'shapes',
  name: '混合形状',
  description: '球、长方体、胶囊、圆柱与随机凸包（岩石）不断落入容器，最多 300 个。',
  camera: { position: [10, 12, 14], target: [0, 2, 0] },
  setup({ world, setInfo }) {
    // 容器
    const wall = { body: { type: 'static' as const } };
    addBox(world, v(0, 1, -4), v(4, 1, 0.2), wall);
    addBox(world, v(0, 1, 4), v(4, 1, 0.2), wall);
    addBox(world, v(-4, 1, 0), v(0.2, 1, 4), wall);
    addBox(world, v(4, 1, 0), v(0.2, 1, 4), wall);

    const rand = rng(2024);
    const rock = (): Shape => {
      const pts: Vec3[] = [];
      for (let i = 0; i < 14; i++) {
        const p = v(rand() * 2 - 1, rand() * 2 - 1, rand() * 2 - 1);
        p.normalize();
        pts.push(p.multiply(v(0.35 + rand() * 0.2, 0.25 + rand() * 0.2, 0.35 + rand() * 0.2)));
      }
      return new ConvexHullShape(pts);
    };
    const makers: (() => Shape)[] = [
      () => new SphereShape(0.25 + rand() * 0.2),
      () => new BoxShape(v(0.2 + rand() * 0.2, 0.2 + rand() * 0.2, 0.2 + rand() * 0.2)),
      () => new CapsuleShape(0.15 + rand() * 0.1, 0.2 + rand() * 0.2),
      () => new CylinderShape(0.25 + rand() * 0.15, 0.15 + rand() * 0.2, 16),
      rock,
    ];
    let count = 0;
    let timer = 0;
    return (dt) => {
      timer += dt;
      if (timer < 0.08 || count >= 300) return;
      timer = 0;
      const shape = makers[count % makers.length]!();
      const body = world.createBody({
        position: v((rand() - 0.5) * 5, 8 + rand() * 2, (rand() - 0.5) * 5),
        rotation: q(rand() * 6, rand() * 6, rand() * 6),
      });
      body.addCollider({ shape, friction: 0.5 });
      count++;
      setInfo(`已生成 ${count} / 300 个物体`);
    };
  },
};
