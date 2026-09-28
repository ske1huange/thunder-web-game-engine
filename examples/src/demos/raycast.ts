import { SphereShape, Transform, Vec3 } from '@thunder/physics';
import type { Demo } from './types';
import { addBox, addCapsule, addSphere, q, rng, v } from './helpers';

export const raycast: Demo = {
  id: 'raycast',
  name: '射线与形状投射',
  description: '中心的激光每帧发出 72 条射线（world.raycast），并用球形投射（world.castShape）探测下方地面。',
  camera: { position: [0, 16, 14], target: [0, 0, 0] },
  setup({ world, overlay, setInfo }) {
    const rand = rng(3);
    for (let i = 0; i < 24; i++) {
      const a = (i / 24) * Math.PI * 2;
      const r = 4 + rand() * 4;
      const pos = v(Math.cos(a) * r, 1 + rand() * 2, Math.sin(a) * r);
      const kind = i % 3;
      if (kind === 0) addBox(world, pos, v(0.4 + rand() * 0.4, 0.4 + rand() * 0.6, 0.4), { body: { rotation: q(0, rand() * 3, 0) } });
      else if (kind === 1) addSphere(world, pos, 0.4 + rand() * 0.4);
      else addCapsule(world, pos, 0.3, 0.5, { body: { rotation: q(rand(), 0, rand()) } });
    }
    let angle = 0;
    const origin = v(0, 1, 0);
    const probe = new SphereShape(0.5);
    const probeXf = new Transform();
    const down = v(0, -10, 0);
    const center = new Vec3();
    setInfo('红线：射线命中；绿线：命中法线；黄色：球形投射的命中位置。');
    return {
      update: (dt) => {
        angle += dt * 0.5;
      },
      render: () => {
        const dir = new Vec3();
        const end = new Vec3();
        for (let i = 0; i < 72; i++) {
          const a = angle + (i / 72) * Math.PI * 2;
          dir.set(Math.cos(a), -0.05, Math.sin(a));
          dir.normalize();
          const hit = world.raycast(origin, dir, 20);
          if (hit) {
            overlay.drawLine(origin, hit.point, 0xff4040);
            end.copy(hit.point).addScaled(hit.normal, 0.4);
            overlay.drawLine(hit.point, end, 0x40ff80);
          } else {
            end.copy(origin).addScaled(dir, 20);
            overlay.drawLine(origin, end, 0x553333);
          }
        }
        // 球形投射
        probeXf.position.set(Math.cos(angle * 0.7) * 6, 9, Math.sin(angle * 0.7) * 6);
        const hit = world.castShape(probe, probeXf, down);
        if (hit) {
          center.copy(probeXf.position).addScaled(down, hit.fraction);
          overlay.drawLine(probeXf.position, center, 0xffd23f);
          for (let k = 0; k < 24; k++) {
            const t0 = (k / 24) * Math.PI * 2;
            const t1 = ((k + 1) / 24) * Math.PI * 2;
            const p0 = v(center.x + Math.cos(t0) * 0.5, center.y, center.z + Math.sin(t0) * 0.5);
            const p1 = v(center.x + Math.cos(t1) * 0.5, center.y, center.z + Math.sin(t1) * 0.5);
            overlay.drawLine(p0, p1, 0xffd23f);
            const s0 = v(center.x + Math.cos(t0) * 0.5, center.y + Math.sin(t0) * 0.5, center.z);
            const s1 = v(center.x + Math.cos(t1) * 0.5, center.y + Math.sin(t1) * 0.5, center.z);
            overlay.drawLine(s0, s1, 0xffd23f);
          }
        }
      },
    };
  },
};
