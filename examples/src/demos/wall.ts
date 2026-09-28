import type { Demo } from './types';
import { addBox, v } from './helpers';

export const wall: Demo = {
  id: 'wall',
  name: '箱子墙（性能）',
  description: '16 × 16 = 256 个箱子砌成的砖墙。按空格朝视线方向发射重球。',
  camera: { position: [0, 5, 20], target: [0, 3, 0] },
  setup({ world, setInfo }) {
    const n = 16;
    const hx = 0.4;
    const hy = 0.2;
    for (let row = 0; row < n; row++) {
      const offset = row % 2 === 0 ? 0 : hx;
      for (let i = 0; i < n; i++) {
        addBox(
          world,
          v((i - n / 2) * 2 * hx + offset, hy + row * 2 * hy, 0),
          v(hx - 0.005, hy, 0.3),
        );
      }
    }
    setInfo('右上角可以查看每步耗时。');
  },
};
