import type { Demo } from './types';
import { addBox, v } from './helpers';

export const pyramid: Demo = {
  id: 'pyramid',
  name: '箱子金字塔',
  description: '12 层共 78 个箱子的金字塔，检验堆叠稳定性与休眠。按空格发射小球击倒它。',
  camera: { position: [14, 8, 16], target: [0, 4, 0] },
  setup({ world, setInfo }) {
    const rows = 12;
    const half = 0.5;
    for (let row = 0; row < rows; row++) {
      for (let i = 0; i < rows - row; i++) {
        const x = (i - (rows - row - 1) / 2) * (2 * half + 0.02);
        addBox(world, v(x, half + row * 2 * half, 0), v(half, half, half));
      }
    }
    setInfo('箱子静止后会进入休眠（颜色变暗）。拖拽或发射小球可以唤醒它们。');
  },
};
