/**
 * 软约束参数（参考 Box2D v3 的 b2Softness / Erin Catto《Solver2D》）。
 *
 * 给定刚度 hertz、阻尼比 zeta 与子步长 h，得到：
 * - biasRate：位置误差 → 速度偏置的系数
 * - massScale / impulseScale：把硬约束变“软”的缩放系数
 */
export class Softness {
  biasRate = 0;
  massScale = 1;
  impulseScale = 0;

  set(hertz: number, zeta: number, h: number): this {
    if (hertz === 0) {
      this.biasRate = 0;
      this.massScale = 1;
      this.impulseScale = 0;
      return this;
    }
    const omega = 2 * Math.PI * hertz;
    const a1 = 2 * zeta + h * omega;
    const a2 = h * omega * a1;
    const a3 = 1 / (1 + a2);
    this.biasRate = omega / a1;
    this.massScale = a2 * a3;
    this.impulseScale = a3;
    return this;
  }
}

export function makeSoft(hertz: number, zeta: number, h: number): Softness {
  return new Softness().set(hertz, zeta, h);
}
