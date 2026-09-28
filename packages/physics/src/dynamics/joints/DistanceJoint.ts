import { Vec3 } from '../../math/Vec3';
import { Softness } from '../solver/Softness';
import type { SolverContext } from '../solver/SolverContext';
import { Joint, type JointOptions } from './Joint';
import { applyPointImpulse, currentArm, pointVelocity } from './JointUtils';

export interface DistanceJointOptions extends JointOptions {
  /** 世界坐标锚点 A */
  anchorA?: Readonly<Vec3>;
  /** 世界坐标锚点 B */
  anchorB?: Readonly<Vec3>;
  localAnchorA?: Readonly<Vec3>;
  localAnchorB?: Readonly<Vec3>;
  /** 静止长度，默认为创建时两锚点的距离 */
  length?: number;
  /** 弹簧模式：以 hertz/dampingRatio 趋向 length */
  enableSpring?: boolean;
  hertz?: number;
  dampingRatio?: number;
  /** 长度范围限制（绳索效果：minLength = 0, maxLength = L） */
  enableLimit?: boolean;
  minLength?: number;
  maxLength?: number;
}

const rA = new Vec3();
const rB = new Vec3();
const d = new Vec3();
const axis = new Vec3();
const vr = new Vec3();
const P = new Vec3();
const tv = new Vec3();
const springSoft = new Softness();

/**
 * 距离关节：保持两锚点之间的距离。可配置为刚性杆、弹簧或绳索（仅限制最大长度）。
 */
export class DistanceJoint extends Joint {
  readonly localAnchorA = new Vec3();
  readonly localAnchorB = new Vec3();
  length: number;
  enableSpring: boolean;
  hertz: number;
  dampingRatio: number;
  enableLimit: boolean;
  minLength: number;
  maxLength: number;

  private impulse = 0;
  private lowerImpulse = 0;
  private upperImpulse = 0;

  constructor(options: DistanceJointOptions) {
    super(options);
    if (options.anchorA) this.bodyA.getLocalPoint(options.anchorA, this.localAnchorA);
    if (options.anchorB) this.bodyB.getLocalPoint(options.anchorB, this.localAnchorB);
    if (options.localAnchorA) this.localAnchorA.copy(options.localAnchorA);
    if (options.localAnchorB) this.localAnchorB.copy(options.localAnchorB);
    const wA = this.getAnchorA(new Vec3());
    const wB = this.getAnchorB(new Vec3());
    this.length = Math.max(options.length ?? wA.distanceTo(wB), 0.001);
    this.enableSpring = options.enableSpring ?? false;
    this.hertz = options.hertz ?? 2;
    this.dampingRatio = options.dampingRatio ?? 0.5;
    this.enableLimit = options.enableLimit ?? false;
    this.minLength = options.minLength ?? 0;
    this.maxLength = options.maxLength ?? Infinity;
  }

  /** 当前两锚点距离 */
  getCurrentLength(): number {
    return this.getAnchorA(rA).distanceTo(this.getAnchorB(rB));
  }

  prepare(ctx: SolverContext): void {
    if (!ctx.enableWarmStarting) this.impulse = this.lowerImpulse = this.upperImpulse = 0;
    if (!this.enableLimit) this.lowerImpulse = this.upperImpulse = 0;
  }

  private frame(): number {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    currentArm(bodyA, this.localAnchorA, rA);
    currentArm(bodyB, this.localAnchorB, rB);
    d.subVectors(bodyB.center0, bodyA.center0)
      .add(bodyB.deltaPosition)
      .sub(bodyA.deltaPosition)
      .add(rB)
      .sub(rA);
    axis.copy(d);
    const len = axis.normalize();
    if (len === 0) axis.set(1, 0, 0);
    return len;
  }

  warmStart(): void {
    this.frame();
    const total = this.impulse + this.lowerImpulse - this.upperImpulse;
    P.copy(axis).scale(total);
    applyPointImpulse(this.bodyA, this.bodyB, rA, rB, P);
  }

  solve(ctx: SolverContext, useBias: boolean): void {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    const length = this.frame();
    // 轴向有效质量
    let k = bodyA.invMass + bodyB.invMass;
    k += bodyA.invInertiaWorld.quadraticForm(tv.crossVectors(rA, axis));
    k += bodyB.invInertiaWorld.quadraticForm(tv.crossVectors(rB, axis));
    const axialMass = k > 0 ? 1 / k : 0;
    const cdot = (): number => pointVelocity(bodyA, bodyB, rA, rB, vr).dot(axis);

    const soft = ctx.jointSoftness;
    const rigid = !this.enableSpring && !(this.enableLimit && this.minLength < this.maxLength);

    if (this.enableSpring) {
      if (this.hertz > 0) {
        springSoft.set(this.hertz, this.dampingRatio, ctx.h);
        const C = length - this.length;
        const impulse =
          -springSoft.massScale * axialMass * (cdot() + springSoft.biasRate * C) -
          springSoft.impulseScale * this.impulse;
        this.impulse += impulse;
        P.copy(axis).scale(impulse);
        applyPointImpulse(bodyA, bodyB, rA, rB, P);
      }
    } else if (rigid) {
      const C = length - this.length;
      let bias = 0;
      let massScale = 1;
      let impulseScale = 0;
      if (useBias) {
        bias = soft.biasRate * C;
        massScale = soft.massScale;
        impulseScale = soft.impulseScale;
      }
      const impulse = -massScale * axialMass * (cdot() + bias) - impulseScale * this.impulse;
      this.impulse += impulse;
      P.copy(axis).scale(impulse);
      applyPointImpulse(bodyA, bodyB, rA, rB, P);
    }

    if (this.enableLimit) {
      // 下限
      if (this.minLength > 0) {
        const C = length - this.minLength;
        let bias = 0;
        let massScale = 1;
        let impulseScale = 0;
        if (C > 0) bias = C * ctx.invH;
        else if (useBias) {
          bias = soft.biasRate * C;
          massScale = soft.massScale;
          impulseScale = soft.impulseScale;
        }
        let impulse = -massScale * axialMass * (cdot() + bias) - impulseScale * this.lowerImpulse;
        const newImpulse = Math.max(0, this.lowerImpulse + impulse);
        impulse = newImpulse - this.lowerImpulse;
        this.lowerImpulse = newImpulse;
        P.copy(axis).scale(impulse);
        applyPointImpulse(bodyA, bodyB, rA, rB, P);
      }
      // 上限
      if (Number.isFinite(this.maxLength)) {
        const C = this.maxLength - length;
        let bias = 0;
        let massScale = 1;
        let impulseScale = 0;
        if (C > 0) bias = C * ctx.invH;
        else if (useBias) {
          bias = soft.biasRate * C;
          massScale = soft.massScale;
          impulseScale = soft.impulseScale;
        }
        let impulse = -massScale * axialMass * (-cdot() + bias) - impulseScale * this.upperImpulse;
        const newImpulse = Math.max(0, this.upperImpulse + impulse);
        impulse = newImpulse - this.upperImpulse;
        this.upperImpulse = newImpulse;
        P.copy(axis).scale(-impulse);
        applyPointImpulse(bodyA, bodyB, rA, rB, P);
      }
    }
  }

  getAnchorA(out: Vec3): Vec3 {
    return this.bodyA.getWorldPoint(this.localAnchorA, out);
  }

  getAnchorB(out: Vec3): Vec3 {
    return this.bodyB.getWorldPoint(this.localAnchorB, out);
  }
}
