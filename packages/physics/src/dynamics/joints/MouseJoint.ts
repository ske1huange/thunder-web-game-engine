import { Mat3 } from '../../math/Mat3';
import { Vec3 } from '../../math/Vec3';
import { Softness } from '../solver/Softness';
import type { SolverContext } from '../solver/SolverContext';
import { Joint, type JointOptions } from './Joint';
import { currentArm } from './JointUtils';

export interface MouseJointOptions extends JointOptions {
  /** 被拖拽刚体（bodyB）上的抓取点（世界坐标） */
  anchor: Readonly<Vec3>;
  /** 目标点（世界坐标），默认等于 anchor */
  target?: Readonly<Vec3>;
  /** 最大拉力（N），默认 1000 * bodyB.mass */
  maxForce?: number;
  /** 弹簧频率，默认 5Hz */
  hertz?: number;
  /** 阻尼比，默认 0.7 */
  dampingRatio?: number;
}

const rB = new Vec3();
const cdot = new Vec3();
const C = new Vec3();
const impulse = new Vec3();
const old = new Vec3();
const K = new Mat3();
const skew = new Mat3();
const tv = new Vec3();
const soft = new Softness();

/** 只对 body 施加作用于臂 r 的冲量 P */
function applyToBody(body: MouseJoint['bodyB'], r: Readonly<Vec3>, P: Readonly<Vec3>): void {
  body.linearVelocity.addScaled(P, body.invMass);
  tv.crossVectors(r, P);
  body.invInertiaWorld.transformVector(tv, tv);
  body.angularVelocity.add(tv);
}

/**
 * 鼠标关节（参考 Box2D）：用软弹簧把 bodyB 上的一点拉向目标点，拉力有上限。
 * bodyA 通常传入一个静态刚体（例如地面），只作为占位，不受影响。
 */
export class MouseJoint extends Joint {
  readonly localAnchorB = new Vec3();
  readonly target = new Vec3();
  maxForce: number;
  hertz: number;
  dampingRatio: number;
  readonly impulse = new Vec3();

  constructor(options: MouseJointOptions) {
    super(options);
    this.bodyB.getLocalPoint(options.anchor, this.localAnchorB);
    this.target.copy(options.target ?? options.anchor);
    this.maxForce = options.maxForce ?? 1000 * Math.max(this.bodyB.mass, 1);
    this.hertz = options.hertz ?? 5;
    this.dampingRatio = options.dampingRatio ?? 0.7;
  }

  setTarget(target: Readonly<Vec3>): void {
    this.target.copy(target);
    this.bodyB.wakeUp();
  }

  prepare(ctx: SolverContext): void {
    if (!ctx.enableWarmStarting) this.impulse.setZero();
  }

  warmStart(): void {
    if (this.bodyB.invMass === 0) return;
    currentArm(this.bodyB, this.localAnchorB, rB);
    applyToBody(this.bodyB, rB, this.impulse);
  }

  solve(ctx: SolverContext): void {
    const body = this.bodyB;
    if (body.invMass === 0) return;
    currentArm(body, this.localAnchorB, rB);
    cdot.crossVectors(body.angularVelocity, rB).add(body.linearVelocity);
    // 当前抓取点 - 目标点
    C.addVectors(body.center0, body.deltaPosition).add(rB).sub(this.target);
    soft.set(this.hertz, this.dampingRatio, ctx.h);
    cdot.addScaled(C, soft.biasRate);
    // 只有 B 参与：K = mB I - [rB]× IB⁻¹ [rB]×
    skew.setSkew(rB);
    K.multiplyMatrices(skew, body.invInertiaWorld).multiplyMatrices(K, skew).scale(-1);
    K.m00 += body.invMass;
    K.m11 += body.invMass;
    K.m22 += body.invMass;
    K.solve(cdot, impulse);
    impulse.scale(-soft.massScale).addScaled(this.impulse, -soft.impulseScale);
    old.copy(this.impulse);
    this.impulse.add(impulse);
    const maxImpulse = this.maxForce * ctx.h;
    const len = this.impulse.length();
    if (len > maxImpulse) this.impulse.scale(maxImpulse / len);
    impulse.subVectors(this.impulse, old);
    applyToBody(body, rB, impulse);
  }

  getAnchorA(out: Vec3): Vec3 {
    return out.copy(this.target);
  }

  getAnchorB(out: Vec3): Vec3 {
    return this.bodyB.getWorldPoint(this.localAnchorB, out);
  }
}
