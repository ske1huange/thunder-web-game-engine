import { Mat3 } from '../../math/Mat3';
import { Quat } from '../../math/Quat';
import { Vec3 } from '../../math/Vec3';
import type { SolverContext } from '../solver/SolverContext';
import { initAnchors, solvePointConstraint } from './BallSocketJoint';
import { Joint, type JointOptions } from './Joint';
import {
  angularMassMatrix,
  applyAngularImpulse,
  applyPointImpulse,
  currentArm,
  currentRotation,
  swingTwist,
} from './JointUtils';

export interface ConeTwistJointOptions extends JointOptions {
  /** 世界坐标锚点 */
  anchor?: Readonly<Vec3>;
  localAnchorA?: Readonly<Vec3>;
  localAnchorB?: Readonly<Vec3>;
  /** 世界坐标扭转轴（例如从肩部指向手肘），默认 bodyA 的 +Y */
  twistAxis?: Readonly<Vec3>;
  /** 摆动锥的半角（弧度），默认 π/4 */
  swingSpan?: number;
  /** 扭转角下限 / 上限（弧度），默认 ±π/4 */
  twistLower?: number;
  twistUpper?: number;
  /** 关节摩擦力矩（N·m），让布娃娃的关节不那么松软，默认 0 */
  maxFrictionTorque?: number;
}

const a = new Vec3();
const s = new Vec3();
const rA = new Vec3();
const rB = new Vec3();
const swingVec = new Vec3();
const L = new Vec3();
const tv = new Vec3();
const qA = new Quat();
const qB = new Quat();
const qE = new Quat();
const K = new Mat3();
const frictionDelta = new Vec3();

/**
 * 锥形扭转关节（布娃娃的肩、髋、颈）：锚点重合，
 * 扭转轴的摆动被限制在半角为 swingSpan 的圆锥内，绕扭转轴的转角限制在 [twistLower, twistUpper]。
 *
 * 相对旋转按摆动-扭转分解（与铰链关节相同），两种限制都是推测式单边约束；
 * 可选的关节摩擦按最大力矩截断，模拟肌肉的阻尼。
 */
export class ConeTwistJoint extends Joint {
  readonly localAnchorA = new Vec3();
  readonly localAnchorB = new Vec3();
  /** bodyA 局部坐标系中的扭转轴 */
  readonly localAxisA = new Vec3(0, 1, 0);
  /** 创建时的相对旋转 qA⁻¹ qB（摆动与扭转都为 0） */
  readonly relativeRotation = new Quat();

  swingSpan: number;
  twistLower: number;
  twistUpper: number;
  maxFrictionTorque: number;

  readonly linearImpulse = new Vec3();
  private swingImpulse = 0;
  private twistLowerImpulse = 0;
  private twistUpperImpulse = 0;
  private readonly frictionImpulse = new Vec3();

  constructor(options: ConeTwistJointOptions) {
    super(options);
    initAnchors(this, options);
    if (options.twistAxis) {
      a.copy(options.twistAxis);
      a.normalize();
      this.bodyA.getLocalVector(a, this.localAxisA);
      this.localAxisA.normalize();
    }
    this.relativeRotation.multiplyConjugateA(this.bodyA.rotation, this.bodyB.rotation);
    this.swingSpan = options.swingSpan ?? Math.PI / 4;
    this.twistLower = options.twistLower ?? -Math.PI / 4;
    this.twistUpper = options.twistUpper ?? Math.PI / 4;
    this.maxFrictionTorque = options.maxFrictionTorque ?? 0;
  }

  /** 当前摆动角（扭转轴偏离初始方向的角度） */
  getSwingAngle(): number {
    this.computeFramesFrom(this.bodyA.rotation, this.bodyB.rotation);
    return swingVec.length();
  }

  /** 当前扭转角 */
  getTwistAngle(): number {
    return this.computeFramesFrom(this.bodyA.rotation, this.bodyB.rotation);
  }

  prepare(ctx: SolverContext): void {
    if (!ctx.enableWarmStarting) {
      this.linearImpulse.setZero();
      this.swingImpulse = this.twistLowerImpulse = this.twistUpperImpulse = 0;
      this.frictionImpulse.setZero();
    }
    if (this.maxFrictionTorque <= 0) this.frictionImpulse.setZero();
  }

  /** 由两刚体朝向计算扭转轴 a、摆动旋转向量 swingVec，返回扭转角 */
  private computeFramesFrom(rotA: Readonly<Quat>, rotB: Readonly<Quat>): number {
    rotA.rotate(this.localAxisA, a);
    // 误差旋转 qE = qB (qA qRel)⁻¹，分解为绕 a 的扭转与摆动
    qE.multiplyQuats(rotA, this.relativeRotation).conjugate();
    qE.premultiply(rotB);
    return swingTwist(qE, a, swingVec);
  }

  private computeFrames(): number {
    currentRotation(this.bodyA, qA);
    currentRotation(this.bodyB, qB);
    return this.computeFramesFrom(qA, qB);
  }

  warmStart(): void {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    this.computeFrames();
    currentArm(bodyA, this.localAnchorA, rA);
    currentArm(bodyB, this.localAnchorB, rB);
    applyPointImpulse(bodyA, bodyB, rA, rB, this.linearImpulse);
    L.copy(this.frictionImpulse).addScaled(a, this.twistLowerImpulse - this.twistUpperImpulse);
    const swing = swingVec.length();
    if (swing > 1e-9) L.addScaled(swingVec, -this.swingImpulse / swing);
    applyAngularImpulse(bodyA, bodyB, L);
  }

  solve(ctx: SolverContext, useBias: boolean): void {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    const twistAngle = this.computeFrames();
    const IA = bodyA.invInertiaWorld;
    const IB = bodyB.invInertiaWorld;
    const soft = ctx.jointSoftness;

    // ---- 关节摩擦：按最大力矩截断的角速度阻尼 ----
    if (this.maxFrictionTorque > 0) {
      const maxImpulse = this.maxFrictionTorque * ctx.h;
      tv.subVectors(bodyB.angularVelocity, bodyA.angularVelocity);
      angularMassMatrix(bodyA, bodyB, K);
      K.solve(tv, frictionDelta);
      const old = L.copy(this.frictionImpulse);
      this.frictionImpulse.addScaled(frictionDelta, -1);
      const len = this.frictionImpulse.length();
      if (len > maxImpulse) this.frictionImpulse.scale(maxImpulse / len);
      frictionDelta.subVectors(this.frictionImpulse, old);
      applyAngularImpulse(bodyA, bodyB, frictionDelta);
    }

    // ---- 摆动限制：摆动角 ≤ swingSpan，约束方向为摆动旋转轴 ----
    const swing = swingVec.length();
    if (swing > 1e-6) {
      s.copy(swingVec).scale(1 / swing);
      const k = IA.quadraticForm(s) + IB.quadraticForm(s);
      const mass = k > 0 ? 1 / k : 0;
      const C = this.swingSpan - swing;
      let bias = 0;
      let massScale = 1;
      let impulseScale = 0;
      if (C > 0) bias = C * ctx.invH;
      else if (useBias) {
        bias = soft.biasRate * C;
        massScale = soft.massScale;
        impulseScale = soft.impulseScale;
      }
      const wRel = tv.subVectors(bodyB.angularVelocity, bodyA.angularVelocity).dot(s);
      let impulse = -mass * massScale * (-wRel + bias) - impulseScale * this.swingImpulse;
      const newImpulse = Math.max(this.swingImpulse + impulse, 0);
      impulse = newImpulse - this.swingImpulse;
      this.swingImpulse = newImpulse;
      L.copy(s).scale(-impulse);
      applyAngularImpulse(bodyA, bodyB, L);
    } else {
      this.swingImpulse = 0;
    }

    // ---- 扭转限制 ----
    {
      const k = IA.quadraticForm(a) + IB.quadraticForm(a);
      const mass = k > 0 ? 1 / k : 0;
      const wRel = (): number => tv.subVectors(bodyB.angularVelocity, bodyA.angularVelocity).dot(a);
      // 下限
      {
        const C = twistAngle - this.twistLower;
        let bias = 0;
        let massScale = 1;
        let impulseScale = 0;
        if (C > 0) bias = C * ctx.invH;
        else if (useBias) {
          bias = soft.biasRate * C;
          massScale = soft.massScale;
          impulseScale = soft.impulseScale;
        }
        let impulse = -mass * massScale * (wRel() + bias) - impulseScale * this.twistLowerImpulse;
        const newImpulse = Math.max(this.twistLowerImpulse + impulse, 0);
        impulse = newImpulse - this.twistLowerImpulse;
        this.twistLowerImpulse = newImpulse;
        L.copy(a).scale(impulse);
        applyAngularImpulse(bodyA, bodyB, L);
      }
      // 上限
      {
        const C = this.twistUpper - twistAngle;
        let bias = 0;
        let massScale = 1;
        let impulseScale = 0;
        if (C > 0) bias = C * ctx.invH;
        else if (useBias) {
          bias = soft.biasRate * C;
          massScale = soft.massScale;
          impulseScale = soft.impulseScale;
        }
        let impulse = -mass * massScale * (-wRel() + bias) - impulseScale * this.twistUpperImpulse;
        const newImpulse = Math.max(this.twistUpperImpulse + impulse, 0);
        impulse = newImpulse - this.twistUpperImpulse;
        this.twistUpperImpulse = newImpulse;
        L.copy(a).scale(-impulse);
        applyAngularImpulse(bodyA, bodyB, L);
      }
    }

    // ---- 点约束 ----
    solvePointConstraint(
      this,
      this.localAnchorA,
      this.localAnchorB,
      this.linearImpulse,
      ctx,
      useBias,
    );
  }

  getAnchorA(out: Vec3): Vec3 {
    return this.bodyA.getWorldPoint(this.localAnchorA, out);
  }

  getAnchorB(out: Vec3): Vec3 {
    return this.bodyB.getWorldPoint(this.localAnchorB, out);
  }
}
