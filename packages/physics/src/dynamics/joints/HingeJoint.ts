import { Quat } from '../../math/Quat';
import { computeBasis, Vec3 } from '../../math/Vec3';
import { Softness } from '../solver/Softness';
import type { SolverContext } from '../solver/SolverContext';
import { initAnchors, solvePointConstraint } from './BallSocketJoint';
import { Joint, type JointOptions } from './Joint';
import {
  applyAngularImpulse,
  applyPointImpulse,
  currentArm,
  currentDirection,
  currentRotation,
} from './JointUtils';

export interface HingeJointOptions extends JointOptions {
  /** 世界坐标锚点 */
  anchor?: Readonly<Vec3>;
  /** 世界坐标铰链轴（会被归一化） */
  axis?: Readonly<Vec3>;
  localAnchorA?: Readonly<Vec3>;
  localAnchorB?: Readonly<Vec3>;
  /** bodyA 局部坐标系中的铰链轴 */
  localAxisA?: Readonly<Vec3>;
  enableLimit?: boolean;
  /** 角度下限（弧度） */
  lowerAngle?: number;
  /** 角度上限（弧度） */
  upperAngle?: number;
  enableMotor?: boolean;
  /** 马达目标角速度（rad/s） */
  motorSpeed?: number;
  /** 马达最大扭矩（N·m） */
  maxMotorTorque?: number;
  /** 角度弹簧 */
  enableSpring?: boolean;
  springHertz?: number;
  springDampingRatio?: number;
  /** 弹簧目标角度 */
  targetAngle?: number;
}

const rA = new Vec3();
const rB = new Vec3();
const a = new Vec3();
const j1 = new Vec3();
const j2 = new Vec3();
const L = new Vec3();
const swingVec = new Vec3();
const tv = new Vec3();
const qA = new Quat();
const qB = new Quat();
const qE = new Quat();
const twist = new Quat();
const spring = new Softness();

const TWO_PI = Math.PI * 2;

/**
 * 由相对误差旋转 qE（世界系）与铰链轴 axis 分解出扭转角（绕轴，[-π, π]），
 * 并把摆动部分的旋转向量写入 swingOut。
 */
function swingTwist(e: Quat, axis: Readonly<Vec3>, swingOut: Vec3): number {
  const d = e.x * axis.x + e.y * axis.y + e.z * axis.z;
  let angle = 2 * Math.atan2(d, e.w);
  if (angle > Math.PI) angle -= TWO_PI;
  else if (angle < -Math.PI) angle += TWO_PI;
  twist.set(axis.x * d, axis.y * d, axis.z * d, e.w);
  if (twist.lengthSq() < 1e-18) twist.identity();
  else twist.normalize();
  // swing = e * twist⁻¹
  twist.conjugate();
  twist.premultiply(e);
  twist.toRotationVector(swingOut);
  return angle;
}

/**
 * 铰链关节（旋转关节）：锚点重合，两刚体只能绕公共轴相对转动。
 * 支持角度限制、马达与角度弹簧（门、车轮、摆锤等）。
 */
export class HingeJoint extends Joint {
  readonly localAnchorA = new Vec3();
  readonly localAnchorB = new Vec3();
  /** bodyA 局部坐标系中的铰链轴 */
  readonly localAxisA = new Vec3(0, 1, 0);
  /** 创建时的相对旋转 qA⁻¹ qB，对应铰链角 0 */
  readonly relativeRotation = new Quat();
  /** 与轴垂直、固定在 A 上的两个方向（锁定摆动用） */
  private readonly localPerpA1 = new Vec3();
  private readonly localPerpA2 = new Vec3();

  enableLimit: boolean;
  lowerAngle: number;
  upperAngle: number;
  enableMotor: boolean;
  motorSpeed: number;
  maxMotorTorque: number;
  enableSpring: boolean;
  springHertz: number;
  springDampingRatio: number;
  targetAngle: number;

  // 累积冲量
  readonly linearImpulse = new Vec3();
  private angImpulse1 = 0;
  private angImpulse2 = 0;
  private motorImpulse = 0;
  private lowerImpulse = 0;
  private upperImpulse = 0;
  private springImpulse = 0;

  constructor(options: HingeJointOptions) {
    super(options);
    initAnchors(this, options);
    if (options.axis) {
      const axis = new Vec3().copy(options.axis);
      axis.normalize();
      this.bodyA.getLocalVector(axis, this.localAxisA);
    }
    if (options.localAxisA) this.localAxisA.copy(options.localAxisA);
    this.localAxisA.normalize();
    computeBasis(this.localAxisA, this.localPerpA1, this.localPerpA2);
    this.relativeRotation.multiplyConjugateA(this.bodyA.rotation, this.bodyB.rotation);

    this.enableLimit = options.enableLimit ?? false;
    this.lowerAngle = options.lowerAngle ?? 0;
    this.upperAngle = options.upperAngle ?? 0;
    this.enableMotor = options.enableMotor ?? false;
    this.motorSpeed = options.motorSpeed ?? 0;
    this.maxMotorTorque = options.maxMotorTorque ?? 0;
    this.enableSpring = options.enableSpring ?? false;
    this.springHertz = options.springHertz ?? 1;
    this.springDampingRatio = options.springDampingRatio ?? 0.5;
    this.targetAngle = options.targetAngle ?? 0;
  }

  /** 当前铰链角（B 相对 A 绕轴转过的角度，创建时为 0，范围 [-π, π]） */
  getAngle(): number {
    this.bodyA.getWorldVector(this.localAxisA, a);
    qE.multiplyQuats(this.bodyA.rotation, this.relativeRotation).conjugate();
    qE.premultiply(this.bodyB.rotation);
    return swingTwist(qE, a, swingVec);
  }

  /** 当前相对角速度（rad/s） */
  getSpeed(): number {
    this.bodyA.getWorldVector(this.localAxisA, a);
    return tv.subVectors(this.bodyB.angularVelocity, this.bodyA.angularVelocity).dot(a);
  }

  prepare(ctx: SolverContext): void {
    if (!ctx.enableWarmStarting) {
      this.linearImpulse.setZero();
      this.angImpulse1 = this.angImpulse2 = 0;
      this.motorImpulse = this.lowerImpulse = this.upperImpulse = this.springImpulse = 0;
    }
    if (!this.enableLimit) this.lowerImpulse = this.upperImpulse = 0;
    if (!this.enableMotor) this.motorImpulse = 0;
    if (!this.enableSpring) this.springImpulse = 0;
  }

  /**
   * 计算当前铰链轴 a、锁定摆动的两个方向 j1/j2 与摆动误差 swingVec，返回铰链角。
   * 误差旋转 qE = qB (qA qRel)⁻¹ 分解为绕轴的扭转（铰链角）与摆动（需锁定为 0）。
   */
  private computeFrames(): number {
    currentRotation(this.bodyA, qA);
    currentRotation(this.bodyB, qB);
    qA.rotate(this.localAxisA, a);
    qA.rotate(this.localPerpA1, j1);
    qA.rotate(this.localPerpA2, j2);
    qE.multiplyQuats(qA, this.relativeRotation).conjugate();
    qE.premultiply(qB);
    return swingTwist(qE, a, swingVec);
  }

  /** 超出限制时，按圆周上的最近距离选择对应的限制（避免 ±π 处跳变） */
  private unwrapForLimits(angle: number): number {
    const lower = this.lowerAngle;
    const upper = this.upperAngle;
    if (angle < lower && lower - angle > angle + TWO_PI - upper) return angle + TWO_PI;
    if (angle > upper && angle - upper > lower + TWO_PI - angle) return angle - TWO_PI;
    return angle;
  }

  warmStart(): void {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    this.computeFrames();
    currentArm(bodyA, this.localAnchorA, rA);
    currentArm(bodyB, this.localAnchorB, rB);
    applyPointImpulse(bodyA, bodyB, rA, rB, this.linearImpulse);
    const axial = this.motorImpulse + this.lowerImpulse - this.upperImpulse + this.springImpulse;
    L.copy(j1).scale(this.angImpulse1).addScaled(j2, this.angImpulse2).addScaled(a, axial);
    applyAngularImpulse(bodyA, bodyB, L);
  }

  solve(ctx: SolverContext, useBias: boolean): void {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    const rawAngle = this.computeFrames();
    const angle = this.enableLimit ? this.unwrapForLimits(rawAngle) : rawAngle;
    const IA = bodyA.invInertiaWorld;
    const IB = bodyB.invInertiaWorld;
    const axialK = IA.quadraticForm(a) + IB.quadraticForm(a);
    const axialMass = axialK > 0 ? 1 / axialK : 0;
    const wRel = (): number => tv.subVectors(bodyB.angularVelocity, bodyA.angularVelocity).dot(a);

    // ---- 弹簧 ----
    if (this.enableSpring && this.springHertz > 0) {
      spring.set(this.springHertz, this.springDampingRatio, ctx.h);
      let C = rawAngle - this.targetAngle;
      if (C > Math.PI) C -= TWO_PI;
      else if (C < -Math.PI) C += TWO_PI;
      const impulse =
        -spring.massScale * axialMass * (wRel() + spring.biasRate * C) -
        spring.impulseScale * this.springImpulse;
      this.springImpulse += impulse;
      L.copy(a).scale(impulse);
      applyAngularImpulse(bodyA, bodyB, L);
    }

    // ---- 马达 ----
    if (this.enableMotor) {
      const maxImpulse = this.maxMotorTorque * ctx.h;
      const old = this.motorImpulse;
      const impulse = -axialMass * (wRel() - this.motorSpeed);
      this.motorImpulse = Math.max(-maxImpulse, Math.min(old + impulse, maxImpulse));
      L.copy(a).scale(this.motorImpulse - old);
      applyAngularImpulse(bodyA, bodyB, L);
    }

    // ---- 角度限制（推测式） ----
    if (this.enableLimit) {
      const soft = ctx.jointSoftness;
      // 下限
      {
        const C = angle - this.lowerAngle;
        let bias = 0;
        let massScale = 1;
        let impulseScale = 0;
        if (C > 0) bias = C * ctx.invH;
        else if (useBias) {
          bias = soft.biasRate * C;
          massScale = soft.massScale;
          impulseScale = soft.impulseScale;
        }
        let impulse = -axialMass * massScale * (wRel() + bias) - impulseScale * this.lowerImpulse;
        const newImpulse = Math.max(this.lowerImpulse + impulse, 0);
        impulse = newImpulse - this.lowerImpulse;
        this.lowerImpulse = newImpulse;
        L.copy(a).scale(impulse);
        applyAngularImpulse(bodyA, bodyB, L);
      }
      // 上限（方向相反）
      {
        const C = this.upperAngle - angle;
        let bias = 0;
        let massScale = 1;
        let impulseScale = 0;
        if (C > 0) bias = C * ctx.invH;
        else if (useBias) {
          bias = soft.biasRate * C;
          massScale = soft.massScale;
          impulseScale = soft.impulseScale;
        }
        let impulse = -axialMass * massScale * (-wRel() + bias) - impulseScale * this.upperImpulse;
        const newImpulse = Math.max(this.upperImpulse + impulse, 0);
        impulse = newImpulse - this.upperImpulse;
        this.upperImpulse = newImpulse;
        L.copy(a).scale(-impulse);
        applyAngularImpulse(bodyA, bodyB, L);
      }
    }

    // ---- 锁定摆动（绕 j1、j2 的转动，2x2 块求解） ----
    {
      const k11 = IA.quadraticForm(j1) + IB.quadraticForm(j1);
      const k22 = IA.quadraticForm(j2) + IB.quadraticForm(j2);
      const k12 = dotM(IA, j1, j2) + dotM(IB, j1, j2);
      const wr = tv.subVectors(bodyB.angularVelocity, bodyA.angularVelocity);
      let b1 = j1.dot(wr);
      let b2 = j2.dot(wr);
      let massScale = 1;
      let impulseScale = 0;
      if (useBias) {
        const soft = ctx.jointSoftness;
        b1 += soft.biasRate * swingVec.dot(j1);
        b2 += soft.biasRate * swingVec.dot(j2);
        massScale = soft.massScale;
        impulseScale = soft.impulseScale;
      }
      const det = k11 * k22 - k12 * k12;
      if (det > 1e-18) {
        const inv = 1 / det;
        const x1 = (k22 * b1 - k12 * b2) * inv;
        const x2 = (k11 * b2 - k12 * b1) * inv;
        const i1 = -massScale * x1 - impulseScale * this.angImpulse1;
        const i2 = -massScale * x2 - impulseScale * this.angImpulse2;
        this.angImpulse1 += i1;
        this.angImpulse2 += i2;
        L.copy(j1).scale(i1).addScaled(j2, i2);
        applyAngularImpulse(bodyA, bodyB, L);
      }
    }

    // ---- 点约束 ----
    solvePointConstraint(this, this.localAnchorA, this.localAnchorB, this.linearImpulse, ctx, useBias);
  }

  getAnchorA(out: Vec3): Vec3 {
    return this.bodyA.getWorldPoint(this.localAnchorA, out);
  }

  getAnchorB(out: Vec3): Vec3 {
    return this.bodyB.getWorldPoint(this.localAnchorB, out);
  }

  /** 世界坐标铰链轴（当前） */
  getAxis(out: Vec3): Vec3 {
    return currentDirection(this.bodyA, this.localAxisA, out);
  }
}

const tmpDot = new Vec3();
function dotM(m: { transformVector(v: Vec3, out: Vec3): Vec3 }, u: Vec3, v: Vec3): number {
  return u.dot(m.transformVector(v, tmpDot));
}
