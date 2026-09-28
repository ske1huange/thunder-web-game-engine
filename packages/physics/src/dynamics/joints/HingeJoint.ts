import { Quat } from '../../math/Quat';
import { Vec3 } from '../../math/Vec3';
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
  /** bodyB 局部坐标系中的铰链轴 */
  localAxisB?: Readonly<Vec3>;
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
const p = new Vec3();
const q = new Vec3();
const j1 = new Vec3();
const j2 = new Vec3();
const L = new Vec3();
const refA = new Vec3();
const refB = new Vec3();
const tv = new Vec3();
const qA = new Quat();
const qB = new Quat();
const spring = new Softness();

/**
 * 铰链关节（旋转关节）：锚点重合，两刚体只能绕公共轴相对转动。
 * 支持角度限制、马达与角度弹簧（门、车轮、摆锤等）。
 */
export class HingeJoint extends Joint {
  readonly localAnchorA = new Vec3();
  readonly localAnchorB = new Vec3();
  readonly localAxisA = new Vec3(0, 1, 0);
  readonly localAxisB = new Vec3(0, 1, 0);
  /** 测量角度的参考方向（与轴垂直） */
  private readonly localRefA = new Vec3();
  private readonly localRefB = new Vec3();
  /** B 上与轴垂直的两个方向，用于锁定另外两个转动自由度 */
  private readonly localPerpB1 = new Vec3();
  private readonly localPerpB2 = new Vec3();

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
      this.bodyB.getLocalVector(axis, this.localAxisB);
    }
    if (options.localAxisA) this.localAxisA.copy(options.localAxisA);
    if (options.localAxisB) this.localAxisB.copy(options.localAxisB);
    this.localAxisA.normalize();
    this.localAxisB.normalize();

    // 参考方向：以 A 的轴构造垂线，转换到 B 中，使初始角度为 0
    this.localAxisA.perpendicular(this.localRefA);
    this.bodyA.getWorldVector(this.localRefA, tv);
    this.bodyB.getLocalVector(tv, this.localRefB);
    // 去掉与 B 轴平行的分量
    this.localRefB.addScaled(this.localAxisB, -this.localRefB.dot(this.localAxisB));
    this.localRefB.normalize();
    this.localPerpB1.copy(this.localRefB);
    this.localPerpB2.crossVectors(this.localAxisB, this.localPerpB1);

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

  /** 当前铰链角（B 相对 A 绕轴转过的角度，弧度） */
  getAngle(): number {
    this.bodyA.getWorldVector(this.localAxisA, a);
    this.bodyA.getWorldVector(this.localRefA, refA);
    this.bodyB.getWorldVector(this.localRefB, refB);
    return Math.atan2(tv.crossVectors(refA, refB).dot(a), refA.dot(refB));
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

  private computeFrames(): number {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    currentRotation(bodyA, qA);
    currentRotation(bodyB, qB);
    qA.rotate(this.localAxisA, a);
    qB.rotate(this.localPerpB1, p);
    qB.rotate(this.localPerpB2, q);
    // 约束 C1 = a·p, C2 = a·q；雅可比 J1 = p × a, J2 = q × a
    j1.crossVectors(p, a);
    j2.crossVectors(q, a);
    qA.rotate(this.localRefA, refA);
    qB.rotate(this.localRefB, refB);
    return Math.atan2(tv.crossVectors(refA, refB).dot(a), refA.dot(refB));
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
    const angle = this.computeFrames();
    const IA = bodyA.invInertiaWorld;
    const IB = bodyB.invInertiaWorld;
    const axialK = IA.quadraticForm(a) + IB.quadraticForm(a);
    const axialMass = axialK > 0 ? 1 / axialK : 0;
    const wRel = (): number => tv.subVectors(bodyB.angularVelocity, bodyA.angularVelocity).dot(a);

    // ---- 弹簧 ----
    if (this.enableSpring && this.springHertz > 0) {
      spring.set(this.springHertz, this.springDampingRatio, ctx.h);
      const C = angle - this.targetAngle;
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

    // ---- 锁定另外两个转动自由度（2x2 块求解） ----
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
        b1 += soft.biasRate * a.dot(p);
        b2 += soft.biasRate * a.dot(q);
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
