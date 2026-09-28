import { Quat } from '../../math/Quat';
import { Vec3 } from '../../math/Vec3';
import type { SolverContext } from '../solver/SolverContext';
import { initAnchors } from './BallSocketJoint';
import { solveAngularLock } from './FixedJoint';
import { Joint, type JointOptions } from './Joint';
import {
  applyAngularImpulse,
  currentArm,
  currentDirection,
  rowApply,
  rowMass,
  rowVelocity,
} from './JointUtils';

export interface SliderJointOptions extends JointOptions {
  /** 世界坐标锚点 */
  anchor?: Readonly<Vec3>;
  /** 世界坐标滑动轴 */
  axis?: Readonly<Vec3>;
  localAnchorA?: Readonly<Vec3>;
  localAnchorB?: Readonly<Vec3>;
  /** bodyA 局部坐标系中的滑动轴 */
  localAxisA?: Readonly<Vec3>;
  enableLimit?: boolean;
  lowerTranslation?: number;
  upperTranslation?: number;
  enableMotor?: boolean;
  /** 马达目标速度（m/s） */
  motorSpeed?: number;
  /** 马达最大推力（N） */
  maxMotorForce?: number;
}

const rA = new Vec3();
const rB = new Vec3();
const d = new Vec3();
const a = new Vec3();
const p1 = new Vec3();
const p2 = new Vec3();
const jA = new Vec3();
const jB = new Vec3();
const rAd = new Vec3();
const tv = new Vec3();

/**
 * 滑动关节（棱柱关节）：两刚体不能相对转动，只能沿一条轴相对平移。
 * 支持平移限制与直线马达（升降机、活塞等）。
 */
export class SliderJoint extends Joint {
  readonly localAnchorA = new Vec3();
  readonly localAnchorB = new Vec3();
  readonly localAxisA = new Vec3(1, 0, 0);
  private readonly localPerpA1 = new Vec3();
  private readonly localPerpA2 = new Vec3();
  readonly relativeRotation = new Quat();

  enableLimit: boolean;
  lowerTranslation: number;
  upperTranslation: number;
  enableMotor: boolean;
  motorSpeed: number;
  maxMotorForce: number;

  readonly angularImpulse = new Vec3();
  private perpImpulse1 = 0;
  private perpImpulse2 = 0;
  private motorImpulse = 0;
  private lowerImpulse = 0;
  private upperImpulse = 0;

  constructor(options: SliderJointOptions) {
    super(options);
    initAnchors(this, { ...options, anchor: options.anchor ?? options.bodyB.position });
    if (options.axis) {
      const axis = new Vec3().copy(options.axis);
      axis.normalize();
      this.bodyA.getLocalVector(axis, this.localAxisA);
    }
    if (options.localAxisA) this.localAxisA.copy(options.localAxisA);
    this.localAxisA.normalize();
    this.localAxisA.perpendicular(this.localPerpA1);
    this.localPerpA2.crossVectors(this.localAxisA, this.localPerpA1);
    this.relativeRotation.multiplyConjugateA(this.bodyA.rotation, this.bodyB.rotation);

    this.enableLimit = options.enableLimit ?? false;
    this.lowerTranslation = options.lowerTranslation ?? 0;
    this.upperTranslation = options.upperTranslation ?? 0;
    this.enableMotor = options.enableMotor ?? false;
    this.motorSpeed = options.motorSpeed ?? 0;
    this.maxMotorForce = options.maxMotorForce ?? 0;
  }

  /** 当前沿轴的平移量 */
  getTranslation(): number {
    const pA = this.getAnchorA(rA);
    const pB = this.getAnchorB(rB);
    this.bodyA.getWorldVector(this.localAxisA, a);
    return tv.subVectors(pB, pA).dot(a);
  }

  prepare(ctx: SolverContext): void {
    if (!ctx.enableWarmStarting) {
      this.angularImpulse.setZero();
      this.perpImpulse1 = this.perpImpulse2 = 0;
      this.motorImpulse = this.lowerImpulse = this.upperImpulse = 0;
    }
    if (!this.enableLimit) this.lowerImpulse = this.upperImpulse = 0;
    if (!this.enableMotor) this.motorImpulse = 0;
  }

  /** 计算当前锚点与轴，返回沿轴平移量 */
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
    currentDirection(bodyA, this.localAxisA, a);
    currentDirection(bodyA, this.localPerpA1, p1);
    currentDirection(bodyA, this.localPerpA2, p2);
    rAd.addVectors(rA, d);
    return d.dot(a);
  }

  /** 设置方向 n 的行雅可比：jA = -(rA + d) × n，jB = rB × n */
  private row(n: Vec3): void {
    jA.crossVectors(rAd, n).negate();
    jB.crossVectors(rB, n);
  }

  private applyRow(n: Vec3, lambda: number): void {
    this.row(n);
    rowApply(this.bodyA, this.bodyB, n, jA, jB, lambda);
  }

  warmStart(): void {
    this.frame();
    applyAngularImpulse(this.bodyA, this.bodyB, this.angularImpulse);
    this.applyRow(p1, this.perpImpulse1);
    this.applyRow(p2, this.perpImpulse2);
    this.applyRow(a, this.motorImpulse + this.lowerImpulse - this.upperImpulse);
  }

  solve(ctx: SolverContext, useBias: boolean): void {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    const translation = this.frame();
    const soft = ctx.jointSoftness;

    // ---- 轴向：马达 ----
    this.row(a);
    const axialMass = rowMass(bodyA, bodyB, a, jA, jB);
    if (this.enableMotor) {
      const maxImpulse = this.maxMotorForce * ctx.h;
      const old = this.motorImpulse;
      const cdot = rowVelocity(bodyA, bodyB, a, jA, jB);
      this.motorImpulse = Math.max(
        -maxImpulse,
        Math.min(old + -axialMass * (cdot - this.motorSpeed), maxImpulse),
      );
      rowApply(bodyA, bodyB, a, jA, jB, this.motorImpulse - old);
    }

    // ---- 轴向：限制（推测式） ----
    if (this.enableLimit) {
      for (const lower of [true, false]) {
        const C = lower ? translation - this.lowerTranslation : this.upperTranslation - translation;
        const sign = lower ? 1 : -1;
        let bias = 0;
        let massScale = 1;
        let impulseScale = 0;
        if (C > 0) bias = C * ctx.invH;
        else if (useBias) {
          bias = soft.biasRate * C;
          massScale = soft.massScale;
          impulseScale = soft.impulseScale;
        }
        this.row(a);
        const cdot = sign * rowVelocity(bodyA, bodyB, a, jA, jB);
        const acc = lower ? this.lowerImpulse : this.upperImpulse;
        let impulse = -axialMass * massScale * (cdot + bias) - impulseScale * acc;
        const newImpulse = Math.max(acc + impulse, 0);
        impulse = newImpulse - acc;
        if (lower) this.lowerImpulse = newImpulse;
        else this.upperImpulse = newImpulse;
        rowApply(bodyA, bodyB, a, jA, jB, sign * impulse);
      }
    }

    // ---- 锁定转动 ----
    solveAngularLock(this, this.relativeRotation, this.angularImpulse, ctx, useBias);

    // ---- 垂直方向的两条平移约束 ----
    for (let i = 0; i < 2; i++) {
      this.frame();
      const n = i === 0 ? p1 : p2;
      this.row(n);
      const mass = rowMass(bodyA, bodyB, n, jA, jB);
      let cdot = rowVelocity(bodyA, bodyB, n, jA, jB);
      let massScale = 1;
      let impulseScale = 0;
      if (useBias) {
        cdot += soft.biasRate * d.dot(n);
        massScale = soft.massScale;
        impulseScale = soft.impulseScale;
      }
      const acc = i === 0 ? this.perpImpulse1 : this.perpImpulse2;
      const impulse = -massScale * mass * cdot - impulseScale * acc;
      if (i === 0) this.perpImpulse1 += impulse;
      else this.perpImpulse2 += impulse;
      rowApply(bodyA, bodyB, n, jA, jB, impulse);
    }
  }

  getAnchorA(out: Vec3): Vec3 {
    return this.bodyA.getWorldPoint(this.localAnchorA, out);
  }

  getAnchorB(out: Vec3): Vec3 {
    return this.bodyB.getWorldPoint(this.localAnchorB, out);
  }
}
