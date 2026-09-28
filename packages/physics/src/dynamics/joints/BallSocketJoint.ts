import { Mat3 } from '../../math/Mat3';
import { Vec3 } from '../../math/Vec3';
import type { SolverContext } from '../solver/SolverContext';
import { Joint, type JointOptions } from './Joint';
import {
  applyPointImpulse,
  currentArm,
  pointMassMatrix,
  pointVelocity,
} from './JointUtils';

export interface BallSocketJointOptions extends JointOptions {
  /** 世界坐标锚点（与 localAnchorA/B 二选一） */
  anchor?: Readonly<Vec3>;
  /** 相对 bodyA 原点的局部锚点 */
  localAnchorA?: Readonly<Vec3>;
  /** 相对 bodyB 原点的局部锚点 */
  localAnchorB?: Readonly<Vec3>;
}

const rA = new Vec3();
const rB = new Vec3();
const cdot = new Vec3();
const C = new Vec3();
const impulse = new Vec3();
const K = new Mat3();

/**
 * 球窝关节（点约束）：两个刚体上的锚点重合，允许任意相对旋转。
 * 常用于链条、布娃娃的肩部等。
 */
export class BallSocketJoint extends Joint {
  readonly localAnchorA = new Vec3();
  readonly localAnchorB = new Vec3();
  /** 累积冲量 */
  readonly impulse = new Vec3();

  constructor(options: BallSocketJointOptions) {
    super(options);
    initAnchors(this, options);
  }

  prepare(ctx: SolverContext): void {
    if (!ctx.enableWarmStarting) this.impulse.setZero();
  }

  warmStart(): void {
    currentArm(this.bodyA, this.localAnchorA, rA);
    currentArm(this.bodyB, this.localAnchorB, rB);
    applyPointImpulse(this.bodyA, this.bodyB, rA, rB, this.impulse);
  }

  solve(ctx: SolverContext, useBias: boolean): void {
    solvePointConstraint(this, this.localAnchorA, this.localAnchorB, this.impulse, ctx, useBias);
  }

  getAnchorA(out: Vec3): Vec3 {
    return this.bodyA.getWorldPoint(this.localAnchorA, out);
  }

  getAnchorB(out: Vec3): Vec3 {
    return this.bodyB.getWorldPoint(this.localAnchorB, out);
  }
}

/** 由世界锚点或局部锚点初始化 */
export function initAnchors(
  joint: Joint & { localAnchorA: Vec3; localAnchorB: Vec3 },
  options: { anchor?: Readonly<Vec3>; localAnchorA?: Readonly<Vec3>; localAnchorB?: Readonly<Vec3> },
): void {
  if (options.anchor) {
    joint.bodyA.getLocalPoint(options.anchor, joint.localAnchorA);
    joint.bodyB.getLocalPoint(options.anchor, joint.localAnchorB);
  }
  if (options.localAnchorA) joint.localAnchorA.copy(options.localAnchorA);
  if (options.localAnchorB) joint.localAnchorB.copy(options.localAnchorB);
}

/** 求解三维点约束（锚点重合），供多个关节复用 */
export function solvePointConstraint(
  joint: Joint,
  localAnchorA: Readonly<Vec3>,
  localAnchorB: Readonly<Vec3>,
  accumulated: Vec3,
  ctx: SolverContext,
  useBias: boolean,
): void {
  const bodyA = joint.bodyA;
  const bodyB = joint.bodyB;
  currentArm(bodyA, localAnchorA, rA);
  currentArm(bodyB, localAnchorB, rB);
  pointVelocity(bodyA, bodyB, rA, rB, cdot);

  let massScale = 1;
  let impulseScale = 0;
  if (useBias) {
    // C = (cB + rB) - (cA + rA)
    C.subVectors(bodyB.center0, bodyA.center0)
      .add(bodyB.deltaPosition)
      .sub(bodyA.deltaPosition)
      .add(rB)
      .sub(rA);
    const soft = ctx.jointSoftness;
    cdot.addScaled(C, soft.biasRate);
    massScale = soft.massScale;
    impulseScale = soft.impulseScale;
  }
  pointMassMatrix(bodyA, bodyB, rA, rB, K);
  K.solve(cdot, impulse);
  impulse.scale(-massScale).addScaled(accumulated, -impulseScale);
  accumulated.add(impulse);
  applyPointImpulse(bodyA, bodyB, rA, rB, impulse);
}
