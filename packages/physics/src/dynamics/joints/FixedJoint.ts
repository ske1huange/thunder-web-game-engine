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
  rotationError,
} from './JointUtils';

export interface FixedJointOptions extends JointOptions {
  /** 世界坐标锚点，默认为 bodyB 的原点 */
  anchor?: Readonly<Vec3>;
  localAnchorA?: Readonly<Vec3>;
  localAnchorB?: Readonly<Vec3>;
}

const rA = new Vec3();
const rB = new Vec3();
const qA = new Quat();
const qB = new Quat();
const err = new Vec3();
const cdot = new Vec3();
const impulse = new Vec3();
const K = new Mat3();

/** 焊接关节：锁定两刚体之间的全部 6 个相对自由度 */
export class FixedJoint extends Joint {
  readonly localAnchorA = new Vec3();
  readonly localAnchorB = new Vec3();
  /** 目标相对旋转 qA⁻¹ qB */
  readonly relativeRotation = new Quat();
  readonly linearImpulse = new Vec3();
  readonly angularImpulse = new Vec3();

  constructor(options: FixedJointOptions) {
    super(options);
    initAnchors(this, { ...options, anchor: options.anchor ?? options.bodyB.position });
    this.relativeRotation.multiplyConjugateA(this.bodyA.rotation, this.bodyB.rotation);
  }

  prepare(ctx: SolverContext): void {
    if (!ctx.enableWarmStarting) {
      this.linearImpulse.setZero();
      this.angularImpulse.setZero();
    }
  }

  warmStart(): void {
    currentArm(this.bodyA, this.localAnchorA, rA);
    currentArm(this.bodyB, this.localAnchorB, rB);
    applyPointImpulse(this.bodyA, this.bodyB, rA, rB, this.linearImpulse);
    applyAngularImpulse(this.bodyA, this.bodyB, this.angularImpulse);
  }

  solve(ctx: SolverContext, useBias: boolean): void {
    solveAngularLock(this, this.relativeRotation, this.angularImpulse, ctx, useBias);
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

/** 锁定全部相对转动（3x3 块求解），供焊接/滑动关节复用 */
export function solveAngularLock(
  joint: Joint,
  relativeRotation: Readonly<Quat>,
  accumulated: Vec3,
  ctx: SolverContext,
  useBias: boolean,
): void {
  const bodyA = joint.bodyA;
  const bodyB = joint.bodyB;
  cdot.subVectors(bodyB.angularVelocity, bodyA.angularVelocity);
  let massScale = 1;
  let impulseScale = 0;
  if (useBias) {
    currentRotation(bodyA, qA);
    currentRotation(bodyB, qB);
    rotationError(qA, qB, relativeRotation, err);
    const soft = ctx.jointSoftness;
    cdot.addScaled(err, soft.biasRate);
    massScale = soft.massScale;
    impulseScale = soft.impulseScale;
  }
  angularMassMatrix(bodyA, bodyB, K);
  K.solve(cdot, impulse);
  impulse.scale(-massScale).addScaled(accumulated, -impulseScale);
  accumulated.add(impulse);
  applyAngularImpulse(bodyA, bodyB, impulse);
}
