import { Quat } from '../../math/Quat';
import { Vec3 } from '../../math/Vec3';
import type { World } from '../../world/World';
import type { RigidBody } from '../RigidBody';
import type { SolverContext } from '../solver/SolverContext';

export interface JointOptions {
  bodyA: RigidBody;
  bodyB: RigidBody;
  /** 相连的两个刚体之间是否还产生碰撞，默认 false */
  collideConnected?: boolean;
  userData?: unknown;
}

const tmpV = new Vec3();

/**
 * 关节基类。
 *
 * 与接触一样采用“软步进”求解：锚点相对质心存储在刚体局部坐标系，
 * 子步内由步起始位姿 + 位移/相对旋转得到当前位姿，求解位置误差时使用软约束偏置。
 */
export abstract class Joint {
  readonly bodyA: RigidBody;
  readonly bodyB: RigidBody;
  readonly collideConnected: boolean;
  userData: unknown;
  /** @internal */
  world: World | null = null;
  /** @internal */
  id = -1;

  constructor(options: JointOptions) {
    if (options.bodyA === options.bodyB) throw new Error('Joint: bodyA 与 bodyB 不能相同');
    this.bodyA = options.bodyA;
    this.bodyB = options.bodyB;
    this.collideConnected = options.collideConnected ?? false;
    this.userData = options.userData;
  }

  /** @internal 每步开始时调用 */
  abstract prepare(ctx: SolverContext): void;
  /** @internal 每个子步开始时施加上一次的累积冲量 */
  abstract warmStart(ctx: SolverContext): void;
  /** @internal */
  abstract solve(ctx: SolverContext, useBias: boolean): void;

  /** 世界坐标锚点（用于调试绘制） */
  abstract getAnchorA(out: Vec3): Vec3;
  abstract getAnchorB(out: Vec3): Vec3;

  /** 唤醒两端刚体 */
  wakeBodies(): void {
    this.bodyA.wakeUp();
    this.bodyB.wakeUp();
  }

  // ---------------- 工具方法 ----------------

  /** 局部点（相对刚体原点）转为相对质心的局部坐标 */
  protected static localToCenter(body: RigidBody, localPoint: Readonly<Vec3>, out: Vec3): Vec3 {
    return out.subVectors(localPoint, body.localCenter);
  }

  /** 世界点转为相对质心的局部坐标 */
  protected static worldToCenterLocal(body: RigidBody, worldPoint: Readonly<Vec3>, out: Vec3): Vec3 {
    body.transform.inverseTransformPoint(worldPoint, out);
    return out.sub(body.localCenter);
  }

  /** 子步内的当前旋转 */
  protected static currentRotation(body: RigidBody, out: Quat): Quat {
    return out.multiplyQuats(body.deltaRotation, body.rotation0);
  }

  /** 子步内的当前质心 */
  protected static currentCenter(body: RigidBody, out: Vec3): Vec3 {
    return out.addVectors(body.center0, body.deltaPosition);
  }

  /** 世界锚点 = 质心 + R * 局部锚点（相对质心） */
  protected static worldAnchor(body: RigidBody, localAnchor: Readonly<Vec3>, out: Vec3): Vec3 {
    body.transform.rotation.rotate(localAnchor, out);
    return out.add(body.center);
  }

  /** 对两个刚体施加线冲量 P（B 正、A 负）与各自的角冲量 */
  protected static applyLinear(
    bodyA: RigidBody,
    bodyB: RigidBody,
    rA: Readonly<Vec3>,
    rB: Readonly<Vec3>,
    P: Readonly<Vec3>,
  ): void {
    if (bodyA.invMass > 0) {
      bodyA.linearVelocity.addScaled(P, -bodyA.invMass);
      tmpV.crossVectors(rA, P);
      bodyA.invInertiaWorld.transformVector(tmpV, tmpV);
      bodyA.angularVelocity.sub(tmpV);
    }
    if (bodyB.invMass > 0) {
      bodyB.linearVelocity.addScaled(P, bodyB.invMass);
      tmpV.crossVectors(rB, P);
      bodyB.invInertiaWorld.transformVector(tmpV, tmpV);
      bodyB.angularVelocity.add(tmpV);
    }
  }

  /** 施加角冲量 L（B 正、A 负） */
  protected static applyAngular(bodyA: RigidBody, bodyB: RigidBody, L: Readonly<Vec3>): void {
    if (bodyA.invMass > 0) {
      bodyA.invInertiaWorld.transformVector(L, tmpV);
      bodyA.angularVelocity.sub(tmpV);
    }
    if (bodyB.invMass > 0) {
      bodyB.invInertiaWorld.transformVector(L, tmpV);
      bodyB.angularVelocity.add(tmpV);
    }
  }
}

export const tmpJointQuat = new Quat();
