import type { Vec3 } from '../../math/Vec3';
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

/**
 * 关节基类。
 *
 * 与接触一样采用“软步进”求解：锚点存储在刚体局部坐标系（相对刚体原点），
 * 子步内由步起始位姿 + 位移/相对旋转得到当前位姿；位置误差通过软约束偏置修正，
 * relax 阶段（useBias = false）只做速度约束。
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
  /** @internal 每个子步开始时施加累积冲量 */
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
}
