import { Softness } from './Softness';

/** 每一步求解时共享的参数 */
export class SolverContext {
  /** 整步时长 */
  dt = 0;
  /** 子步时长 */
  h = 0;
  /** 子步时长倒数 */
  invH = 0;
  subSteps = 4;
  readonly contactSoftness = new Softness();
  readonly staticSoftness = new Softness();
  readonly jointSoftness = new Softness();
  /** 接触推出速度上限（m/s） */
  maxBiasVelocity = 3;
  restitutionThreshold = 1;
  enableWarmStarting = true;
}
