import type { DebugDrawer, RigidBody, World } from '@thunder/physics';
import type { GradingPresetName, LightRig, LightingPresetName, PhysicsView } from '@thunder/render';

export interface DemoContext {
  world: World;
  /** 静态地面刚体（带平面碰撞体），也可作为 MouseJoint 的 bodyA */
  ground: RigidBody;
  view: PhysicsView;
  /** 光源系统：demo 可添加点光源 / 聚光灯并绑定到刚体（切换 demo 时自动清除） */
  lights: LightRig;
  /** 每帧清空的叠加线段层（例如射线可视化） */
  overlay: DebugDrawer;
  /** 在左下角显示提示信息 */
  setInfo(text: string): void;
}

/** 每个固定步之前调用 */
export type DemoUpdate = (dt: number, time: number) => void;

export interface DemoHooks {
  /** 每个固定步之前调用（可修改世界） */
  update?: DemoUpdate;
  /** 每帧渲染前调用（可向 overlay 画线） */
  render?: () => void;
}

export interface Demo {
  id: string;
  name: string;
  description: string;
  camera?: { position: [number, number, number]; target: [number, number, number] };
  /** 为 false 时不创建默认地面 */
  ground?: boolean;
  /** 默认光照预设（studio） */
  lighting?: LightingPresetName;
  /** 默认调色预设（neutral） */
  grading?: GradingPresetName;
  setup(ctx: DemoContext): DemoHooks | DemoUpdate | void;
}
