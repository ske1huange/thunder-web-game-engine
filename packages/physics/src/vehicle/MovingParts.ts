import { HingeJoint } from '../dynamics/joints/HingeJoint';
import { SliderJoint } from '../dynamics/joints/SliderJoint';
import type { RigidBody } from '../dynamics/RigidBody';
import type { Vec3 } from '../math/Vec3';
import type { World, WorldController } from '../world/World';

/** 可动部件的状态 */
export type PartState = 'closed' | 'opening' | 'open' | 'closing';

/** 判定“到位”的角度容差（弧度） */
const ANGLE_EPS = 0.03;
/** 判定“到位”的距离容差（米） */
const DIST_EPS = 0.004;

export interface HingedPartOptions {
  /** 部件名称（例如 'door_fl'） */
  name: string;
  /** 车身（或其他父刚体） */
  chassis: RigidBody;
  /** 部件刚体（动态） */
  body: RigidBody;
  /** 世界坐标铰链点 */
  anchor: Readonly<Vec3>;
  /** 世界坐标铰链轴 */
  axis: Readonly<Vec3>;
  /** 打开角度（弧度，按右手定则绕 axis 的有符号角，创建时的姿态为 0 = 关闭） */
  openAngle: number;
  /**
   * 打开后的行为：
   * - 'free'：在 [关闭, 打开] 之间自由摆动（车门：加速、刹车、转弯时会甩动，甩回关闭位置会重新锁上）
   * - 'hold'：马达把部件顶在打开位置（引擎盖、后备箱，相当于气弹簧 / 撑杆）
   */
  holdOpen?: 'free' | 'hold';
  /** 开合角速度（rad/s），默认 1.6 */
  speed?: number;
  /** 马达最大力矩（N·m），默认 部件质量 × 20 */
  maxTorque?: number;
  /** 关闭后是否锁止（默认 true；false 时关闭位置也只是限制） */
  latch?: boolean;
}

/**
 * 铰链部件（车门、引擎盖、后备箱、翻盖大灯等），完全由物理驱动：
 * 关闭时铰链角度被锁止；打开 / 关闭由铰链马达推动，受力矩上限约束，会被障碍物挡住；
 * 自由模式下打开的部件随车身运动甩动，甩回关闭位置时重新锁上。
 */
export class HingedPart {
  readonly name: string;
  readonly body: RigidBody;
  readonly joint: HingeJoint;
  readonly openAngle: number;
  holdOpen: 'free' | 'hold';
  speed: number;
  maxTorque: number;
  latch: boolean;
  private _state: PartState = 'closed';

  constructor(world: World, options: HingedPartOptions) {
    this.name = options.name;
    this.body = options.body;
    this.openAngle = options.openAngle;
    this.holdOpen = options.holdOpen ?? 'free';
    this.speed = options.speed ?? 1.6;
    this.maxTorque = options.maxTorque ?? options.body.mass * 20;
    this.latch = options.latch ?? true;
    this.joint = world.addJoint(
      new HingeJoint({
        bodyA: options.chassis,
        bodyB: options.body,
        anchor: options.anchor,
        axis: options.axis,
        enableLimit: true,
        lowerAngle: 0,
        upperAngle: 0,
      }),
    );
    this.applyState();
  }

  get state(): PartState {
    return this._state;
  }

  /** 当前角度（弧度，0 为关闭） */
  get angle(): number {
    return this.joint.getAngle();
  }

  /** 打开程度 0..1 */
  get openness(): number {
    return Math.min(1, Math.max(0, this.angle / this.openAngle));
  }

  open(): void {
    if (this._state === 'open' || this._state === 'opening') return;
    this.setState('opening');
  }

  close(): void {
    if (this._state === 'closed' || this._state === 'closing') return;
    this.setState('closing');
  }

  toggle(): void {
    if (this._state === 'closed' || this._state === 'closing') this.open();
    else this.close();
  }

  /** @internal 每步之前调用：检测到位并切换状态 */
  update(): void {
    const angle = this.angle;
    const toOpen = Math.abs(angle - this.openAngle);
    switch (this._state) {
      case 'opening':
        if (toOpen < ANGLE_EPS) this.setState('open');
        break;
      case 'closing':
        if (Math.abs(angle) < ANGLE_EPS * 0.5) this.setState('closed');
        break;
      case 'open':
        // 自由摆动的部件被甩回关闭位置：重新锁上（像车门被关上）
        if (this.holdOpen === 'free' && this.latch && Math.abs(angle) < ANGLE_EPS * 0.3) {
          this.setState('closed');
        }
        break;
    }
  }

  private setState(state: PartState): void {
    this._state = state;
    this.applyState();
    this.joint.wakeBodies();
  }

  /** 按状态配置铰链的限制与马达 */
  private applyState(): void {
    const j = this.joint;
    const lo = Math.min(0, this.openAngle);
    const hi = Math.max(0, this.openAngle);
    const dir = Math.sign(this.openAngle) || 1;
    j.enableLimit = true;
    j.maxMotorTorque = this.maxTorque;
    switch (this._state) {
      case 'closed':
        j.lowerAngle = this.latch ? 0 : lo;
        j.upperAngle = this.latch ? 0 : hi;
        j.enableMotor = false;
        break;
      case 'opening':
        j.lowerAngle = lo;
        j.upperAngle = hi;
        j.enableMotor = true;
        j.motorSpeed = dir * this.speed;
        break;
      case 'open':
        j.lowerAngle = lo;
        j.upperAngle = hi;
        j.enableMotor = this.holdOpen === 'hold';
        j.motorSpeed = dir * this.speed;
        break;
      case 'closing':
        j.lowerAngle = lo;
        j.upperAngle = hi;
        j.enableMotor = true;
        j.motorSpeed = -dir * this.speed;
        break;
    }
  }
}

export interface SlidingPartOptions {
  name: string;
  chassis: RigidBody;
  body: RigidBody;
  /** 世界坐标锚点（一般取部件中心） */
  anchor: Readonly<Vec3>;
  /** 世界坐标滑动方向 */
  axis: Readonly<Vec3>;
  /** 打开时沿 axis 的位移（米，可为负） */
  travel: number;
  /** 开合速度（m/s），默认 0.3 */
  speed?: number;
  /** 马达最大推力（N），默认 部件质量 × 60 */
  maxForce?: number;
}

/**
 * 滑动部件（天窗、升降尾翼、滑门等）：滑动关节 + 直线马达，
 * 关闭时锁止，打开后马达把部件顶在打开位置。
 */
export class SlidingPart {
  readonly name: string;
  readonly body: RigidBody;
  readonly joint: SliderJoint;
  readonly travel: number;
  speed: number;
  maxForce: number;
  private _state: PartState = 'closed';

  constructor(world: World, options: SlidingPartOptions) {
    this.name = options.name;
    this.body = options.body;
    this.travel = options.travel;
    this.speed = options.speed ?? 0.3;
    this.maxForce = options.maxForce ?? options.body.mass * 60;
    this.joint = world.addJoint(
      new SliderJoint({
        bodyA: options.chassis,
        bodyB: options.body,
        anchor: options.anchor,
        axis: options.axis,
        enableLimit: true,
        lowerTranslation: 0,
        upperTranslation: 0,
      }),
    );
    this.applyState();
  }

  get state(): PartState {
    return this._state;
  }

  /** 当前位移（米，0 为关闭） */
  get position(): number {
    return this.joint.getTranslation();
  }

  get openness(): number {
    return Math.min(1, Math.max(0, this.position / this.travel));
  }

  open(): void {
    if (this._state === 'open' || this._state === 'opening') return;
    this.setState('opening');
  }

  close(): void {
    if (this._state === 'closed' || this._state === 'closing') return;
    this.setState('closing');
  }

  toggle(): void {
    if (this._state === 'closed' || this._state === 'closing') this.open();
    else this.close();
  }

  /** @internal */
  update(): void {
    const x = this.position;
    if (this._state === 'opening' && Math.abs(x - this.travel) < DIST_EPS) this.setState('open');
    else if (this._state === 'closing' && Math.abs(x) < DIST_EPS) this.setState('closed');
  }

  private setState(state: PartState): void {
    this._state = state;
    this.applyState();
    this.joint.wakeBodies();
  }

  private applyState(): void {
    const j = this.joint;
    const lo = Math.min(0, this.travel);
    const hi = Math.max(0, this.travel);
    const dir = Math.sign(this.travel) || 1;
    j.enableLimit = true;
    j.maxMotorForce = this.maxForce;
    if (this._state === 'closed') {
      j.lowerTranslation = 0;
      j.upperTranslation = 0;
      j.enableMotor = false;
    } else {
      j.lowerTranslation = lo;
      j.upperTranslation = hi;
      j.enableMotor = true;
      j.motorSpeed = (this._state === 'closing' ? -dir : dir) * this.speed;
    }
  }
}

export interface WiperPartOptions {
  name: string;
  chassis: RigidBody;
  body: RigidBody;
  anchor: Readonly<Vec3>;
  axis: Readonly<Vec3>;
  /** 摆动角度（弧度，有符号） */
  sweep: number;
  /** 摆动角速度（rad/s），默认 3 */
  speed?: number;
  /** 马达最大力矩（N·m），默认 部件质量 × 30 */
  maxTorque?: number;
}

/**
 * 雨刮：铰链马达在 [0, sweep] 之间往复；关闭时回到 0（停放位置）并锁止。
 */
export class WiperPart {
  readonly name: string;
  readonly body: RigidBody;
  readonly joint: HingeJoint;
  readonly sweep: number;
  speed: number;
  maxTorque: number;
  private running = false;
  private parking = false;
  private direction = 1;

  constructor(world: World, options: WiperPartOptions) {
    this.name = options.name;
    this.body = options.body;
    this.sweep = options.sweep;
    this.speed = options.speed ?? 3;
    this.maxTorque = options.maxTorque ?? options.body.mass * 30;
    this.joint = world.addJoint(
      new HingeJoint({
        bodyA: options.chassis,
        bodyB: options.body,
        anchor: options.anchor,
        axis: options.axis,
        enableLimit: true,
        lowerAngle: 0,
        upperAngle: 0,
        maxMotorTorque: this.maxTorque,
      }),
    );
  }

  get isRunning(): boolean {
    return this.running;
  }

  get angle(): number {
    return this.joint.getAngle();
  }

  start(): void {
    if (this.running) return;
    this.running = true;
    this.parking = false;
    this.direction = 1;
    this.configure();
  }

  stop(): void {
    if (!this.running) return;
    this.running = false;
    this.parking = true;
    this.configure();
  }

  toggle(): void {
    if (this.running) this.stop();
    else this.start();
  }

  /** @internal */
  update(): void {
    if (!this.running && !this.parking) return;
    const t = this.angle / this.sweep; // 0..1
    if (this.running) {
      if (this.direction > 0 && t > 0.97) {
        this.direction = -1;
        this.configure();
      } else if (this.direction < 0 && t < 0.03) {
        this.direction = 1;
        this.configure();
      }
    } else if (t < 0.02) {
      this.parking = false;
      this.configure();
    }
  }

  private configure(): void {
    const j = this.joint;
    const lo = Math.min(0, this.sweep);
    const hi = Math.max(0, this.sweep);
    const sign = Math.sign(this.sweep) || 1;
    j.maxMotorTorque = this.maxTorque;
    if (this.running || this.parking) {
      j.lowerAngle = lo;
      j.upperAngle = hi;
      j.enableMotor = true;
      j.motorSpeed = (this.running ? this.direction : -1) * sign * this.speed;
    } else {
      j.lowerAngle = 0;
      j.upperAngle = 0;
      j.enableMotor = false;
    }
    j.wakeBodies();
  }
}

export type MovingPart = HingedPart | SlidingPart | WiperPart;

/**
 * 管理一组可动部件：注册为世界控制器，每步之前更新各部件的状态。
 */
export class VehicleParts implements WorldController {
  readonly parts = new Map<string, MovingPart>();

  constructor(readonly world: World) {
    world.addController(this);
  }

  addHinged(options: HingedPartOptions): HingedPart {
    return this.add(new HingedPart(this.world, options));
  }

  addSliding(options: SlidingPartOptions): SlidingPart {
    return this.add(new SlidingPart(this.world, options));
  }

  addWiper(options: WiperPartOptions): WiperPart {
    return this.add(new WiperPart(this.world, options));
  }

  get(name: string): MovingPart | undefined {
    return this.parts.get(name);
  }

  /** 切换部件（车门等开合、雨刮启停） */
  toggle(name: string): void {
    this.parts.get(name)?.toggle();
  }

  /** @internal WorldController */
  preStep(): void {
    for (const p of this.parts.values()) p.update();
  }

  /** 移除控制器与所有部件的关节（部件刚体由调用方销毁） */
  destroy(): void {
    this.world.removeController(this);
    for (const p of this.parts.values()) {
      if (p.joint.world) this.world.removeJoint(p.joint);
    }
    this.parts.clear();
  }

  private add<T extends MovingPart>(part: T): T {
    if (this.parts.has(part.name)) throw new Error(`部件名称重复：${part.name}`);
    this.parts.set(part.name, part);
    return part;
  }
}
