import type { Collider } from '../dynamics/Collider';
import type { RigidBody } from '../dynamics/RigidBody';
import { Quat } from '../math/Quat';
import { Vec3 } from '../math/Vec3';
import type { QueryFilter } from '../query/Queries';
import type { World, WorldController } from '../world/World';

export interface WheelOptions {
  /** 悬挂连接点（车身局部坐标，相对刚体原点） */
  position: Readonly<Vec3>;
  /** 车轮半径，默认 0.4 */
  radius?: number;
  /** 悬挂自然长度，默认 0.3 */
  suspensionRestLength?: number;
  /** 悬挂最大伸缩行程（相对自然长度），默认 0.2 */
  maxSuspensionTravel?: number;
  /** 悬挂刚度（每单位车身质量，1/s²），默认 30 */
  suspensionStiffness?: number;
  /** 压缩阻尼（每单位车身质量），默认 4.4 */
  dampingCompression?: number;
  /** 回弹阻尼（每单位车身质量），默认 2.3 */
  dampingRelaxation?: number;
  /** 轮胎抓地系数：摩擦冲量上限 = 悬挂力 × frictionSlip，默认 2 */
  frictionSlip?: number;
  /** 侧向摩擦刚度，默认 1 */
  sideFrictionStiffness?: number;
  /** 侧向力作用点向质心高度靠拢的程度（0 = 作用在质心高度，不产生侧倾；1 = 作用在接地点），默认 0.05 */
  rollInfluence?: number;
  /** 悬挂力上限（N），默认不限 */
  maxSuspensionForce?: number;
}

/** 单个车轮：参数、驾驶输入与每步更新的接地状态 */
export class Wheel {
  readonly localPosition: Vec3;
  radius: number;
  suspensionRestLength: number;
  maxSuspensionTravel: number;
  suspensionStiffness: number;
  dampingCompression: number;
  dampingRelaxation: number;
  frictionSlip: number;
  sideFrictionStiffness: number;
  rollInfluence: number;
  maxSuspensionForce: number;

  /** 转向角（弧度，绕车身 up 轴，正值向左） */
  steering = 0;
  /** 驱动力（N），正值向前 */
  engineForce = 0;
  /** 制动力（N） */
  brake = 0;

  /** 是否接地 */
  isInContact = false;
  /** 当前悬挂长度 */
  suspensionLength: number;
  /** 接地点与地面法线（世界坐标） */
  readonly contactPoint = new Vec3();
  readonly contactNormal = new Vec3();
  groundBody: RigidBody | null = null;
  groundCollider: Collider | null = null;
  /** 本步悬挂力（N） */
  suspensionForce = 0;
  /** 本步侧向 / 纵向冲量 */
  sideImpulse = 0;
  forwardImpulse = 0;
  /** 抓地情况：1 为完全抓地，小于 1 表示打滑 */
  skidInfo = 1;
  /** 滚动角（弧度，累计） */
  rotation = 0;
  /** 本步滚动角增量 */
  deltaRotation = 0;

  /** @internal */
  prevSuspensionLength: number;
  /** @internal */
  prevRotation = 0;
  /** @internal 世界坐标的连接点、悬挂方向（向下）、车轴、前进方向 */
  readonly connectionWorld = new Vec3();
  readonly directionWorld = new Vec3();
  readonly axleWorld = new Vec3();
  readonly forwardWorld = new Vec3();
  /** @internal */
  suspensionRelativeVelocity = 0;
  /** @internal */
  clippedInvContactDotSuspension = 1;

  constructor(options: WheelOptions) {
    this.localPosition = new Vec3().copy(options.position);
    this.radius = options.radius ?? 0.4;
    this.suspensionRestLength = options.suspensionRestLength ?? 0.3;
    this.maxSuspensionTravel = options.maxSuspensionTravel ?? 0.2;
    this.suspensionStiffness = options.suspensionStiffness ?? 30;
    this.dampingCompression = options.dampingCompression ?? 4.4;
    this.dampingRelaxation = options.dampingRelaxation ?? 2.3;
    this.frictionSlip = options.frictionSlip ?? 2;
    this.sideFrictionStiffness = options.sideFrictionStiffness ?? 1;
    this.rollInfluence = options.rollInfluence ?? 0.05;
    this.maxSuspensionForce = options.maxSuspensionForce ?? Infinity;
    this.suspensionLength = this.suspensionRestLength;
    this.prevSuspensionLength = this.suspensionLength;
  }
}

export interface RaycastVehicleOptions {
  /** 车身刚体（动态） */
  chassis: RigidBody;
  /** 车身局部坐标系中的前进方向，默认 (0, 0, -1) */
  forward?: Readonly<Vec3>;
  /** 车身局部坐标系中的向上方向，默认 (0, 1, 0) */
  up?: Readonly<Vec3>;
  /** 车轮射线的过滤条件（车身总是被排除） */
  filter?: QueryFilter;
  /** 是否注册为世界控制器、在每步前自动更新，默认 true（否则需自行在 step 前调用 update） */
  autoUpdate?: boolean;
}

const tmpV = new Vec3();
const tmpV2 = new Vec3();
const tmpV3 = new Vec3();
const relPos = new Vec3();
const impulse = new Vec3();
const steerQ = new Quat();
const spinQ = new Quat();
const chassisPos = new Vec3();
const chassisRot = new Quat();
const upWorld = new Vec3();

/**
 * 射线车辆（参考 Bullet btRaycastVehicle / cannon-es RaycastVehicle）。
 *
 * 车身是普通的动态刚体，每个车轮是一条向下的射线：
 * - 悬挂：弹簧 + 压缩 / 回弹阻尼，力作用在接地点
 * - 轮胎：侧向摩擦冲量抵消横向滑移，纵向为驱动力或制动；两者合力超过抓地上限时打滑
 * - 侧向力的作用点可向质心高度靠拢（rollInfluence），减小侧翻倾向
 * - 地面是动态刚体时，反作用冲量作用在地面刚体上
 *
 * ```ts
 * const car = new RaycastVehicle(world, { chassis });
 * car.addWheel({ position: new Vec3(-0.8, 0, -1.3) }); // 左前 ...
 * car.setSteering(0.4, 0); car.applyEngineForce(2000, 2); car.setBrake(0, 2);
 * ```
 */
export class RaycastVehicle implements WorldController {
  readonly world: World;
  readonly chassis: RigidBody;
  readonly wheels: Wheel[] = [];
  readonly localForward = new Vec3(0, 0, -1);
  readonly localUp = new Vec3(0, 1, 0);
  readonly localRight = new Vec3();
  filter: QueryFilter | undefined;
  private readonly rayFilter: QueryFilter;

  constructor(world: World, options: RaycastVehicleOptions) {
    this.world = world;
    this.chassis = options.chassis;
    if (options.forward) this.localForward.copy(options.forward);
    if (options.up) this.localUp.copy(options.up);
    this.localForward.normalize();
    this.localUp.normalize();
    this.localRight.crossVectors(this.localForward, this.localUp);
    this.localRight.normalize();
    this.filter = options.filter;
    const chassis = this.chassis;
    this.rayFilter = {
      includeSensors: false,
      excludeBody: chassis,
      predicate: (c) => {
        if (c.body === chassis) return false;
        const user = this.filter;
        if (!user) return true;
        if (user.excludeBody && c.body === user.excludeBody) return false;
        const cat = user.categoryBits ?? 0x0001;
        const mask = user.maskBits ?? 0xffffffff;
        if ((c.filter.categoryBits & mask) === 0 || (c.filter.maskBits & cat) === 0) return false;
        return user.predicate ? user.predicate(c) : true;
      },
    };
    if (options.autoUpdate ?? true) world.addController(this);
  }

  addWheel(options: WheelOptions): Wheel {
    const wheel = new Wheel(options);
    this.wheels.push(wheel);
    return wheel;
  }

  setSteering(angle: number, wheelIndex: number): void {
    this.wheels[wheelIndex]!.steering = angle;
  }

  applyEngineForce(force: number, wheelIndex: number): void {
    this.wheels[wheelIndex]!.engineForce = force;
    if (force !== 0) this.chassis.wakeUp();
  }

  setBrake(brake: number, wheelIndex: number): void {
    this.wheels[wheelIndex]!.brake = brake;
  }

  /** 沿车头方向的速度（m/s），后退为负 */
  get forwardSpeed(): number {
    this.chassis.getWorldVector(this.localForward, tmpV);
    return tmpV.dot(this.chassis.linearVelocity);
  }

  /** 接地的车轮数 */
  get wheelsInContact(): number {
    let n = 0;
    for (const w of this.wheels) if (w.isInContact) n++;
    return n;
  }

  /** 从世界中移除（不再自动更新） */
  destroy(): void {
    this.world.removeController(this);
  }

  /** @internal WorldController */
  preStep(dt: number): void {
    this.update(dt);
  }

  /** 更新悬挂与轮胎并向车身施加冲量（autoUpdate 为 false 时在 world.step 之前调用） */
  update(dt: number): void {
    const chassis = this.chassis;
    if (!chassis.world || dt <= 0) return;
    for (const w of this.wheels) {
      w.prevSuspensionLength = w.suspensionLength;
      w.prevRotation = w.rotation;
    }
    if (!chassis.isAwake) return;

    for (const w of this.wheels) this.castRay(w);
    this.updateSuspension();
    this.updateFriction(dt);

    // 车轮滚动（渲染用）
    for (const w of this.wheels) {
      if (w.isInContact) {
        chassis.getVelocityAtPoint(w.contactPoint, tmpV);
        w.deltaRotation = (w.forwardWorld.dot(tmpV) * dt) / w.radius;
      } else {
        w.deltaRotation *= 0.99;
      }
      w.rotation += w.deltaRotation;
    }
  }

  /** 更新车轮的世界坐标框架并向下做射线检测 */
  private castRay(w: Wheel): void {
    const chassis = this.chassis;
    const xf = chassis.transform;
    xf.transformPoint(w.localPosition, w.connectionWorld);
    tmpV.copy(this.localUp).negate();
    xf.transformVector(tmpV, w.directionWorld);
    steerQ.setFromAxisAngle(this.localUp, w.steering);
    steerQ.rotate(this.localRight, tmpV);
    xf.transformVector(tmpV, w.axleWorld);
    steerQ.rotate(this.localForward, tmpV);
    xf.transformVector(tmpV, w.forwardWorld);

    const maxLen = w.suspensionRestLength + w.maxSuspensionTravel;
    const minLen = Math.max(0, w.suspensionRestLength - w.maxSuspensionTravel);
    const hit = this.world.raycast(
      w.connectionWorld,
      w.directionWorld,
      maxLen + w.radius,
      this.rayFilter,
    );
    if (!hit) {
      w.isInContact = false;
      w.groundBody = null;
      w.groundCollider = null;
      w.suspensionLength = maxLen;
      w.suspensionRelativeVelocity = 0;
      w.clippedInvContactDotSuspension = 1;
      w.contactNormal.copy(w.directionWorld).negate();
      w.contactPoint.copy(w.connectionWorld).addScaled(w.directionWorld, maxLen + w.radius);
      w.suspensionForce = 0;
      return;
    }
    w.isInContact = true;
    w.groundBody = hit.body;
    w.groundCollider = hit.collider;
    w.contactPoint.copy(hit.point);
    w.contactNormal.copy(hit.normal);
    w.suspensionLength = Math.min(maxLen, Math.max(minLen, hit.distance - w.radius));
    const denominator = w.contactNormal.dot(w.directionWorld);
    // 接地点处车身相对地面的速度
    chassis.getVelocityAtPoint(w.contactPoint, tmpV);
    if (!hit.body.isStatic()) tmpV.sub(hit.body.getVelocityAtPoint(w.contactPoint, tmpV2));
    const projVel = w.contactNormal.dot(tmpV);
    if (denominator >= -0.1) {
      w.suspensionRelativeVelocity = 0;
      w.clippedInvContactDotSuspension = 10;
    } else {
      const inv = -1 / denominator;
      w.suspensionRelativeVelocity = projVel * inv;
      w.clippedInvContactDotSuspension = inv;
    }
  }

  /**
   * 在车身上施加力（持续整个步长，随子步积分），地面是动态刚体时施加反作用力。
   * 以力而不是步首冲量的形式施加，静止时车身速度为 0，可以正常休眠。
   */
  private applyWheelForce(w: Wheel, force: Readonly<Vec3>, point: Readonly<Vec3>): void {
    this.chassis.applyForce(force, point, false);
    const ground = w.groundBody;
    if (ground && ground.isDynamic()) {
      tmpV2.copy(force).negate();
      // 车在动时唤醒脚下的动态刚体
      const moving = this.chassis.linearVelocity.lengthSq() > 0.01;
      ground.applyForce(tmpV2, w.contactPoint, moving);
    }
  }

  /** 悬挂：弹簧 + 阻尼，力作用在接地点 */
  private updateSuspension(): void {
    const chassis = this.chassis;
    const mass = chassis.mass;
    for (const w of this.wheels) {
      if (!w.isInContact) {
        w.suspensionForce = 0;
        continue;
      }
      let force =
        w.suspensionStiffness *
        (w.suspensionRestLength - w.suspensionLength) *
        w.clippedInvContactDotSuspension;
      const damping = w.suspensionRelativeVelocity < 0 ? w.dampingCompression : w.dampingRelaxation;
      force -= damping * w.suspensionRelativeVelocity;
      w.suspensionForce = Math.min(Math.max(force * mass, 0), w.maxSuspensionForce);
      impulse.copy(w.contactNormal).scale(w.suspensionForce);
      this.applyWheelForce(w, impulse, w.contactPoint);
    }
  }

  /** 轮胎摩擦：侧向冲量抵消横滑，纵向为驱动 / 制动，超过抓地上限时按比例打滑 */
  private updateFriction(dt: number): void {
    const chassis = this.chassis;
    const wheels = this.wheels;
    const onGround = Math.max(1, this.wheelsInContact);
    let sliding = false;
    for (const w of wheels) {
      w.sideImpulse = 0;
      w.forwardImpulse = 0;
      w.skidInfo = 1;
      if (!w.isInContact) continue;
      // 车轴投影到地面平面，前进方向 = 法线 × 车轴
      const n = w.contactNormal;
      w.axleWorld.addScaled(n, -w.axleWorld.dot(n));
      w.axleWorld.normalize();
      w.forwardWorld.crossVectors(n, w.axleWorld);
      w.forwardWorld.normalize();
      const ground = w.groundBody!;
      w.sideImpulse =
        bilateralImpulse(chassis, ground, w.contactPoint, w.axleWorld) * w.sideFrictionStiffness;

      // 纵向：驱动力，或者制动（冲量把纵向相对速度降为 0，按制动力截断）
      if (w.engineForce !== 0) {
        w.forwardImpulse = w.engineForce * dt;
      } else if (w.brake > 0) {
        const maxImpulse = w.brake * dt;
        // 各轮平分“刹停”所需的冲量（与 Bullet calcRollingFriction 相同）
        const j = bilateralImpulse(chassis, ground, w.contactPoint, w.forwardWorld, 1) / onGround;
        w.forwardImpulse = Math.max(-maxImpulse, Math.min(j, maxImpulse));
      }

      // 摩擦圆：纵向与侧向合力不超过 悬挂力 × frictionSlip
      const maxImp = w.suspensionForce * dt * w.frictionSlip;
      const x = w.forwardImpulse * 0.5;
      const y = w.sideImpulse;
      const sq = x * x + y * y;
      if (sq > maxImp * maxImp) {
        sliding = true;
        w.skidInfo = maxImp / Math.sqrt(sq);
      }
    }
    if (sliding) {
      for (const w of wheels) {
        if (w.skidInfo < 1) {
          w.forwardImpulse *= w.skidInfo;
          w.sideImpulse *= w.skidInfo;
        }
      }
    }

    // 施加（冲量 / dt 作为整步持续的力）
    chassis.getWorldVector(this.localUp, upWorld);
    const invDt = 1 / dt;
    for (const w of wheels) {
      if (!w.isInContact) continue;
      if (w.forwardImpulse !== 0) {
        impulse.copy(w.forwardWorld).scale(w.forwardImpulse * invDt);
        this.applyWheelForce(w, impulse, w.contactPoint);
      }
      if (w.sideImpulse !== 0) {
        impulse.copy(w.axleWorld).scale(w.sideImpulse * invDt);
        // 侧向力作用点向质心高度靠拢，减小侧倾
        relPos.subVectors(w.contactPoint, chassis.center);
        relPos.addScaled(upWorld, -upWorld.dot(relPos) * (1 - w.rollInfluence));
        tmpV3.addVectors(chassis.center, relPos);
        this.chassis.applyForce(impulse, tmpV3, false);
        const ground = w.groundBody;
        if (ground && ground.isDynamic()) {
          ground.applyForce(impulse.negate(), w.contactPoint, false);
        }
      }
    }
  }

  /**
   * 车轮在渲染插值系数 alpha 下的世界位姿（车轮中心与朝向，车轮局部 X 轴为车轴）。
   * spin 为 false 时不含滚动（刹车卡钳、挡泥板等随转向与悬挂运动但不转的部件）。
   */
  getWheelTransform(
    index: number,
    outPosition: Vec3,
    outRotation: Quat,
    alpha = 1,
    spin = true,
  ): void {
    const w = this.wheels[index]!;
    this.chassis.interpolate(alpha, chassisPos, chassisRot);
    const len = w.prevSuspensionLength + (w.suspensionLength - w.prevSuspensionLength) * alpha;
    chassisRot.rotate(w.localPosition, outPosition).add(chassisPos);
    tmpV.copy(this.localUp).negate();
    chassisRot.rotate(tmpV, tmpV);
    outPosition.addScaled(tmpV, len);
    const rot = spin ? w.prevRotation + (w.rotation - w.prevRotation) * alpha : 0;
    steerQ.setFromAxisAngle(this.localUp, w.steering);
    // 前进时车轮绕车轴（right）负方向滚动
    spinQ.setFromAxisAngle(this.localRight, -rot);
    outRotation.multiplyQuats(chassisRot, steerQ).multiply(spinQ);
  }
}

const bv1 = new Vec3();
const bv2 = new Vec3();
const br1 = new Vec3();
const br2 = new Vec3();
const bc = new Vec3();

/**
 * 在接触点沿方向 dir 消除两刚体相对速度所需的冲量（作用在 A 上，B 受反作用）。
 * damping 为 Bullet resolveSingleBilateral 中的软化系数（侧向摩擦用 0.2）。
 */
function bilateralImpulse(
  a: RigidBody,
  b: RigidBody,
  point: Readonly<Vec3>,
  dir: Readonly<Vec3>,
  damping = 0.2,
): number {
  a.getVelocityAtPoint(point, bv1);
  if (!b.isStatic()) bv1.sub(b.getVelocityAtPoint(point, bv2));
  const relVel = dir.dot(bv1);
  let k = a.invMass + b.invMass;
  br1.subVectors(point, a.center).cross(dir);
  k += a.invInertiaWorld.transformVector(br1, bc).dot(br1);
  if (b.invMass > 0) {
    br2.subVectors(point, b.center).cross(dir);
    k += b.invInertiaWorld.transformVector(br2, bc).dot(br2);
  }
  return k > 0 ? (-damping * relVel) / k : 0;
}
