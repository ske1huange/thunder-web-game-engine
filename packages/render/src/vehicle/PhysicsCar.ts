import {
  BoxShape,
  type HingedPart,
  type MovingPart,
  Quat,
  RaycastVehicle,
  type RigidBody,
  type Shape,
  type SlidingPart,
  Vec3,
  VehicleParts,
  type WiperPart,
  type World,
  type WorldController,
} from '@thunder/physics';
import * as THREE from 'three';
import type { LightRig } from '../lighting/LightRig';
import { collectTriangles, convexHullFromPositions } from '../physics/colliders';

type Vec3Tuple = readonly [number, number, number];
/** 车身局部方向 */
export type CarDirection = 'up' | 'down' | 'forward' | 'back' | 'left' | 'right';

export interface CarWheelBinding {
  /** 车轮节点名（车轮网格的中心作为轮心，允许节点原点不在轮心） */
  node: string;
  /** 刹车卡钳节点：随转向与悬挂运动，但不随车轮滚动 */
  caliper?: string;
  /** 是否转向，默认前半部分的车轮 */
  steer?: boolean;
  /** 是否驱动，默认后半部分的车轮 */
  drive?: boolean;
  /** 车轮半径，默认按网格包围盒计算 */
  radius?: number;
}

export interface CarHingeBinding {
  node: string;
  /** 部件名称（open / close / toggle 使用），默认节点名 */
  name?: string;
  /**
   * 铰链所在的边（车身坐标）：
   * front / rear —— 车门（前沿）、引擎盖（后沿）、后备箱（前沿）；top / bottom —— 鸥翼门（上沿）
   */
  edge: 'front' | 'rear' | 'top' | 'bottom';
  /**
   * 铰链轴，默认：front/rear 边上的竖直侧面板（车门）为 up，水平面板（引擎盖、后备箱）为 right，
   * top/bottom 边为 forward。剪刀门可用 front + right。
   */
  axis?: 'up' | 'right' | 'forward';
  /** 打开角度（度）；方向自动取“向外 / 向上” */
  angle: number;
  /** 打开后自由摆动（默认车门）或顶在打开位置（默认引擎盖、后备箱、鸥翼门） */
  holdOpen?: 'free' | 'hold';
  /** 部件质量（kg），默认车门 25、其他 15 */
  mass?: number;
  /** 开合角速度（rad/s） */
  speed?: number;
}

export interface CarSliderBinding {
  node: string;
  name?: string;
  /** 打开时的移动方向（车身坐标） */
  direction: CarDirection | Vec3Tuple;
  /** 打开时的位移（米） */
  travel: number;
  mass?: number;
  speed?: number;
  /** 车速超过该值（m/s）时自动打开（例如主动尾翼） */
  autoAbove?: number;
  /** 车速低于该值（m/s）时自动关闭，默认 autoAbove × 0.6 */
  autoBelow?: number;
}

export interface CarWiperBinding {
  node: string;
  name?: string;
  /** 雨刮的转轴在刮片的哪一端（车身坐标的左 / 右） */
  pivot: 'left' | 'right';
  /** 摆动角度（度），默认 80 */
  sweep?: number;
  /** 转轴方向（车身坐标），默认挡风玻璃法线的近似值 0.85·up + 0.5·forward */
  axis?: Vec3Tuple;
  mass?: number;
  speed?: number;
}

export interface CarLightsBinding {
  /** 大灯（开启时自发光；传入 rig 时每个大灯还有一盏随车运动的聚光灯） */
  head?: string[];
  /** 刹车灯（大灯开启时微亮作为尾灯，刹车时高亮） */
  brake?: string[];
  /** 倒车灯 */
  reverse?: string[];
  rig?: LightRig;
  /** 大灯聚光灯强度，默认 80 */
  headlightIntensity?: number;
}

export interface PhysicsCarOptions {
  /** 模型中车头方向（车身坐标），默认 [0, 0, -1]；glTF 车模常为 [0, 0, 1] */
  forward?: Vec3Tuple;
  /** 模型中向上方向，默认 [0, 1, 0] */
  up?: Vec3Tuple;
  /** 车身质量（kg），默认 1200 */
  mass?: number;
  /** 质心高度（车身坐标沿 up），默认车轮中心高度 + 0.1 m，越低越不易侧翻 */
  centerOfMassHeight?: number;
  /** 车身与部件的摩擦系数，默认 0.5 */
  friction?: number;
  wheels: CarWheelBinding[];
  suspension?: {
    restLength?: number;
    travel?: number;
    /** 每单位车身质量的刚度，默认 35 */
    stiffness?: number;
    dampingCompression?: number;
    dampingRelaxation?: number;
    frictionSlip?: number;
    rollInfluence?: number;
  };
  /** 驱动轮总驱动力（N），默认 5200 */
  engineForce?: number;
  /** 总制动力（N），默认 9000 */
  brakeForce?: number;
  /** 低速时的最大转向角（弧度），默认 0.55，车速越高越小 */
  maxSteer?: number;
  hinges?: CarHingeBinding[];
  sliders?: CarSliderBinding[];
  wipers?: CarWiperBinding[];
  /** 方向盘：绕自身法线转动，转角 = 前轮转向角 × ratio（默认 8） */
  steeringWheel?: { node: string; ratio?: number };
  lights?: CarLightsBinding;
  /** 碰撞体凸包的最大顶点数，默认 48 */
  maxHullPoints?: number;
}

/** 驾驶输入 */
export interface CarInput {
  /** 油门：正为前进，负为倒车（高速前进时按负值相当于刹车） */
  throttle: number;
  /** 转向：正为向左，范围 -1..1 */
  steer: number;
  /** 额外的制动 0..1 */
  brake?: number;
  /** 手刹（后轮） */
  handbrake?: boolean;
}

interface Anchor {
  object: THREE.Group;
  body: RigidBody;
}

interface WheelVisual {
  object: THREE.Group;
  caliper: THREE.Group | null;
}

interface LightMaterial {
  material: THREE.MeshStandardMaterial;
  emissive: THREE.Color;
}

const DIRS: Record<
  CarDirection,
  (f: THREE.Vector3, u: THREE.Vector3, r: THREE.Vector3) => THREE.Vector3
> = {
  up: (_f, u) => u.clone(),
  down: (_f, u) => u.clone().negate(),
  forward: (f) => f.clone(),
  back: (f) => f.clone().negate(),
  right: (_f, _u, r) => r.clone(),
  left: (_f, _u, r) => r.clone().negate(),
};

let carCounter = 0;
const tmpP = new Vec3();
const tmpQ = new Quat();
const tmpV3 = new THREE.Vector3();
const tmpQuat = new THREE.Quaternion();

/** 在车身坐标（F / U / R 轴）上的投影范围 */
interface Extent {
  /** 车身坐标中的中心 */
  center: THREE.Vector3;
  min: [number, number, number];
  max: [number, number, number];
  /** 车身坐标顶点（相对车身原点） */
  positions: Float32Array;
}

/**
 * 由车模（three.js 对象，例如 gltf.scene）创建完全由物理驱动的汽车：
 *
 * - 车身：刚体（凸包碰撞体，质心降低），姿态由射线车辆的悬挂决定（加速抬头、刹车点头、转弯侧倾）
 * - 车轮：由悬挂长度、转向角与滚动角驱动；刹车卡钳随转向与悬挂但不滚动；方向盘随转向转动
 * - 车门 / 引擎盖 / 后备箱：独立刚体 + 铰链（锁止、马达开合、自由摆动或保持打开），可被撞、会被甩
 * - 天窗 / 尾翼：滑动关节 + 马达（尾翼可按车速自动升起）；雨刮：往复的铰链马达
 * - 车灯：大灯（可带聚光灯）、刹车灯、倒车灯随驾驶状态变化
 *
 * 部件按节点名称绑定，铰链轴心由部件网格在车身坐标中的包围盒自动推算。
 * 创建后把 `car.root` 加入场景（模型会被移入其中），每帧调用 `car.sync(alpha)`。
 */
export class PhysicsCar implements WorldController {
  readonly world: World;
  /** 加入场景的根节点（世界坐标） */
  readonly root = new THREE.Group();
  readonly chassis: RigidBody;
  readonly vehicle: RaycastVehicle;
  readonly parts: VehicleParts;
  /** 车身与所有部件的刚体（例如在 PhysicsView 中隐藏它们） */
  readonly bodies: RigidBody[] = [];
  /** 本车所有碰撞体共用的碰撞组（负数：彼此不碰撞） */
  readonly collisionGroup: number;
  readonly input: Required<CarInput> = { throttle: 0, steer: 0, brake: 0, handbrake: false };
  /** 当前前轮转向角（弧度） */
  steering = 0;
  headlights = false;
  /** 本步是否在刹车 / 倒车（车灯使用） */
  braking = false;
  reversing = false;

  private readonly anchors: Anchor[] = [];
  private readonly wheelVisuals: WheelVisual[] = [];
  private readonly steerWheels: number[] = [];
  private readonly driveWheels: number[] = [];
  private readonly rearWheels: number[] = [];
  /** 按车速自动开合的滑动部件（只在越过阈值时触发，手动切换不会被覆盖） */
  private readonly autoSliders: {
    part: SlidingPart;
    above: number;
    below: number;
    fast: boolean;
  }[] = [];
  private steeringWheel: {
    pivot: THREE.Group;
    base: THREE.Quaternion;
    axis: THREE.Vector3;
    ratio: number;
  } | null = null;
  private readonly headMaterials: LightMaterial[] = [];
  private readonly brakeMaterials: LightMaterial[] = [];
  private readonly reverseMaterials: LightMaterial[] = [];
  private readonly spotLights: THREE.SpotLight[] = [];
  private readonly lightRig: LightRig | undefined;
  private readonly engineForce: number;
  private readonly brakeForce: number;
  private readonly maxSteer: number;

  constructor(world: World, model: THREE.Object3D, options: PhysicsCarOptions) {
    this.world = world;
    this.collisionGroup = -(1000 + carCounter++);
    this.engineForce = options.engineForce ?? 5200;
    this.brakeForce = options.brakeForce ?? 9000;
    this.maxSteer = options.maxSteer ?? 0.55;
    this.lightRig = options.lights?.rig;
    const friction = options.friction ?? 0.5;
    const maxHull = options.maxHullPoints ?? 48;
    const group = this.collisionGroup;

    // ---- 车身坐标系：模型的世界位置与朝向（不含缩放） ----
    model.updateWorldMatrix(true, true);
    const carPos = new THREE.Vector3();
    const carQuat = new THREE.Quaternion();
    model.matrixWorld.decompose(carPos, carQuat, new THREE.Vector3());
    const frame = new THREE.Object3D();
    frame.matrixWorld.compose(carPos, carQuat, new THREE.Vector3(1, 1, 1));
    const F = new THREE.Vector3(...(options.forward ?? [0, 0, -1])).normalize();
    const U = new THREE.Vector3(...(options.up ?? [0, 1, 0])).normalize();
    const R = new THREE.Vector3().crossVectors(F, U).normalize();
    const toWorld = (local: THREE.Vector3): Vec3 => {
      const w = local.clone().applyQuaternion(carQuat).add(carPos);
      return new Vec3(w.x, w.y, w.z);
    };
    const toWorldDir = (local: THREE.Vector3): Vec3 => {
      const w = local.clone().applyQuaternion(carQuat);
      return new Vec3(w.x, w.y, w.z);
    };
    const find = (name: string): THREE.Object3D => {
      const o = model.getObjectByName(name);
      if (!o) throw new Error(`PhysicsCar：模型中找不到节点 "${name}"`);
      return o;
    };
    const extentOf = (object: THREE.Object3D, filter?: (m: THREE.Mesh) => boolean): Extent => {
      const { positions } = collectTriangles(object, { relativeTo: frame, filter });
      if (positions.length < 3) throw new Error(`PhysicsCar：节点 "${object.name}" 没有网格`);
      const min: [number, number, number] = [Infinity, Infinity, Infinity];
      const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
      const p = new THREE.Vector3();
      for (let i = 0; i < positions.length; i += 3) {
        p.set(positions[i]!, positions[i + 1]!, positions[i + 2]!);
        const proj = [p.dot(F), p.dot(U), p.dot(R)];
        for (let k = 0; k < 3; k++) {
          if (proj[k]! < min[k]!) min[k] = proj[k]!;
          if (proj[k]! > max[k]!) max[k] = proj[k]!;
        }
      }
      const center = new THREE.Vector3()
        .addScaledVector(F, (min[0] + max[0]) / 2)
        .addScaledVector(U, (min[1] + max[1]) / 2)
        .addScaledVector(R, (min[2] + max[2]) / 2);
      return { center, min, max, positions };
    };
    /** 由车身坐标顶点生成碰撞形状（相对 center），退化时用包围盒 */
    const shapeOf = (e: Extent): Shape => {
      const local = new Float32Array(e.positions.length);
      for (let i = 0; i < local.length; i += 3) {
        local[i] = e.positions[i]! - e.center.x;
        local[i + 1] = e.positions[i + 1]! - e.center.y;
        local[i + 2] = e.positions[i + 2]! - e.center.z;
      }
      try {
        return convexHullFromPositions(local, maxHull);
      } catch {
        const h = e.max.map((mx, k) => Math.max((mx - e.min[k]!) / 2, 0.01));
        // 包围盒轴沿 F/U/R，这里假设车身坐标轴与 F/U/R 对齐（常见的 ±X/±Y/±Z）
        const hx = Math.abs(F.x) * h[0]! + Math.abs(U.x) * h[1]! + Math.abs(R.x) * h[2]!;
        const hy = Math.abs(F.y) * h[0]! + Math.abs(U.y) * h[1]! + Math.abs(R.y) * h[2]!;
        const hz = Math.abs(F.z) * h[0]! + Math.abs(U.z) * h[1]! + Math.abs(R.z) * h[2]!;
        return new BoxShape(new Vec3(hx, hy, hz));
      }
    };

    // ---- 收集要单独驱动的节点，剩下的都属于车身 ----
    const detached = new Set<THREE.Object3D>();
    for (const w of options.wheels) {
      detached.add(find(w.node));
      if (w.caliper) detached.add(find(w.caliper));
    }
    for (const h of options.hinges ?? []) detached.add(find(h.node));
    for (const s of options.sliders ?? []) detached.add(find(s.node));
    for (const w of options.wipers ?? []) detached.add(find(w.node));
    const isDetached = (o: THREE.Object3D): boolean => {
      for (let p: THREE.Object3D | null = o; p; p = p.parent) if (detached.has(p)) return true;
      return false;
    };
    const bodyExtent = extentOf(model, (m) => !isDetached(m));

    // ---- 车身刚体 ----
    const chassis = world.createBody({
      position: toWorld(new THREE.Vector3()),
      rotation: new Quat(carQuat.x, carQuat.y, carQuat.z, carQuat.w),
      mass: options.mass ?? 1200,
      angularDamping: 0.1,
    });
    const bodyShape = shapeOf(bodyExtent);
    chassis.addCollider({
      shape: bodyShape,
      position: new Vec3(bodyExtent.center.x, bodyExtent.center.y, bodyExtent.center.z),
      friction,
      filter: { group },
    });
    this.chassis = chassis;
    this.bodies.push(chassis);

    // ---- 车轮 ----
    const susp = options.suspension ?? {};
    const restLength = susp.restLength ?? 0.3;
    const stiffness = susp.stiffness ?? 35;
    const wheelExtents = options.wheels.map((w) => extentOf(find(w.node)));
    const avgWheelHeight =
      wheelExtents.reduce((s, e) => s + e.center.dot(U), 0) / Math.max(1, wheelExtents.length);
    // 降低质心
    const comHeight = options.centerOfMassHeight ?? avgWheelHeight + 0.1;
    const lc = chassis.localCenter;
    const lcU = lc.x * U.x + lc.y * U.y + lc.z * U.z;
    chassis.setCenterOfMassOffset(new Vec3(U.x, U.y, U.z).scale(comHeight - lcU));

    this.vehicle = new RaycastVehicle(world, {
      chassis,
      forward: new Vec3(F.x, F.y, F.z),
      up: new Vec3(U.x, U.y, U.z),
      filter: { predicate: (c) => c.filter.group !== group },
    });
    // 静态压缩量：让车停稳时车轮正好位于模型中的位置
    const compression = 9.81 / (Math.max(1, options.wheels.length) * stiffness);
    options.wheels.forEach((w, i) => {
      const e = wheelExtents[i]!;
      const radius = w.radius ?? Math.max(e.max[0] - e.min[0], e.max[1] - e.min[1]) / 2;
      const connection = e.center.clone().addScaledVector(U, restLength - compression);
      this.vehicle.addWheel({
        position: new Vec3(connection.x, connection.y, connection.z),
        radius,
        suspensionRestLength: restLength,
        maxSuspensionTravel: susp.travel ?? 0.2,
        suspensionStiffness: stiffness,
        dampingCompression: susp.dampingCompression ?? 4.5,
        dampingRelaxation: susp.dampingRelaxation ?? 3,
        frictionSlip: susp.frictionSlip ?? 2.5,
        rollInfluence: susp.rollInfluence ?? 0.1,
      });
      const front = e.center.dot(F) > 0;
      if (w.steer ?? front) this.steerWheels.push(i);
      if (w.drive ?? !front) this.driveWheels.push(i);
      if (!front) this.rearWheels.push(i);
    });

    // ---- 部件 ----
    this.parts = new VehicleParts(world);
    const parts: { part: MovingPart; node: THREE.Object3D; center: THREE.Vector3 }[] = [];
    const createPartBody = (e: Extent, mass: number): RigidBody => {
      const body = world.createBody({
        position: toWorld(e.center),
        rotation: new Quat(carQuat.x, carQuat.y, carQuat.z, carQuat.w),
        mass,
        angularDamping: 0.3,
        linearDamping: 0.05,
      });
      body.addCollider({ shape: shapeOf(e), friction, filter: { group } });
      this.bodies.push(body);
      return body;
    };
    const axisVec = { up: U, right: R, forward: F };

    for (const h of options.hinges ?? []) {
      const node = find(h.node);
      const e = extentOf(node);
      const ext = [e.max[0] - e.min[0], e.max[1] - e.min[1], e.max[2] - e.min[2]];
      const thin = ext.indexOf(Math.min(...ext));
      const axisName =
        h.axis ??
        (h.edge === 'top' || h.edge === 'bottom' ? 'forward' : thin === 2 ? 'up' : 'right');
      const A = axisVec[axisName];
      // 铰链点：所在边的中点
      const coord = [
        (e.min[0] + e.max[0]) / 2,
        (e.min[1] + e.max[1]) / 2,
        (e.min[2] + e.max[2]) / 2,
      ];
      if (h.edge === 'front') coord[0] = e.max[0];
      else if (h.edge === 'rear') coord[0] = e.min[0];
      else if (h.edge === 'top') coord[1] = e.max[1];
      else coord[1] = e.min[1];
      const P = new THREE.Vector3()
        .addScaledVector(F, coord[0]!)
        .addScaledVector(U, coord[1]!)
        .addScaledVector(R, coord[2]!);
      // 向外的方向：面板最薄的轴，指向远离车身中心的一侧
      const N = [F, U, R][thin]!.clone();
      if (N.dot(tmpV3.subVectors(e.center, bodyExtent.center)) < 0) N.negate();
      const swing = new THREE.Vector3().crossVectors(A, tmpV3.subVectors(e.center, P));
      const sign = swing.dot(N) >= 0 ? 1 : -1;
      const isDoor = axisName === 'up';
      const mass = h.mass ?? (isDoor ? 25 : 15);
      const body = createPartBody(e, mass);
      const part = this.parts.addHinged({
        name: h.name ?? h.node,
        chassis,
        body,
        anchor: toWorld(P),
        axis: toWorldDir(A),
        openAngle: (sign * h.angle * Math.PI) / 180,
        holdOpen: h.holdOpen ?? (isDoor ? 'free' : 'hold'),
        speed: h.speed,
      });
      parts.push({ part, node, center: e.center });
    }

    for (const s of options.sliders ?? []) {
      const node = find(s.node);
      const e = extentOf(node);
      const D =
        typeof s.direction === 'string'
          ? DIRS[s.direction](F, U, R)
          : new THREE.Vector3(...s.direction).normalize();
      const body = createPartBody(e, s.mass ?? 6);
      const part = this.parts.addSliding({
        name: s.name ?? s.node,
        chassis,
        body,
        anchor: toWorld(e.center),
        axis: toWorldDir(D),
        travel: s.travel,
        speed: s.speed,
      });
      if (s.autoAbove !== undefined) {
        this.autoSliders.push({
          part,
          above: s.autoAbove,
          below: s.autoBelow ?? s.autoAbove * 0.6,
          fast: false,
        });
      }
      parts.push({ part, node, center: e.center });
    }

    for (const w of options.wipers ?? []) {
      const node = find(w.node);
      const e = extentOf(node);
      const mid = [(e.min[0] + e.max[0]) / 2, (e.min[1] + e.max[1]) / 2];
      const pivotR = w.pivot === 'left' ? e.min[2] : e.max[2];
      const tipR = w.pivot === 'left' ? e.max[2] : e.min[2];
      const P = new THREE.Vector3()
        .addScaledVector(F, mid[0]!)
        .addScaledVector(U, mid[1]!)
        .addScaledVector(R, pivotR);
      const T = new THREE.Vector3()
        .addScaledVector(F, mid[0]!)
        .addScaledVector(U, mid[1]!)
        .addScaledVector(R, tipR);
      const A = w.axis
        ? new THREE.Vector3(...w.axis).normalize()
        : new THREE.Vector3().addScaledVector(U, 0.85).addScaledVector(F, 0.5).normalize();
      // 摆动方向：刮片末端向上
      const lift = new THREE.Vector3().crossVectors(A, tmpV3.subVectors(T, P)).dot(U);
      const sweep = ((lift >= 0 ? 1 : -1) * (w.sweep ?? 80) * Math.PI) / 180;
      const body = createPartBody(e, w.mass ?? 1.5);
      const part = this.parts.addWiper({
        name: w.name ?? w.node,
        chassis,
        body,
        anchor: toWorld(P),
        axis: toWorldDir(A),
        sweep,
        speed: w.speed,
      });
      parts.push({ part, node, center: e.center });
    }

    // ---- 视觉：模型挂在车身锚点下，部件 / 车轮移入各自的锚点 ----
    const makeAnchor = (localPos: THREE.Vector3): THREE.Group => {
      const g = new THREE.Group();
      g.position.copy(localPos).applyQuaternion(carQuat).add(carPos);
      g.quaternion.copy(carQuat);
      this.root.add(g);
      g.updateMatrixWorld(true);
      return g;
    };
    const chassisAnchor = makeAnchor(new THREE.Vector3());
    chassisAnchor.attach(model);
    this.anchors.push({ object: chassisAnchor, body: chassis });
    this.root.updateMatrixWorld(true);

    for (const { part, node, center } of parts) {
      const a = makeAnchor(center);
      a.attach(node);
      this.anchors.push({ object: a, body: part.body });
    }
    options.wheels.forEach((w, i) => {
      const center = wheelExtents[i]!.center;
      const object = makeAnchor(center);
      object.attach(find(w.node));
      let caliper: THREE.Group | null = null;
      if (w.caliper) {
        caliper = makeAnchor(center);
        caliper.attach(find(w.caliper));
      }
      this.wheelVisuals.push({ object, caliper });
    });

    // ---- 方向盘：绕自身法线（最薄的轴）转动 ----
    if (options.steeringWheel) {
      const node = find(options.steeringWheel.node);
      const { positions } = collectTriangles(node, { relativeTo: node });
      const box = new THREE.Box3().setFromArray(positions);
      const size = box.getSize(new THREE.Vector3());
      const localAxis =
        size.x <= size.y && size.x <= size.z
          ? new THREE.Vector3(1, 0, 0)
          : size.y <= size.z
            ? new THREE.Vector3(0, 1, 0)
            : new THREE.Vector3(0, 0, 1);
      node.updateWorldMatrix(true, false);
      const centerWorld = box.getCenter(new THREE.Vector3()).applyMatrix4(node.matrixWorld);
      const axisWorld = localAxis.transformDirection(node.matrixWorld);
      // 轴指向驾驶员（车尾方向），正转向角使方向盘逆时针转
      if (axisWorld.dot(F.clone().applyQuaternion(carQuat)) > 0) axisWorld.negate();
      const pivot = new THREE.Group();
      pivot.position.copy(centerWorld);
      this.root.add(pivot);
      node.parent!.attach(pivot);
      pivot.updateMatrixWorld(true);
      pivot.attach(node);
      const pivotWorldQuat = pivot.getWorldQuaternion(new THREE.Quaternion());
      const axis = axisWorld.applyQuaternion(pivotWorldQuat.invert()).normalize();
      this.steeringWheel = {
        pivot,
        base: pivot.quaternion.clone(),
        axis,
        ratio: options.steeringWheel.ratio ?? 8,
      };
    }

    // ---- 车灯 ----
    const lights = options.lights ?? {};
    const collectMaterials = (names: string[] | undefined, color: number, out: LightMaterial[]) => {
      for (const name of names ?? []) {
        find(name).traverse((o) => {
          const mesh = o as THREE.Mesh;
          if (!mesh.isMesh) return;
          const src = Array.isArray(mesh.material) ? mesh.material[0] : mesh.material;
          const material =
            src instanceof THREE.MeshStandardMaterial
              ? src.clone()
              : new THREE.MeshStandardMaterial({ color: 0x222222 });
          mesh.material = material;
          out.push({ material, emissive: new THREE.Color(color) });
        });
      }
    };
    collectMaterials(lights.head, 0xfff2d6, this.headMaterials);
    collectMaterials(lights.brake, 0xff1a0a, this.brakeMaterials);
    collectMaterials(lights.reverse, 0xffffff, this.reverseMaterials);
    if (lights.rig) {
      for (const name of lights.head ?? []) {
        const e = extentOf(find(name));
        const light = lights.rig.addSpotLight({
          color: 0xfff2d6,
          intensity: lights.headlightIntensity ?? 80,
          distance: 60,
          angle: 0.5,
          penumbra: 0.6,
        });
        light.visible = false;
        const dir = F.clone().addScaledVector(U, -0.08);
        const offset = e.center.clone().addScaledVector(F, 0.1);
        lights.rig.attachToBody(light, chassis, { offset, direction: dir });
        this.spotLights.push(light);
      }
    }

    world.addController(this);
    this.sync(1);
  }

  /** 设置驾驶输入（每步读取） */
  setInput(input: Partial<CarInput>): void {
    Object.assign(this.input, input);
  }

  /** 部件（车门、引擎盖、天窗、雨刮等） */
  part(name: string): MovingPart | undefined {
    return this.parts.get(name);
  }

  open(name: string): void {
    const p = this.parts.get(name);
    if (p && 'open' in p) p.open();
  }

  close(name: string): void {
    const p = this.parts.get(name);
    if (p && 'close' in p) p.close();
  }

  /** 切换部件：车门类开 / 关，雨刮启动 / 停止 */
  toggle(name: string): void {
    this.parts.toggle(name);
    this.chassis.wakeUp();
  }

  setHeadlights(on: boolean): void {
    this.headlights = on;
  }

  /** 车速（m/s，后退为负） */
  get speed(): number {
    return this.vehicle.forwardSpeed;
  }

  /** @internal WorldController：每步前把驾驶输入转换为车轮的驱动 / 制动 / 转向 */
  preStep(dt: number): void {
    const v = this.vehicle;
    const input = this.input;
    const speed = v.forwardSpeed;
    // 转向：随车速减小最大转角，平滑趋近
    const maxSteer = this.maxSteer / (1 + Math.abs(speed) / 20);
    const target = Math.max(-1, Math.min(1, input.steer)) * maxSteer;
    this.steering += (target - this.steering) * Math.min(1, 6 * dt);
    for (const i of this.steerWheels) v.setSteering(this.steering, i);

    const throttle = Math.max(-1, Math.min(1, input.throttle));
    let engine = throttle * this.engineForce;
    let brake = Math.max(0, Math.min(1, input.brake)) * this.brakeForce;
    // 与行驶方向相反的油门视为刹车
    if ((throttle < 0 && speed > 1) || (throttle > 0 && speed < -1)) {
      brake += Math.abs(throttle) * this.brakeForce;
      engine = 0;
    }
    const perDrive = engine / Math.max(1, this.driveWheels.length);
    const n = v.wheels.length;
    for (let i = 0; i < n; i++) {
      v.applyEngineForce(this.driveWheels.includes(i) ? perDrive : 0, i);
      // 松开油门时的滚动阻力
      v.setBrake(brake > 0 ? brake / n : throttle === 0 ? 120 : 0, i);
    }
    if (input.handbrake) for (const i of this.rearWheels) v.setBrake(this.brakeForce, i);
    this.braking = brake > 0 || input.handbrake;
    this.reversing = throttle < 0 && speed < 0.5;
    if (throttle !== 0 || input.steer !== 0) this.chassis.wakeUp();

    for (const s of this.autoSliders) {
      if (!s.fast && speed > s.above) {
        s.fast = true;
        s.part.open();
      } else if (s.fast && speed < s.below) {
        s.fast = false;
        s.part.close();
      }
    }
  }

  /** 每帧渲染前调用：按插值系数同步车身、部件、车轮、方向盘与车灯 */
  sync(alpha = 1): void {
    for (const a of this.anchors) {
      a.body.interpolate(alpha, tmpP, tmpQ);
      a.object.position.set(tmpP.x, tmpP.y, tmpP.z);
      a.object.quaternion.set(tmpQ.x, tmpQ.y, tmpQ.z, tmpQ.w);
    }
    this.wheelVisuals.forEach((w, i) => {
      this.vehicle.getWheelTransform(i, tmpP, tmpQ, alpha);
      w.object.position.set(tmpP.x, tmpP.y, tmpP.z);
      w.object.quaternion.set(tmpQ.x, tmpQ.y, tmpQ.z, tmpQ.w);
      if (w.caliper) {
        this.vehicle.getWheelTransform(i, tmpP, tmpQ, alpha, false);
        w.caliper.position.set(tmpP.x, tmpP.y, tmpP.z);
        w.caliper.quaternion.set(tmpQ.x, tmpQ.y, tmpQ.z, tmpQ.w);
      }
    });
    const sw = this.steeringWheel;
    if (sw) {
      sw.pivot.quaternion
        .copy(sw.base)
        .multiply(tmpQuat.setFromAxisAngle(sw.axis, this.steering * sw.ratio));
    }
    for (const m of this.headMaterials) {
      m.material.emissive.copy(m.emissive);
      m.material.emissiveIntensity = this.headlights ? 3 : 0;
    }
    for (const m of this.brakeMaterials) {
      m.material.emissive.copy(m.emissive);
      m.material.emissiveIntensity = this.braking ? 4 : this.headlights ? 0.8 : 0;
    }
    for (const m of this.reverseMaterials) {
      m.material.emissive.copy(m.emissive);
      m.material.emissiveIntensity = this.reversing ? 2.5 : 0;
    }
    for (const l of this.spotLights) l.visible = this.headlights;
  }

  /** 从世界中移除（刚体、关节、控制器、光源），并把 root 从场景中移除 */
  destroy(): void {
    this.world.removeController(this);
    this.vehicle.destroy();
    this.parts.destroy();
    for (const b of this.bodies) if (b.world) this.world.destroyBody(b);
    for (const l of this.spotLights) this.lightRig?.removeLight(l);
    this.root.removeFromParent();
  }
}

/** 由车模创建物理驱动的汽车（等同于 new PhysicsCar） */
export function createPhysicsCar(
  world: World,
  model: THREE.Object3D,
  options: PhysicsCarOptions,
): PhysicsCar {
  return new PhysicsCar(world, model, options);
}

/** 部件类型守卫 */
export function isHingedPart(p: MovingPart | undefined): p is HingedPart {
  return !!p && 'openAngle' in p;
}

export function isSlidingPart(p: MovingPart | undefined): p is SlidingPart {
  return !!p && 'travel' in p;
}

export function isWiperPart(p: MovingPart | undefined): p is WiperPart {
  return !!p && 'sweep' in p;
}
