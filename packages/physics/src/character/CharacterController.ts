import { Manifold } from '../collision/Manifold';
import { type CollideConfig, collideShapes } from '../collision/narrowphase/Collide';
import { type ManifoldList, collideMesh } from '../collision/narrowphase/CollideMesh';
import type { Collider } from '../dynamics/Collider';
import type { RigidBody } from '../dynamics/RigidBody';
import { AABB } from '../math/AABB';
import { Quat } from '../math/Quat';
import { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import type { QueryFilter, ShapeCastHit } from '../query/Queries';
import { CapsuleShape } from '../shapes/CapsuleShape';
import type { MeshShape } from '../shapes/MeshShape';
import { type Shape, ShapeType } from '../shapes/Shape';
import type { World } from '../world/World';

export interface CharacterControllerOptions {
  /** 角色形状（凸形状），默认胶囊：半径 0.3、半高 0.6（总高 1.8 m），沿 up 方向竖立 */
  shape?: Shape;
  /** 形状中心的初始位置 */
  position?: Readonly<Vec3>;
  /** 向上方向，默认 (0, 1, 0) */
  up?: Readonly<Vec3>;
  /** 可行走的最大坡度（弧度），默认 45° */
  maxSlopeAngle?: number;
  /** 自动迈上的台阶高度（米），默认 0.3，0 表示关闭 */
  stepHeight?: number;
  /** 贴地距离：下坡 / 下台阶时向下吸附的最大距离（米），默认 0.3 */
  snapDistance?: number;
  /** 与表面保持的间隙（米），默认 0.02 */
  skinWidth?: number;
  /** 滑动迭代次数，默认 5 */
  maxSlideIterations?: number;
  /** 角色质量（kg），用于推动动态刚体与压在动态刚体上的重量，默认 70 */
  mass?: number;
  /** 推动动态刚体的力度系数（0 为不推），默认 1 */
  pushStrength?: number;
  /**
   * 是否创建一个运动学刚体代表角色（默认 true），使动态物体能与角色碰撞。
   * 该刚体在每次 move 后用 moveKinematic 跟随角色。
   */
  createBody?: boolean;
  /** 角色查询使用的过滤条件（角色自己的刚体总是被排除） */
  filter?: QueryFilter;
}

/** 一次 move 中与角色发生的碰撞 */
export interface CharacterHit {
  collider: Collider;
  body: RigidBody;
  /** 世界坐标碰撞点 */
  point: Vec3;
  /** 被碰表面的法线（指向角色） */
  normal: Vec3;
}

const DEFAULT_UP = new Vec3(0, 1, 0);
const Y_AXIS = new Vec3(0, 1, 0);

const tmpXf = new Transform();
const tmpAabb = new AABB();
const tmpV = new Vec3();
const tmpV2 = new Vec3();
const tmpV3 = new Vec3();
const castDir = new Vec3();
const remaining = new Vec3();
const horizontal = new Vec3();
const vertical = new Vec3();
const planeN = new Vec3();
const prevN = new Vec3();
const startPos = new Vec3();
const slidePos = new Vec3();
const carry = new Vec3();
const manifold = new Manifold();
const manifoldList: ManifoldList = { manifolds: [], manifoldCount: 0 };

const enum SlideMode {
  /** 水平移动：陡坡视为墙 */
  Horizontal,
  /** 竖直向下（重力）：落在可行走表面上就停下，不沿缓坡下滑 */
  Down,
  /** 其他（向上、平台携带） */
  Free,
}

/**
 * 角色控制器（运动学角色，参考 Jolt CharacterVirtual、Box3D mover、Rapier KinematicCharacterController）。
 *
 * 角色不参与刚体求解，每次 `move` 用形状投射做“碰撞并滑动”（collide and slide）：
 * - 墙角折线处理、陡坡不可攀爬（按墙处理）、落在缓坡上不下滑
 * - 自动迈上台阶（上 → 前 → 下三段投射）
 * - 贴地：下坡、下台阶时吸附到地面
 * - 穿透恢复：被其他物体挤入时推出
 * - 站在运动的刚体（平台）上时随之移动；推动动态刚体；站在动态刚体上时施加重量
 *
 * ```ts
 * const character = new CharacterController(world, { position: new Vec3(0, 1, 0) });
 * // 每个固定步（world.step 之前）
 * velocity.y = character.isGrounded ? (jump ? 5 : 0) : velocity.y - 9.81 * dt;
 * character.move(tmp.copy(velocity).scale(dt), dt);
 * ```
 */
export class CharacterController {
  readonly world: World;
  readonly shape: Shape;
  readonly up: Vec3;
  /** 形状中心的当前位置 */
  readonly position = new Vec3();
  /** 上一次 move 之前的位置（渲染插值用） */
  readonly previousPosition = new Vec3();
  /** 形状的朝向（让胶囊沿 up 方向竖立） */
  readonly rotation = new Quat();
  /** 上一次 move 实际产生的速度 */
  readonly velocity = new Vec3();
  /** 代表角色的运动学刚体（createBody 为 false 时为 null） */
  readonly body: RigidBody | null;

  maxSlopeAngle: number;
  stepHeight: number;
  snapDistance: number;
  skinWidth: number;
  maxSlideIterations: number;
  mass: number;
  pushStrength: number;
  filter: QueryFilter | undefined;

  /** 是否站在可行走的地面上 */
  isGrounded = false;
  /** 上一次 move 中是否撞到天花板（向上移动被挡住） */
  hitCeiling = false;
  /** 地面法线（未着地时为 up） */
  readonly groundNormal = new Vec3();
  /** 地面接触点 */
  readonly groundPoint = new Vec3();
  groundCollider: Collider | null = null;
  groundBody: RigidBody | null = null;
  /** 上一次 move 中的碰撞（地面除外） */
  readonly hits: CharacterHit[] = [];

  private readonly castXf = new Transform();
  private readonly queryFilter: QueryFilter;

  constructor(world: World, options: CharacterControllerOptions = {}) {
    this.world = world;
    this.shape = options.shape ?? new CapsuleShape(0.3, 0.6);
    this.up = new Vec3().copy(options.up ?? DEFAULT_UP);
    this.up.normalize();
    this.rotation.setFromUnitVectors(Y_AXIS, this.up);
    this.maxSlopeAngle = options.maxSlopeAngle ?? Math.PI / 4;
    this.stepHeight = options.stepHeight ?? 0.3;
    this.snapDistance = options.snapDistance ?? 0.3;
    this.skinWidth = options.skinWidth ?? 0.02;
    this.maxSlideIterations = options.maxSlideIterations ?? 5;
    this.mass = options.mass ?? 70;
    this.pushStrength = options.pushStrength ?? 1;
    this.filter = options.filter;
    if (options.position) this.position.copy(options.position);
    this.previousPosition.copy(this.position);
    this.groundNormal.copy(this.up);
    this.body = null;
    if (options.createBody ?? true) {
      const body = world.createBody({
        type: 'kinematic',
        position: this.position,
        rotation: this.rotation,
        allowSleep: false,
      });
      body.addCollider({ shape: this.shape, friction: 0 });
      this.body = body;
    }
    this.queryFilter = new CharacterQueryFilter(this);
  }

  /** 可行走坡度对应的余弦 */
  get minWalkableCos(): number {
    return Math.cos(this.maxSlopeAngle);
  }

  /** 表面法线是否可行走 */
  isWalkable(normal: Readonly<Vec3>): boolean {
    return normal.dot(this.up) >= this.minWalkableCos - 1e-6;
  }

  /** 瞬移到指定位置（不做碰撞检测） */
  teleport(position: Readonly<Vec3>): void {
    this.position.copy(position);
    this.previousPosition.copy(position);
    this.velocity.setZero();
    this.isGrounded = false;
    this.groundBody = null;
    this.groundCollider = null;
    this.body?.setTransform(position, this.rotation);
  }

  /** 渲染插值：previousPosition → position */
  interpolate(alpha: number, out: Vec3): Vec3 {
    return out.lerpVectors(this.previousPosition, this.position, alpha);
  }

  /** 移除角色的运动学刚体 */
  destroy(): void {
    if (this.body?.world) this.body.world.destroyBody(this.body);
  }

  /**
   * 按期望位移移动角色（一般为 期望速度 × dt，重力由调用方加到速度上）。
   * 应在 world.step 之前、世界未锁定时调用。
   */
  move(displacement: Readonly<Vec3>, dt: number): void {
    this.world.assertUnlocked();
    this.previousPosition.copy(this.position);
    this.hits.length = 0;
    this.hitCeiling = false;
    const up = this.up;
    const wasGrounded = this.isGrounded;

    // 1. 站在运动的刚体上：先随平台移动
    const ground = this.groundBody;
    if (wasGrounded && ground && !ground.isStatic() && dt > 0) {
      ground.getVelocityAtPoint(this.groundPoint, carry).scale(dt);
      if (carry.lengthSq() > 1e-12) this.slide(carry, SlideMode.Free, dt, false);
    }

    // 2. 穿透恢复
    this.depenetrate();

    // 3. 水平与竖直分量
    const vUp = displacement.x * up.x + displacement.y * up.y + displacement.z * up.z;
    vertical.copy(up).scale(vUp);
    horizontal.subVectors(displacement, vertical);
    const movingUp = vUp > 1e-9;

    if (horizontal.lengthSq() > 1e-12) {
      if (wasGrounded && !movingUp && this.stepHeight > 0) this.moveWithSteps(horizontal, dt);
      else this.slide(horizontal, SlideMode.Horizontal, dt, true);
    }
    if (vUp !== 0) {
      this.slide(vertical, movingUp ? SlideMode.Free : SlideMode.Down, dt, true);
    }

    // 4. 地面检测与贴地
    this.updateGround(wasGrounded && !movingUp, movingUp);

    // 5. 站在动态刚体上：施加重量
    const g = this.groundBody;
    if (this.isGrounded && g && g.isDynamic() && this.mass > 0) {
      const gravity = this.world.gravity;
      tmpV.copy(up).scale(-this.mass * Math.max(0, -gravity.dot(up)));
      g.applyForce(tmpV, this.groundPoint);
    }

    if (dt > 0) this.velocity.subVectors(this.position, this.previousPosition).scale(1 / dt);
    // 运动学刚体在下一步中移动到角色位置
    const body = this.body;
    if (body) {
      if (dt > 0) body.moveKinematic(this.position, this.rotation, dt);
      else body.setTransform(this.position, this.rotation);
    }
  }

  // ------------------------------------------------------------------

  private cast(from: Readonly<Vec3>, translation: Readonly<Vec3>): ShapeCastHit | null {
    this.castXf.position.copy(from);
    this.castXf.rotation.copy(this.rotation);
    return this.world.castShape(this.shape, this.castXf, translation, this.queryFilter);
  }

  /**
   * 碰撞并滑动：沿 delta 移动，遇到表面时停在 skinWidth 外，把剩余位移投影到表面上继续。
   */
  private slide(delta: Readonly<Vec3>, mode: SlideMode, dt: number, record: boolean): boolean {
    const up = this.up;
    const skin = this.skinWidth;
    const target = this.world.linearSlop;
    remaining.copy(delta);
    let blocked = false;
    let havePrev = false;
    const targetLen = delta.length();
    for (let iter = 0; iter < this.maxSlideIterations; iter++) {
      const len = remaining.length();
      if (len < 1e-7) break;
      const hit = this.cast(this.position, remaining);
      if (!hit) {
        this.position.add(remaining);
        break;
      }
      castDir.copy(remaining).scale(1 / len);
      const dist = hit.fraction * len;
      const moveDist = Math.max(0, dist - skin);
      this.position.addScaled(castDir, moveDist);

      // 被碰表面的法线（指向角色）
      planeN.copy(hit.normal).negate();
      const walkable = this.isWalkable(planeN);
      // 沿法线保持 skinWidth 的间隙（弯曲的表面上沿切向滑动会逐渐贴近表面）
      const gap = target + (dist - moveDist) * Math.max(0, -castDir.dot(planeN));
      if (gap < skin) this.position.addScaled(planeN, skin - gap);
      if (record) this.recordHit(hit, planeN, castDir, len - moveDist, mode, dt, walkable);

      if (mode === SlideMode.Down && walkable) {
        // 落在可行走的表面上：停下，不沿缓坡下滑
        remaining.setZero();
        break;
      }
      if (mode === SlideMode.Free && planeN.dot(up) < -0.5) this.hitCeiling = true;
      if (mode === SlideMode.Horizontal && !walkable) {
        // 陡坡 / 墙：去掉法线的竖直分量，按竖直的墙处理（不能沿陡坡爬上去）
        const nu = planeN.dot(up);
        if (nu > 0) {
          planeN.addScaled(up, -nu);
          if (planeN.normalize() < 1e-6) break;
        }
        blocked = true;
      }

      remaining.copy(castDir).scale(len - moveDist);
      const into = remaining.dot(planeN);
      if (into < 0) remaining.addScaled(planeN, -into);
      if (mode === SlideMode.Horizontal && walkable) {
        // 可行走的斜坡：沿坡面前进并保持水平速度大小
        const rest = len - moveDist;
        const l = remaining.length();
        if (l > 1e-9) remaining.scale(rest / l);
      }
      // 折线：同时贴着前后两个不同的面时，沿两面的交线移动
      if (havePrev && prevN.dot(planeN) < 0.999 && remaining.dot(prevN) < 0) {
        tmpV.crossVectors(prevN, planeN);
        if (tmpV.normalize() < 1e-6) {
          remaining.setZero();
          break;
        }
        remaining.copy(tmpV).scale(remaining.dot(tmpV));
      }
      prevN.copy(planeN);
      havePrev = true;
      // 不允许比原位移走得更远
      if (remaining.length() > targetLen) remaining.scale(targetLen / remaining.length());
    }
    return blocked;
  }

  /** 水平移动时自动迈上台阶：上 → 前 → 下 三段投射，比直接滑动走得更远则采用 */
  private moveWithSteps(delta: Readonly<Vec3>, dt: number): void {
    const up = this.up;
    const skin = this.skinWidth;
    startPos.copy(this.position);
    const startBottom = this.bottomHeight();
    const hitCount = this.hits.length;
    const blocked = this.slide(delta, SlideMode.Horizontal, dt, true);
    if (!blocked) return;
    const dirLen = delta.length();
    tmpV2.copy(delta).scale(1 / dirLen);
    const progressA = tmpV3.subVectors(this.position, startPos).dot(tmpV2);
    if (progressA >= dirLen - 1e-4) return;
    slidePos.copy(this.position);
    const slideHits = this.hits.splice(hitCount);

    // 向上
    this.position.copy(startPos);
    tmpV.copy(up).scale(this.stepHeight);
    const upHit = this.cast(this.position, tmpV);
    const climb = upHit ? Math.max(0, upHit.fraction * this.stepHeight - skin) : this.stepHeight;
    if (climb > 1e-4) {
      this.position.addScaled(up, climb);
      // 向前
      this.slide(delta, SlideMode.Horizontal, dt, true);
      // 向下落回地面
      const down = climb + 2 * skin;
      tmpV.copy(up).scale(-down);
      const downHit = this.cast(this.position, tmpV);
      const progressB = tmpV3.subVectors(this.position, startPos).dot(tmpV2);
      if (downHit && progressB > progressA + 1e-4) {
        const drop = Math.max(0, downHit.fraction * down - skin);
        const risen = climb - drop;
        planeN.copy(downHit.normal).negate();
        this.position.addScaled(up, -drop);
        if (risen > -1e-4 && this.landingWalkable(downHit, planeN, tmpV2, startBottom)) return;
      }
    }
    // 迈不上去：恢复直接滑动的结果
    this.position.copy(slidePos);
    this.hits.length = hitCount;
    for (const h of slideHits) this.hits.push(h);
  }

  /**
   * 落点是否可站立。落点在台阶边缘时投射得到的法线是倾斜的，
   * 此时沿前进方向稍微前移后竖直向下做射线检测，取台阶顶面的法线；
   * 顶面比 refBottom（角色最低点的参考高度）高出不超过 stepHeight 才算可站立。
   */
  private landingWalkable(
    hit: ShapeCastHit,
    normal: Readonly<Vec3>,
    dir: Readonly<Vec3>,
    refBottom: number,
  ): boolean {
    if (this.isWalkable(normal)) return true;
    const up = this.up;
    tmpV.copy(hit.point).addScaled(dir, 0.02).addScaled(up, this.stepHeight);
    tmpV3.copy(up).negate();
    const ray = this.world.raycast(tmpV, tmpV3, this.stepHeight * 2, this.queryFilter);
    if (!ray || !this.isWalkable(ray.normal)) return false;
    return ray.point.dot(up) <= refBottom + this.stepHeight + this.skinWidth;
  }

  /** 角色形状最低点沿 up 方向的高度 */
  private bottomHeight(): number {
    const up = this.up;
    let min = Infinity;
    for (const p of this.shape.getCorePoints()) {
      this.rotation.rotate(p, tmpV3);
      tmpV3.add(this.position);
      min = Math.min(min, tmpV3.dot(up));
    }
    return min - this.shape.radius;
  }

  /** 向下探测地面；贴地时把角色移到地面上方 skinWidth 处 */
  private updateGround(snap: boolean, movingUp: boolean): void {
    const up = this.up;
    const skin = this.skinWidth;
    const probe = 2 * skin + 0.02;
    const distance = snap ? Math.max(this.snapDistance, probe) : probe;
    tmpV.copy(up).scale(-distance);
    const hit = movingUp ? null : this.cast(this.position, tmpV);
    if (hit) {
      planeN.copy(hit.normal).negate();
      const d = hit.fraction * distance;
      if (
        this.isWalkable(planeN) ||
        this.landingWalkable(hit, planeN, horizontalDir(this), this.bottomHeight())
      ) {
        this.isGrounded = true;
        this.groundNormal.copy(planeN);
        this.groundPoint.copy(hit.point);
        this.groundCollider = hit.collider;
        this.groundBody = hit.body;
        if (d > skin) this.position.addScaled(up, -(d - skin));
        return;
      }
      // 先碰到的是陡峭的棱（例如圆底贴着台阶边缘）：从中心竖直向下找真正的地面
      const toBottom = this.position.dot(up) - this.bottomHeight();
      tmpV3.copy(up).negate();
      const ray = this.world.raycast(this.position, tmpV3, toBottom + distance, this.queryFilter);
      if (ray && this.isWalkable(ray.normal)) {
        this.isGrounded = true;
        this.groundNormal.copy(ray.normal);
        this.groundPoint.copy(ray.point);
        this.groundCollider = ray.collider;
        this.groundBody = ray.body;
        // 向下贴地不能越过先碰到的棱
        const snapDown = Math.min(ray.distance - toBottom, d) - skin;
        if (snapDown > 0) this.position.addScaled(up, -snapDown);
        return;
      }
    }
    this.isGrounded = false;
    this.groundNormal.copy(up);
    this.groundCollider = null;
    this.groundBody = null;
  }

  /** 记录碰撞并推动动态刚体 */
  private recordHit(
    hit: ShapeCastHit,
    normal: Readonly<Vec3>,
    dir: Readonly<Vec3>,
    remainingLength: number,
    mode: SlideMode,
    dt: number,
    walkable: boolean,
  ): void {
    if (mode === SlideMode.Down && walkable) return; // 地面由 updateGround 处理
    this.hits.push({
      collider: hit.collider,
      body: hit.body,
      point: hit.point.clone(),
      normal: new Vec3().copy(normal),
    });
    const body = hit.body;
    if (!body.isDynamic() || this.pushStrength <= 0 || dt <= 0) return;
    // 按角色质量推动：冲量 = m · (被挡住的速度沿法线的分量)
    const into = -dir.dot(normal) * (remainingLength / dt);
    if (into <= 0) return;
    const m = Math.min(this.mass, body.mass * 4);
    tmpV.copy(normal).scale(-m * into * this.pushStrength * 0.5);
    // 水平推动，避免把物体压进地面
    if (mode === SlideMode.Horizontal) tmpV.addScaled(this.up, -tmpV.dot(this.up));
    body.applyImpulse(tmpV, hit.point);
  }

  /** 穿透恢复：与重叠的物体逐个计算最深穿透并推出 */
  private depenetrate(): void {
    const xf = tmpXf;
    for (let iter = 0; iter < 4; iter++) {
      xf.position.copy(this.position);
      xf.rotation.copy(this.rotation);
      this.shape.computeAABB(xf, tmpAabb);
      let moved = false;
      for (const collider of this.world.queryAABB(tmpAabb, this.queryFilter)) {
        xf.position.copy(this.position);
        const depth = this.penetration(collider, xf, tmpV);
        if (depth > this.world.linearSlop) {
          this.position.addScaled(tmpV, depth);
          moved = true;
        }
      }
      if (!moved) break;
    }
  }

  /** 角色与碰撞体的最深穿透深度（> 0）与推出方向（写入 outDir） */
  private penetration(collider: Collider, xf: Readonly<Transform>, outDir: Vec3): number {
    const config: CollideConfig = { speculativeDistance: 0, linearSlop: this.world.linearSlop };
    const other = collider.shape;
    let deepest = 0;
    if (other.type === ShapeType.TriMesh || other.type === ShapeType.Heightfield) {
      collideMesh(
        other as MeshShape,
        collider.worldTransform,
        this.shape,
        xf,
        true,
        manifoldList,
        config,
      );
      for (let mi = 0; mi < manifoldList.manifoldCount; mi++) {
        const m = manifoldList.manifolds[mi]!;
        const s = m.minSeparation();
        if (-s > deepest) {
          deepest = -s;
          outDir.copy(m.normal).negate();
        }
      }
      return deepest;
    }
    collideShapes(this.shape, xf, other, collider.worldTransform, manifold, config);
    if (manifold.pointCount === 0) return 0;
    const s = manifold.minSeparation();
    if (s >= 0) return 0;
    outDir.copy(manifold.normal).negate();
    return -s;
  }
}

/** 角色查询使用的过滤：排除角色自身的刚体，再叠加用户的过滤条件 */
class CharacterQueryFilter implements QueryFilter {
  readonly includeSensors = false;
  constructor(private readonly owner: CharacterController) {}
  get categoryBits(): number | undefined {
    return this.owner.filter?.categoryBits;
  }
  get maskBits(): number | undefined {
    return this.owner.filter?.maskBits;
  }
  get excludeBody(): RigidBody | undefined {
    return this.owner.body ?? this.owner.filter?.excludeBody;
  }
  readonly predicate = (c: Collider): boolean => {
    const owner = this.owner;
    if (c.body === owner.body) return false;
    const user = owner.filter;
    if (user?.excludeBody && c.body === user.excludeBody) return false;
    return user?.predicate ? user.predicate(c) : true;
  };
}

const hDir = new Vec3();

/** 角色上一次的水平移动方向（用于台阶边缘的地面判定） */
function horizontalDir(c: CharacterController): Vec3 {
  hDir.copy(c.velocity).addScaled(c.up, -c.velocity.dot(c.up));
  if (hDir.normalize() < 1e-9) hDir.setZero();
  return hDir;
}
