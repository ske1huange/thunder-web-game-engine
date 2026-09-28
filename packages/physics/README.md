# @thunder/physics

Thunder Physics —— 面向 HTML5 游戏的纯 TypeScript 3D 刚体物理引擎，零运行时依赖。

产物：ESM（`dist/index.js`）、CommonJS（`dist/index.cjs`）、浏览器全局脚本（`dist/thunder-physics.global.js`，全局变量 `ThunderPhysics`）以及类型声明。

## 世界与模拟

```ts
import { World, Vec3 } from '@thunder/physics';

const world = new World({
  gravity: new Vec3(0, -9.81, 0), // 默认值
  subSteps: 4, // 每步子步数
  fixedTimeStep: 1 / 60, // advance() 使用的固定步长
});

world.step(1 / 60); // 推进固定时间
const alpha = world.advance(dt); // 用帧时间累加推进，返回渲染插值系数
```

常用选项：`contactHertz`（接触刚度，默认 30）、`contactDampingRatio`（10）、`jointDampingRatio`（2）、`contactPushMaxVelocity`（穿透推出速度上限，3 m/s）、`restitutionThreshold`（1 m/s）、`linearSlop`（0.005 m）、`enableSleep`、`timeToSleep`（0.5 s）、`enableContinuous`。

## 刚体与碰撞体

```ts
import {
  BoxShape,
  CapsuleShape,
  ConvexHullShape,
  CylinderShape,
  PlaneShape,
  Quat,
  SphereShape,
} from '@thunder/physics';

const body = world.createBody({
  type: 'dynamic', // 'static' | 'kinematic' | 'dynamic'
  position: new Vec3(0, 2, 0),
  rotation: new Quat().setFromEuler(0, Math.PI / 4, 0),
  linearDamping: 0,
  angularDamping: 0.05,
  isBullet: false, // 高速物体：连续碰撞也检测动态物体
});

// 一个刚体可以挂多个碰撞体（复合形状）
body.addCollider({
  shape: new BoxShape(new Vec3(1, 0.1, 0.1)),
  density: 1,
  friction: 0.6,
  restitution: 0,
});
body.addCollider({ shape: new SphereShape(0.3), position: new Vec3(1, 0, 0) });

body.applyImpulse(new Vec3(0, 5, 0)); // 作用于质心
body.applyForce(new Vec3(10, 0, 0), worldPoint); // 作用于某点
body.setLinearVelocity(new Vec3(1, 0, 0));
body.setTransform(position, rotation); // 瞬移
body.interpolate(alpha, outPosition, outRotation);
world.destroyBody(body);
```

形状：`SphereShape(radius)`、`CapsuleShape(radius, halfHeight)`（沿 Y 轴）、`BoxShape(halfExtents)`、`CylinderShape(radius, halfHeight, segments = 16)`（沿 Y 轴，碰撞几何为正多棱柱近似）、`ConvexHullShape(points)`、`PlaneShape()`（局部法线 +Y，只能用于静态刚体）。

碰撞过滤与 Box2D 相同：`filter: { categoryBits, maskBits, group }`；`isSensor: true` 的碰撞体只触发 `sensorEnter` / `sensorExit`，不产生碰撞响应。

运动学刚体可以用 `body.moveKinematic(targetPosition, targetRotation, dt)` 驱动。

## 关节

```ts
import {
  BallSocketJoint,
  DistanceJoint,
  FixedJoint,
  HingeJoint,
  MouseJoint,
  SliderJoint,
} from '@thunder/physics';

world.addJoint(new BallSocketJoint({ bodyA, bodyB, anchor: new Vec3(0, 5, 0) }));

world.addJoint(
  new HingeJoint({
    bodyA,
    bodyB,
    anchor,
    axis: new Vec3(0, 1, 0),
    enableLimit: true,
    lowerAngle: -Math.PI / 2,
    upperAngle: Math.PI / 2,
    enableMotor: true,
    motorSpeed: 2,
    maxMotorTorque: 100,
    enableSpring: false,
    springHertz: 1,
    springDampingRatio: 0.5,
    targetAngle: 0,
  }),
);

world.addJoint(
  new DistanceJoint({
    bodyA,
    bodyB,
    anchorA,
    anchorB,
    enableSpring: true,
    hertz: 2,
    dampingRatio: 0.5,
  }),
);
world.addJoint(
  new DistanceJoint({
    bodyA,
    bodyB,
    anchorA,
    anchorB,
    enableLimit: true,
    minLength: 0,
    maxLength: 3,
  }),
); // 绳索
world.addJoint(new FixedJoint({ bodyA, bodyB, anchor }));
world.addJoint(
  new SliderJoint({
    bodyA,
    bodyB,
    anchor,
    axis,
    enableLimit: true,
    lowerTranslation: 0,
    upperTranslation: 4,
  }),
);

// 拖拽：bodyA 传一个静态刚体
const drag = world.addJoint(
  new MouseJoint({ bodyA: ground, bodyB: body, anchor: hitPoint, maxForce: 1000 * body.mass }),
);
drag.setTarget(mouseWorldPoint);
world.removeJoint(drag);
```

所有关节默认 `collideConnected: false`（相连的两个刚体之间不碰撞）。

## 查询

```ts
world.raycast(origin, direction, maxDistance, filter); // 最近命中 | null
world.raycastAll(origin, direction, maxDistance, filter); // 按距离排序的全部命中
world.queryAABB(aabb, filter); // Collider[]
world.overlapShape(shape, transform, filter); // Collider[]
world.castShape(shape, transform, translation, filter); // { collider, fraction, point, normal } | null
```

`filter`：`{ categoryBits, maskBits, includeSensors, excludeBody, predicate }`。

## 事件

事件在 `step()` 结束后派发，回调中可以安全地创建 / 销毁刚体。

```ts
world.on('collisionStart', ({ bodyA, bodyB, point, normal, approachSpeed }) => {});
world.on('collisionEnd', ({ colliderA, colliderB }) => {});
world.on('sensorEnter', ({ sensor, visitor }) => {});
world.on('sensorExit', ({ sensor, visitor }) => {});
world.on('sleep', ({ body }) => {});
world.on('wake', ({ body }) => {});
```

## 调试绘制

实现 `DebugDrawer` 接口（`drawLine(from, to, color)`、可选 `drawPoint`），然后：

```ts
world.debugDraw(drawer, {
  shapes: true,
  aabbs: false,
  contacts: true,
  joints: true,
  centerOfMass: false,
});
```

three.js 实现示例见 [`examples/src/ThreeDebugRenderer.ts`](../../examples/src/ThreeDebugRenderer.ts)。

## 约定

- 单位：米、千克、秒；右手坐标系，默认重力沿 -Y
- 数学类型的实例方法会修改自身（与 three.js 相同），热路径中请复用对象
- 确定性：核心代码不使用随机数，所有迭代顺序确定；同一 JS 引擎下同样输入得到逐位相同的结果（跨浏览器不保证）
