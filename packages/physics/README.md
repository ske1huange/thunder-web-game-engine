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

形状：`SphereShape(radius)`、`CapsuleShape(radius, halfHeight)`（沿 Y 轴）、`BoxShape(halfExtents)`、`CylinderShape(radius, halfHeight, segments = 16)`（沿 Y 轴，碰撞几何为正多棱柱近似）、`ConvexHullShape(points)`、`PlaneShape()`（局部法线 +Y，只能用于静态刚体）、`TriMeshShape`、`HeightfieldShape`（见下一节）。

碰撞过滤与 Box2D 相同：`filter: { categoryBits, maskBits, group }`；`isSensor: true` 的碰撞体只触发 `sensorEnter` / `sensorExit`，不产生碰撞响应。

运动学刚体可以用 `body.moveKinematic(targetPosition, targetRotation, dt)` 驱动。

## 三角网格与高度场（关卡、地形）

静态几何用 `TriMeshShape` 或 `HeightfieldShape`，只能挂在静态或运动学刚体上（运动学网格可做移动平台）。

```ts
import { HeightfieldShape, TriMeshShape } from '@thunder/physics';

// 三角网格：顶点 [x, y, z, ...] + 下标（从外侧看逆时针为正面）
const level = world.createBody({ type: 'static' });
level.addCollider({
  shape: new TriMeshShape(positions, indices, {
    doubleSided: false, // 默认单面：从背面进入不会被推出
    weldTolerance: 1e-5, // 焊接重复顶点，用于识别相邻三角形
  }),
});

// 高度场：rows × cols 个采样（行沿 +Z，列沿 +X），局部坐标以原点为中心
const terrain = world.createBody({ type: 'static' });
const field = new HeightfieldShape({ heights, rows: 129, cols: 129, cellSize: 1, heightScale: 1 });
terrain.addCollider({ shape: field });
field.heightAt(x, z); // 局部坐标处的地面高度
```

- **BVH**：三角网格按中位数二分建立静态 BVH（TypedArray 平铺），查询与射线检测不分配内存；高度场按格子直接定位，射线用二维 DDA。
- **内部棱（ghost collision）**：构建时按相邻关系标记每条棱是否“活跃”（凸且不平坦）。物体碰到非活跃棱 / 顶点时，接触改用三角形面法线，或交给相邻三角形处理，因此箱子、球在拼接的地面上滑动、滚动不会被接缝绊住。
- **多流形**：与网格的接触按法线分组（夹角 < 5° 合并），每组至多 4 个点；`contact.manifolds[0..manifoldCount)`。
- **查询 / 连续碰撞**：射线、形状投射、重叠查询都支持网格；高速物体对网格做连续碰撞时只考虑质心即将穿过的三角形，避免在起伏地形上“卡顿”。

从 three.js / glTF 模型生成碰撞体见 `@thunder/render` 的 `createTriMeshFromObject`、`createConvexHullFromObject`。

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
