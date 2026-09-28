# thunder-web-game-engine

雷霆 web 游戏引擎 —— 面向 HTML5 游戏的 TypeScript 引擎套件。

当前包含：

| 包                                     | 说明                                                                    |
| -------------------------------------- | ----------------------------------------------------------------------- |
| [`@thunder/physics`](packages/physics) | **Thunder Physics**：纯 TypeScript 编写、零运行时依赖的 3D 刚体物理引擎 |
| [`@thunder/examples`](examples)        | 基于 Vite + three.js 的演示站（9 个 demo）                              |

## Thunder Physics 特性

参考了 Box2D v3 / Box3D、Jolt、Rapier、cannon-es、OimoPhysics、planck.js、matter.js 等公开项目（见 [docs/references.md](docs/references.md)）。

- **刚体**：动态 / 静态 / 运动学，一个刚体可挂多个碰撞体（复合形状），质量属性按密度自动计算（平行轴定理）
- **形状**：球、胶囊、长方体、圆柱、任意点集凸包（增量凸包 + 共面合并）、无限平面
- **碰撞检测**
  - 宽相：静态 / 动态双 AABB 树（移植 Box2D 动态树：扩展包围盒、SAH 插入、AVL 旋转）
  - 窄相：多面体 SAT（面 / 棱 + 高斯图剪枝）+ 参考面裁剪、GJK 距离、球 / 胶囊 / 平面专用算法；流形至多 4 点并带特征编号
  - 推测接触（speculative contact，推测距离随相对速度增大）
  - 连续碰撞：高速物体对静态物体做形状投射（`isBullet` 还会检测动态物体）
- **求解器**：Box2D v3 的 **Soft Step**：子步进 + 软约束 + relax + 分离的弹性阶段，warm start，二维库仑摩擦锥
- **关节**：球窝、铰链（角度限制 / 马达 / 弹簧）、距离（刚性杆 / 弹簧 / 绳索）、焊接、滑动（平移限制 / 直线马达）、鼠标拖拽
- **休眠**：并查集岛屿，整岛静止后休眠，接触 / 关节 / 施力时自动唤醒
- **查询**：射线（最近 / 全部）、AABB、形状重叠、形状投射，支持类别掩码、传感器、排除刚体与自定义过滤
- **事件**：`collisionStart` / `collisionEnd`（含接近速度）、`sensorEnter` / `sensorExit`、`sleep` / `wake`，在 step 结束后派发
- **工程**：固定步长累加器 + 渲染插值、确定性（同一环境下同样输入逐位相同）、调试线框输出接口、ESM / CJS / IIFE 三种产物 + 类型声明

## 快速开始

```bash
pnpm install
pnpm dev          # 启动演示站 http://localhost:5173
pnpm test         # 单元测试 + 物理行为测试
pnpm build        # 构建 packages/physics（dist/）
pnpm bench        # 基准测试
```

演示站操作：左键拖拽抓取物体，右键 / 滚轮调整视角，空格发射小球，`P` 暂停，`N` 单步，`R` 重置，`D` 调试线框。

## 使用示例

```ts
import { BoxShape, PlaneShape, Quat, Vec3, World } from '@thunder/physics';

const world = new World({ gravity: new Vec3(0, -9.81, 0) });

// 地面
const ground = world.createBody({ type: 'static' });
ground.addCollider({ shape: new PlaneShape() });

// 动态箱子
const box = world.createBody({ position: new Vec3(0, 5, 0) });
box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)), friction: 0.6, restitution: 0.1 });

// 事件
world.on('collisionStart', (e) => console.log('撞击速度', e.approachSpeed));

// 游戏循环：固定步长 + 渲染插值
const p = new Vec3();
const q = new Quat();
function frame(dt: number) {
  const alpha = world.advance(dt);
  box.interpolate(alpha, p, q);
  mesh.position.set(p.x, p.y, p.z); // 例如 three.js 网格
  mesh.quaternion.set(q.x, q.y, q.z, q.w);
}

// 射线检测
const hit = world.raycast(new Vec3(0, 10, 0), new Vec3(0, -1, 0), 100);
if (hit) console.log(hit.body, hit.point, hit.normal, hit.distance);
```

更多用法见 [packages/physics/README.md](packages/physics/README.md)，实现细节见 [docs/architecture.md](docs/architecture.md)。

## 性能

`pnpm bench` 的结果（Node 22，1/60 s，4 子步，**关闭休眠**、所有物体持续参与求解）：

| 场景            | 动态刚体 | 接触点 | 平均 ms/步 |
| --------------- | -------: | -----: | ---------: |
| 100 个箱子堆积  |      100 |   ~590 |       ~3.9 |
| 金字塔 20 层    |      210 |  ~2040 |        ~12 |
| 500 个箱子堆积  |      500 |  ~3020 |        ~28 |
| 1000 个箱子堆积 |     1000 |  ~6280 |        ~50 |
| 1000 个球堆积   |     1000 |   1000 |         ~9 |

实际游戏中静止的物体会进入休眠，几乎不占用求解时间（例如 256 个箱子的砖墙静止后每步约 0.1 ms）。

## 目录结构

```
packages/physics/        物理引擎
  src/math/              Vec3 / Quat / Mat3 / Transform / AABB
  src/shapes/            碰撞形状与凸多面体
  src/collision/         宽相（AABB 树）、窄相（SAT、GJK、形状投射）、接触流形
  src/dynamics/          刚体、碰撞体、接触管理、求解器、关节
  src/world/             World：模拟主循环、休眠、连续碰撞、事件
  src/query/             射线、AABB、重叠、形状投射查询
  src/debug/             调试绘制接口
  test/                  单元测试与物理行为测试（Vitest）
examples/                three.js 演示站
bench/                   基准测试
docs/                    架构说明与参考项目调研
```

## 路线图

- 性能：结构体数组（SoA）数据布局、约束图着色 + Web Worker 并行、WebAssembly SIMD
- 功能：三角网格与高度场（静态地形）、角色控制器（character mover）、锥形 / 6 自由度关节、序列化与快照回放
- 引擎其余部分：渲染、ECS、资源管理等包

## License

[MIT](LICENSE)
