# thunder-web-game-engine

雷霆 web 游戏引擎 —— 面向 HTML5 游戏的 TypeScript 引擎套件。

当前包含：

| 包                                     | 说明                                                                             |
| -------------------------------------- | -------------------------------------------------------------------------------- |
| [`@thunder/physics`](packages/physics) | **Thunder Physics**：纯 TypeScript 编写、零运行时依赖的 3D 刚体物理引擎          |
| [`@thunder/render`](packages/render)   | **Thunder Render**：基于 three.js 的渲染层——多光源与阴影、后期调色、物理世界同步 |
| [`@thunder/examples`](examples)        | 基于 Vite + three.js 的演示站（15 个 demo）                                      |

## Thunder Physics 特性

参考了 Box2D v3 / Box3D、Jolt、Rapier、cannon-es、OimoPhysics、planck.js、matter.js 等公开项目（见 [docs/references.md](docs/references.md)）。

- **刚体**：动态 / 静态 / 运动学，一个刚体可挂多个碰撞体（复合形状），质量属性按密度自动计算（平行轴定理）
- **形状**：球、胶囊、长方体、圆柱、任意点集凸包（增量凸包 + 共面合并）、无限平面
- **角色控制器**：碰撞并滑动、坡度限制、自动上台阶、贴地、移动平台、推动物体、穿透恢复
- **关卡与地形**：三角网格（BVH 加速、单面 / 双面、内部棱“幽灵碰撞”消除）、高度场（DDA 射线）、多流形接触；可由 three.js / glTF 模型直接生成碰撞体
- **碰撞检测**
  - 宽相：静态 / 动态双 AABB 树（移植 Box2D 动态树：扩展包围盒、SAH 插入、AVL 旋转）
  - 窄相：多面体 SAT（面 / 棱 + 高斯图剪枝）+ 参考面裁剪、GJK 距离、球 / 胶囊 / 平面专用算法；流形至多 4 点并带特征编号
  - 推测接触（speculative contact，推测距离随相对速度增大）
  - 连续碰撞：高速物体对静态物体做形状投射（`isBullet` 还会检测动态物体）
  - 接触缓存：相对位姿几乎不变的接触沿用上一次的流形，静止堆叠几乎不需要窄相
- **求解器**：Box2D v3 的 **Soft Step**：子步进 + 软约束 + relax + 分离的弹性阶段，warm start，二维库仑摩擦锥
- **关节**：球窝、锥形扭转（布娃娃：摆动锥 + 扭转范围 + 关节摩擦）、铰链（角度限制 / 马达 / 弹簧）、距离（刚性杆 / 弹簧 / 绳索）、焊接、滑动（平移限制 / 直线马达）、鼠标拖拽
- **射线车辆**：悬挂弹簧阻尼、轮胎侧向摩擦与打滑、转向 / 驱动 / 制动、防侧翻；`world.addController` 挂接每步前后的自定义逻辑
- **车身可动部件**：车门 / 引擎盖 / 后备箱（铰链锁止、马达开合、自由摆动或保持打开）、天窗 / 尾翼（滑动）、雨刮（往复），都是真实刚体，会被挡住、会被甩上
- **休眠**：并查集岛屿，整岛静止后休眠，接触 / 关节 / 施力时自动唤醒
- **查询**：射线（最近 / 全部）、AABB、形状重叠、形状投射，支持类别掩码、传感器、排除刚体与自定义过滤
- **事件**：`collisionStart` / `collisionEnd`（含接近速度）、`sensorEnter` / `sensorExit`、`sleep` / `wake`，在 step 结束后派发
- **Web Worker**：`WorkerWorld` 在 Worker 中运行整个世界，主线程通过可序列化的描述创建刚体 / 关节，位姿快照以可转移的 `Float64Array` 回传，事件与查询异步返回
- **工程**：固定步长累加器 + 渲染插值、确定性（同一环境下同样输入逐位相同）、调试线框输出接口、ESM / CJS / IIFE 三种产物 + 类型声明

## Thunder Render：光源与调色

物理引擎只负责模拟，画面由 `@thunder/render` 负责（依赖 three.js 与 `@thunder/physics`）：

- **光源**：主光（太阳 / 月亮）+ 半球天光 + 环境光 + 环境贴图（IBL）；任意数量的点光源 / 聚光灯，可**绑定到刚体**上随之移动旋转
- **阴影**：主光阴影以相机注视点为中心并做纹素对齐，大场景不截断、移动时不闪烁；阴影质量 off / low / medium / high，可调柔和度
- **光照预设**：室内、白天、黄昏、夜晚、阴天（同时设置背景与雾）
- **调色**：色调映射（ACES / AgX / Neutral 等）、曝光、白平衡（色温 / 色调）、对比度、饱和度、Lift / Gamma / Gain、暗角、3D LUT（内置程序化 LUT，也可加载 `.cube`）
- **调色预设**：中性、电影感、暖色、冷色、复古、黑白、鲜艳
- **物理同步**：`PhysicsView` 按碰撞形状生成网格、渲染插值、休眠变暗、单个刚体的外观（颜色 / 自发光 / 粗糙度 / 金属度 / 阴影）
- **物理驱动的车模**：`createPhysicsCar` 按 glTF 节点名绑定车身、车轮、刹车卡钳、方向盘、车门、引擎盖、后备箱、天窗、尾翼、雨刮与车灯，所有动作都由物理驱动（悬挂姿态、转向、开合、被甩上的车门）

```ts
import { PhysicsView, RenderPipeline } from '@thunder/render';

const pipeline = new RenderPipeline(camera, {
  container,
  lighting: 'sunset',
  grading: 'cinematic',
});
const view = new PhysicsView(world);
pipeline.scene.add(view.root);

// 发光球：自发光材质 + 绑定的点光源
view.setAppearance(ball, { emissive: 0xff5a36, emissiveIntensity: 3, castShadow: false });
pipeline.lights.attachToBody(
  pipeline.lights.addPointLight({ color: 0xff5a36, intensity: 12 }),
  ball,
);

// 每帧
const alpha = world.advance(dt);
view.sync(alpha);
pipeline.render(controls.target, alpha);
```

详见 [packages/render/README.md](packages/render/README.md)。演示站左上角可以切换光照与调色预设，「光源与调色」demo 展示了绑定在刚体上的点光源与聚光灯，「车模部件」demo 是一辆可以开、各部件都能动的跑车（右下角面板或数字键 1–8 控制部件）。

## 安装

```bash
npm install @thunder/physics                  # 物理引擎，零依赖
npm install @thunder/render three             # 可选：three.js 渲染层
```

不使用打包工具时，也可以直接引用浏览器脚本（全局变量 `ThunderPhysics`）：

```html
<script src="https://unpkg.com/@thunder/physics/dist/thunder-physics.global.js"></script>
```

## 开发

```bash
pnpm install
pnpm dev             # 启动演示站 http://localhost:5173
pnpm test            # 单元测试 + 物理行为测试
pnpm build           # 构建 packages/physics 与 packages/render（dist/）
pnpm bench           # 基准测试
pnpm size            # 包体积检查（gzip 预算）
pnpm check:packages  # 发布前检查（publint + npm pack 预演）
pnpm docs:api        # 生成 API 文档到 docs/api
```

## 发布到 npm

1. 在 npm 上创建（或确认拥有）`thunder` 组织，使 `@thunder/*` 作用域可以发布；
2. 生成 npm 自动化令牌（Automation token），在 GitHub 仓库 Settings → Secrets and variables → Actions 中添加为 `NPM_TOKEN`；
3. 更新 `packages/*/package.json` 的 `version` 与 `CHANGELOG.md`，合并到 main；
4. 打标签并推送：`git tag v0.1.0 && git push origin v0.1.0`。

[Release 工作流](.github/workflows/release.yml) 会重新运行检查、构建，确认标签与包版本一致后执行 `pnpm publish`（带 npm provenance 来源证明，`workspace:` 依赖会被替换为实际版本号）。

演示站操作：左键拖拽抓取物体，右键 / 滚轮调整视角，空格发射小球，`P` 暂停，`N` 单步，`R` 重置，`D` 调试线框；左上角可切换光照与调色预设。

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

| 场景                                      | 动态刚体 | 接触点 | 平均 ms/步 |
| ----------------------------------------- | -------: | -----: | ---------: |
| 100 个箱子堆积                            |      100 |   ~570 |       ~1.8 |
| 金字塔 20 层                              |      210 |  ~2360 |       ~7.4 |
| 500 个箱子堆积                            |      500 |  ~3050 |        ~17 |
| 1000 个箱子堆积                           |     1000 |  ~6320 |        ~31 |
| 1000 个球堆积                             |     1000 |   1000 |       ~9.5 |
| 64×64 高度场上 200 个箱子 / 球 / 胶囊堆积 |      200 |  ~1020 |        ~11 |

实际游戏中静止的物体会进入休眠，几乎不占用求解时间（例如 256 个箱子的砖墙静止后每步约 0.1 ms）。

## 目录结构

```
packages/physics/        物理引擎
  src/math/              Vec3 / Quat / Mat3 / Transform / AABB
  src/shapes/            碰撞形状、凸多面体、三角网格 / 高度场与 BVH
  src/collision/         宽相（AABB 树）、窄相（SAT、GJK、形状投射）、接触流形
  src/dynamics/          刚体、碰撞体、接触管理、求解器、关节
  src/world/             World：模拟主循环、休眠、连续碰撞、事件
  src/character/         角色控制器
  src/vehicle/           射线车辆、车身可动部件
  src/worker/            Web Worker：可序列化描述、Worker 宿主、主线程 WorkerWorld
  src/query/             射线、AABB、重叠、形状投射查询
  src/debug/             调试绘制接口
  test/                  单元测试与物理行为测试（Vitest）
packages/render/         渲染层（three.js）
  src/lighting/          LightRig：光源、阴影跟随、光照预设
  src/postprocessing/    PostProcessor：色调映射、调色着色器、3D LUT、调色预设
  src/physics/           PhysicsView（物理 → 网格同步）、调试线渲染、three.js → 碰撞体
  src/vehicle/           PhysicsCar：按节点名把车模绑定到物理
  src/RenderPipeline.ts  渲染器 + 场景 + 光源 + 后期的一站式封装
examples/                three.js 演示站
bench/                   基准测试
docs/                    架构说明与参考项目调研
```

## 路线图

- 性能：结构体数组（SoA）数据布局、约束图着色 + Web Worker 并行、WebAssembly SIMD
- 功能：6 自由度关节、世界快照与回放
- 渲染：级联阴影（CSM）、泛光（Bloom）、屏幕空间环境光遮蔽（SSAO）、实时调节面板
- 引擎其余部分：ECS、资源管理、音频等包

## License

[MIT](LICENSE)
