# 参考项目调研

创建本项目前调研了 GitHub 上主流的开源物理引擎（星数为 2026 年 9 月的数据）。

| 项目                                                                                                            |       Stars | 语言        | 类型    | 借鉴点                                                                                                                            |
| --------------------------------------------------------------------------------------------------------------- | ----------: | ----------- | ------- | --------------------------------------------------------------------------------------------------------------------------------- |
| [erincatto/box3d](https://github.com/erincatto/box3d)                                                           |        6.5k | C           | 3D      | Soft Step 求解器（子步进 + 软约束 + relax + 弹性分阶段）、推测接触、岛屿休眠、传感器 / 接触 / 移动事件、确定性                    |
| [erincatto/box2d](https://github.com/erincatto/box2d)（v3）                                                     |       10.4k | C           | 2D      | 同一套 Soft Step 设计、动态 AABB 树、一个刚体多个形状、碰撞过滤规则、连续碰撞中忽略起点已接触的命中                               |
| [pmndrs/cannon-es](https://github.com/pmndrs/cannon-es)                                                         |           — | TypeScript  | 3D      | 模块划分（math / shapes / collision / solver / constraints / world）、热循环不分配对象、圆柱用多棱柱近似、three.js 生态的接入方式 |
| [saharan/OimoPhysics](https://github.com/saharan/OimoPhysics)                                                   |         956 | Haxe → JS   | 3D      | BVH 宽相、GJK/EPA、多种关节、岛屿拆分休眠、射线 / 凸体投射                                                                        |
| [jrouwe/JoltPhysics.js](https://github.com/jrouwe/JoltPhysics.js)                                               |         574 | C++ → WASM  | 3D      | 对象层过滤、关节种类、摆动-扭转分解、确定性设计                                                                                   |
| [dimforge/rapier](https://github.com/dimforge/rapier)                                                           |        5.8k | Rust → WASM | 2D/3D   | RigidBody 与 Collider 分离的 API、事件队列、查询管线                                                                              |
| [piqnt/planck.js](https://github.com/piqnt/planck.js)                                                           |        5.3k | TypeScript  | 2D      | TypeScript 工程组织（common / collision / dynamics / serializer）、Box2D API 的 TS 化                                             |
| [liabru/matter-js](https://github.com/liabru/matter-js)                                                         |       18.4k | JavaScript  | 2D      | 上手简单的 API、内置调试渲染与演示站写法                                                                                          |
| [Prozi/detect-collisions](https://github.com/Prozi/detect-collisions)                                           |         263 | TypeScript  | 2D 碰撞 | BVH + SAT、射线与调试绘制                                                                                                         |
| [schteppe/cannon.js](https://github.com/schteppe/cannon.js) / [lo-th/Oimo.js](https://github.com/lo-th/Oimo.js) | 5.0k / 3.2k | JavaScript  | 3D      | 早期纯 JS 3D 引擎，API 形态参考                                                                                                   |

## 技术选型结论

1. **纯 TypeScript、零依赖**：与 cannon-es、planck.js 一样直接运行在浏览器与 Node 中，调试方便，无需 WASM 工具链；性能热点通过预计算雅可比、避免分配等方式优化，后续可按需引入 WASM SIMD。
2. **求解器采用 Box2D v3 / Box3D 的 Soft Step**：相比传统 PGS + Baumgarte，子步进 + 软约束在堆叠、质量比与关节链上更稳定，参数更直观（刚度 Hz、阻尼比）。
3. **窄相以 SAT + 裁剪为主、GJK 为辅**（Box2D / Box3D / Jolt 的做法）：面接触直接得到完整流形，穿透时不依赖 EPA；GJK 用于分离距离、传感器、查询与形状投射。
4. **API 形态参考 Rapier / cannon-es**：`World.createBody` + `body.addCollider`，关节以类实例加入世界，查询与事件集中在 `World` 上。

## 延伸阅读

- Erin Catto, _Solver2D_（Box2D 博客，2024）：各类求解器在堆叠 / 质量比 / 关节上的对比
- Erin Catto, _Releasing Box2D 3.0_（Box2D 博客，2024）
- Dirk Gregorius, _The Separating Axis Test between Convex Polyhedra_（GDC 2013）
- Christer Ericson, _Real-Time Collision Detection_（最近点、GJK 子算法）
- Gino van den Bergen, _Ray Casting against General Convex Objects_（形状投射）
