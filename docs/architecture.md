# Thunder Physics 架构说明

## 模块分层

```mermaid
flowchart TB
  world[world/World<br/>模拟主循环 · 休眠 · 连续碰撞 · 事件]
  query[query<br/>射线 · AABB · 重叠 · 形状投射]
  debug[debug<br/>DebugDrawer]
  dynamics[dynamics<br/>RigidBody · Collider · ContactManager<br/>ContactSolver · Joints]
  narrow[collision/narrowphase<br/>SAT + 裁剪 · GJK · ShapeCast]
  broad[collision/broadphase<br/>DynamicAabbTree · BroadPhase]
  shapes[shapes<br/>Sphere · Capsule · Box · Cylinder · ConvexHull · Plane]
  math[math<br/>Vec3 · Quat · Mat3 · Transform · AABB]
  world --> dynamics
  world --> query
  world --> debug
  query --> narrow
  query --> broad
  dynamics --> narrow
  dynamics --> broad
  narrow --> shapes
  broad --> math
  shapes --> math
```

下层不依赖上层；`RigidBody` 等对 `World` 只有类型依赖。

## 一步模拟（`World.step(dt)`）

```mermaid
flowchart LR
  A[宽相<br/>移动过的代理重新配对] --> B[窄相<br/>更新接触流形 · 匹配冲量]
  B --> C[求解<br/>子步循环]
  C --> D[写回位姿<br/>休眠计时]
  D --> E[连续碰撞<br/>高速物体形状投射]
  E --> F[更新宽相<br/>扩展包围盒]
  F --> G[岛屿休眠]
  G --> H[派发事件]
```

1. **宽相**：只有扩展包围盒发生变化的代理才重新查询；静态与动态分两棵树，静态之间从不配对。新配对经过过滤（同一刚体、传感器规则、类别掩码、`collideConnected`）后创建 `Contact`。
2. **窄相**：扩展包围盒不再重叠的接触被销毁；双方都休眠 / 静止的接触跳过更新。其余接触重新生成流形，新的接触点先按特征编号、再按局部位置与上一帧匹配，继承冲量用于 warm start。开始 / 结束接触时排入事件队列，并唤醒被碰到的休眠刚体。
3. **求解**：见下一节。
4. **写回**：质心 = 起始质心 + 位移，旋转 = 相对旋转 × 起始旋转；按 `|v| + maxExtent·|ω|` 更新休眠计时。
5. **连续碰撞**：本步位移超过 `0.5 × minExtent` 的动态刚体，把碰撞体从起点沿位移投射到静态物体（`isBullet` 还包括动态物体），命中则回退到撞击时刻；起点已接触（t = 0）的命中交给推测接触处理。
6. **更新宽相**：包围盒超出扩展包围盒时重新插入，并沿速度方向预测一步。
7. **休眠**：用并查集把通过接触 / 关节相连的唤醒刚体分成岛屿，岛内所有刚体静止超过 `timeToSleep` 则整岛休眠。
8. **事件**：`step` 结束后才派发，回调中可以安全修改世界。

## Soft Step 求解器

参考 Box2D v3（Erin Catto，《Solver2D》系列文章）。设整步时长 `dt`，子步数 `n`，子步长 `h = dt / n`。

```
prepare joints; prepare contacts          // 锚点、雅可比、有效质量
for i in 1..n:
  integrate velocities                    // 重力、外力、阻尼、速度上限
  warm start joints; warm start contacts  // 施加上一次的累积冲量
  solve joints (useBias = true)
  solve contacts (useBias = true)
  integrate positions                     // 累积位移 dp 与相对旋转 dq
  relax joints (useBias = false)
  relax contacts (useBias = false)
apply restitution
```

### 软约束

给定刚度 `hertz`、阻尼比 `ζ`、子步长 `h`：

```
ω = 2π·hertz
a1 = 2ζ + hω,  a2 = hω·a1,  a3 = 1 / (1 + a2)
biasRate = ω / a1,  massScale = a2·a3,  impulseScale = a3
```

冲量：`λ = -m_eff · massScale · (Cdot + biasRate · C) - impulseScale · λ_acc`。
接触刚度取 `min(contactHertz, 0.25 × 子步频率)`，与静态物体的接触刚度加倍；关节刚度为接触刚度的 2 倍。

### 接触

- 锚点 `rA = p - cA`、`rB = p - cB` 在准备阶段固定，因此一步内雅可比为常量，可预先计算 `rA×n`、`IA⁻¹(rA×n)` 等，求解时只剩标量运算。
- 当前分离距离由子步内的位移与相对旋转得到：`s = dot(dpB - dpA + dqB·rB - dqA·rA, n) + (s0 - dot(rB - rA, n))`。
- `s > 0`：推测接触，偏置为 `s / h`，允许在本子步内恰好闭合间隙；`s ≤ 0`：软约束推出，推出速度不超过 `contactPushMaxVelocity`。
- 摩擦：两个切向同时求解，按圆盘（`μ·λn`）截断。
- 弹性：求解前记录法向相对速度，所有子步之后若接近速度超过阈值且产生过法向冲量，则施加 `-m_eff·(vn + e·v0)`。
- 求解与 relax 两个阶段以相反顺序遍历接触点，抵消 Gauss-Seidel 顺序带来的不对称（否则对称堆叠也会缓慢侧倾）。

### 关节

关节锚点存储在刚体局部坐标系（相对刚体原点，求解时减去局部质心），子步内由 `center0 + dp`、`dq · rotation0` 得到当前位姿，位置误差用关节软约束修正，relax 阶段只做速度约束。

- 球窝：3×3 点约束块求解。
- 铰链：以创建时的相对旋转为基准，误差旋转分解为**扭转**（绕轴，即铰链角）与**摆动**（按旋转向量锁定为 0，2×2 块求解）；角度限制为推测式单边约束，超出限制时按圆周最近距离选择对应的限制。
- 焊接 / 滑动：相对旋转误差的旋转向量作为 3×3 角约束；滑动关节另有两条垂直方向的平移约束与轴向限制 / 马达。
- 鼠标关节：只作用于 bodyB 的软点约束，冲量按 `maxForce · h` 截断。

## 碰撞检测

### 宽相

`DynamicAabbTree` 移植自 Box2D：叶子存放扩展 `margin`（默认 0.1 m）的包围盒，小幅移动无需更新；插入时按表面积启发式选兄弟节点，回溯时做 AVL 旋转保持平衡。

### 窄相

形状分三类：平面、线段类（球 = 退化线段、胶囊）、多面体（长方体、圆柱、凸包）。

| 组合            | 算法                                                                                                                               |
| --------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| 线段类 - 线段类 | 线段最近点；近似平行的两根胶囊生成 2 个点                                                                                          |
| 多面体 - 线段类 | 核心分离时用 GJK 距离，若最近特征为面则把线段裁剪到面内得到 2 个点；核心相交时用 SAT（面 + 棱×线段，高斯图剪枝）                   |
| 多面体 - 多面体 | SAT：A 的面、B 的面、棱-棱（Gregorius 高斯图剪枝，预存棱方向与弧法线）；面接触时选入射面并用参考面侧平面做 Sutherland-Hodgman 裁剪 |
| 平面 - 任意     | 核心点 / 顶点对平面的距离                                                                                                          |

流形最多 4 个点：先取最深点，再取离它最远的点，再取与前两点构成最大三角形的点，最后取最能扩大面积的点。

凸包由增量算法（beneath-beyond）计算，再把共面三角形合并为多边形面，保证面接触能得到完整的裁剪多边形。圆柱用正 N 棱柱近似碰撞几何（与 cannon-es 相同），质量属性按真实圆柱计算。

### GJK 与形状投射

GJK 作用于“核心点集 + 半径”，使用 Johnson 子算法求单纯形最近点（含退化三角形 / 四面体处理）。形状投射使用保守推进：每次用 GJK 求最近距离 `d` 与法线 `n`，沿 `n` 的接近速度推进 `(d - target) / dot(v, n)`，由凸性保证不会越过接触。

## 确定性与性能约定

- 核心代码不使用随机数；刚体、碰撞体、接触都按创建顺序存放在数组中，迭代顺序确定。
- 数学对象可变、带 `out` 参数；热路径不分配对象，模块内复用临时变量。
- 物理量单位为米、千克、秒。

## 渲染层（`@thunder/render`）

物理引擎不依赖任何渲染库；`@thunder/render` 基于 three.js 实现画面，与物理世界的接口只有两处：`PhysicsView` 读取刚体与碰撞体生成网格、`LightRig.attachToBody` 读取刚体位姿驱动光源，二者都使用 `body.interpolate(alpha)` 与固定步长的物理保持平滑。

```mermaid
flowchart LR
  A[RenderPass<br/>线性 HDR · MSAA] --> B[OutputPass<br/>曝光 · 色调映射 · sRGB]
  B --> C[调色<br/>白平衡 · LGG · 对比度 · 饱和度 · 暗角]
  C --> D[LUTPass<br/>3D LUT]
```

- 曝光与色调映射作用于线性 HDR 画面；其余调色在色调映射之后的显示空间进行，参数与常见调色软件一致。
- 参数为中性时跳过调色通道，LUT 强度为 0 时跳过 LUT 通道。
- 主光阴影相机以相机注视点为中心，并在光源视平面内按纹素对齐（坐标轴与 three.js 阴影相机 `lookAt` 一致），避免移动时阴影闪烁。
