# @thunder/physics 更新日志

格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## 0.1.0

首个公开版本。

### 刚体与碰撞

- 动态 / 静态 / 运动学刚体，一个刚体可挂多个碰撞体，质量属性按密度自动计算
- 形状：球、胶囊、长方体、圆柱、凸包、无限平面、三角网格（`TriMeshShape`）、高度场（`HeightfieldShape`）
- 宽相：静态 / 动态双 AABB 树；窄相：多面体 SAT + 裁剪、GJK、平面与线段类专用算法
- 三角网格：BVH、单面 / 双面、内部棱（ghost collision）修正、多流形接触
- 推测接触、连续碰撞（形状投射，网格只对质心即将穿过的三角形投射）
- 接触缓存：相对位姿几乎不变的接触沿用上次的流形

### 求解器与关节

- Box2D v3 风格的 Soft Step 求解器：子步进、软约束、relax、分离的弹性阶段、warm start
- 关节：球窝、锥形扭转、铰链、距离、焊接、滑动、鼠标拖拽
- 并查集岛屿休眠

### 游戏功能

- `CharacterController`：碰撞并滑动、坡度限制、自动上台阶、贴地、移动平台、推动物体、穿透恢复
- `RaycastVehicle`：射线悬挂、轮胎摩擦与打滑、转向 / 驱动 / 制动、防侧翻
- `World.addController`：每步前后的自定义逻辑
- 查询：射线、AABB、形状重叠、形状投射，支持过滤
- 事件：碰撞开始 / 结束、传感器进入 / 离开、休眠 / 唤醒

### 工程

- Web Worker：`runPhysicsWorker` + `WorkerWorld`，可序列化的形状 / 刚体 / 关节描述
- 固定步长 + 渲染插值，确定性模拟
- ESM / CJS / IIFE（`ThunderPhysics` 全局变量）产物与类型声明，零运行时依赖
