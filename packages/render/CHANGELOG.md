# @thunder/render 更新日志

格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## 0.1.0

首个公开版本（依赖 three.js ≥ 0.170 与 @thunder/physics）。

- `RenderPipeline`：渲染器 + 场景 + 光源 + 后期的一站式封装
- `LightRig`：主光阴影（跟随相机、纹素对齐）、半球光、环境光、IBL，点光源 / 聚光灯可绑定到刚体，光照预设
- `PostProcessor`：色调映射、曝光、白平衡、对比度、饱和度、Lift / Gamma / Gain、暗角、3D LUT，调色预设
- `PhysicsView`：按碰撞形状生成网格（含三角网格与高度场）、渲染插值、刚体外观
- `ThreeDebugRenderer`：物理调试线框
- `createTriMeshFromObject` / `createConvexHullFromObject` / `convexHullFromPositions`：由 three.js 对象（例如 glTF 模型）或顶点生成碰撞体
- `PhysicsCar` / `createPhysicsCar`：按节点名把车模绑定为物理驱动的汽车（车身、车轮、刹车卡钳、方向盘、车门、引擎盖、后备箱、天窗、尾翼、雨刮、车灯）
