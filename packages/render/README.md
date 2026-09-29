# @thunder/render

Thunder Render —— 基于 three.js 的渲染层：多光源与阴影、后期调色，以及与 `@thunder/physics` 物理世界的同步。

依赖（peerDependencies）：`three >= 0.170`、`@thunder/physics`。

```bash
npm install @thunder/render @thunder/physics three
```

## 渲染管线

```ts
import * as THREE from 'three';
import { RenderPipeline } from '@thunder/render';

const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 500);
const pipeline = new RenderPipeline(camera, {
  container: document.body, // 画布插入的位置（也可传 canvas）
  lighting: 'studio', // 光照预设
  grading: 'neutral', // 调色预设
  shadowQuality: 'medium', // off | low | medium | high
  environment: true, // 环境贴图（IBL）
  postprocessing: true, // 关闭后直接渲染（仍保留色调映射）
});

pipeline.setSize(window.innerWidth, window.innerHeight);
pipeline.render(focus, alpha); // focus：阴影跟随中心；alpha：物理插值系数
```

## 光源（`pipeline.lights`，`LightRig`）

| 成员                                               | 说明                                                               |
| -------------------------------------------------- | ------------------------------------------------------------------ |
| `sun` / `hemisphere` / `ambient`                   | 主光（平行光）、半球天光、环境光                                   |
| `applyPreset(name, scene?)`                        | 应用光照预设，传 scene 时一并设置背景、雾与环境贴图强度            |
| `setSunAngles(azimuth, elevation)`                 | 主光方向（度）                                                     |
| `setShadowQuality(q)`                              | `off` / `low`（1024）/ `medium`（2048）/ `high`（4096）            |
| `addPointLight(opts)` / `addSpotLight(opts)`       | 添加点光源 / 聚光灯（颜色、强度、距离、衰减、阴影…）               |
| `attachToBody(light, body, { offset, direction })` | 光源绑定到刚体，随刚体（插值后的）位姿移动旋转；刚体销毁后自动解除 |
| `removeLight(light)` / `clearLights()`             | 移除光源                                                           |
| `enableEnvironment(renderer, scene)`               | 由 RoomEnvironment 生成 PMREM 环境贴图                             |
| `update(focus, alpha)`                             | 每帧调用（`RenderPipeline.render` 已包含）                         |

主光阴影以 `focus` 为中心、半宽 `halfExtent`（默认 25 m）覆盖，并按阴影贴图纹素对齐，相机移动时阴影边缘不闪烁。

光照预设：

| 名称       | 说明                           |
| ---------- | ------------------------------ |
| `studio`   | 室内：深色背景，中性白光       |
| `day`      | 白天：高角度暖白阳光、蓝色天空 |
| `sunset`   | 黄昏：低角度橙色阳光、长阴影   |
| `night`    | 夜晚：微弱月光，适合配合点光源 |
| `overcast` | 阴天：以天光为主、阴影很淡     |

> 点光源阴影需要渲染 6 个面，开销较大；光源放在刚体内部时，请用 `view.setAppearance(body, { castShadow: false })` 关闭该刚体的阴影，否则它会挡住自己的光。

## 调色（`pipeline.post`，`PostProcessor`）

处理顺序：场景渲染（线性 HDR，MSAA）→ 曝光 + 色调映射 + sRGB → 调色 → 3D LUT。

```ts
pipeline.setGrading('cinematic'); // 预设：整体替换
pipeline.setGrading({ contrast: 1.2, vignette: 0.4 }); // 部分参数：与当前合并
```

| 参数                              | 默认      | 说明                                                                   |
| --------------------------------- | --------- | ---------------------------------------------------------------------- |
| `toneMapping`                     | `aces`    | `none` / `linear` / `reinhard` / `cineon` / `aces` / `agx` / `neutral` |
| `exposure`                        | 1         | 曝光倍数                                                               |
| `temperature` / `tint`            | 0         | 白平衡：色温（-1 冷 .. 1 暖）/ 色调（-1 绿 .. 1 品红），保持亮度       |
| `contrast` / `saturation`         | 1         | 对比度（以中灰为轴）/ 饱和度（0 为黑白）                               |
| `lift` / `gamma` / `gain`         | 0 / 1 / 1 | 暗部 / 中间调 / 亮部，各通道 RGB                                       |
| `vignette` / `vignetteSmoothness` | 0 / 0.5   | 暗角强度与柔和度                                                       |
| `lut` / `lutIntensity`            | null / 1  | 内置 LUT 名称或 `Data3DTexture`，及混合强度                            |

调色预设：`neutral` 中性、`cinematic` 电影感（青橙 LUT + 对比 + 暗角）、`warm` 暖色、`cool` 冷色、`vintage` 复古（低对比、提亮暗部、褐色）、`noir` 黑白（高对比 + 暗角）、`vivid` 鲜艳。

### 3D LUT

```ts
import { createLUT, getBuiltinLUT } from '@thunder/render';
import { LUTCubeLoader } from 'three/addons/loaders/LUTCubeLoader.js';

pipeline.setGrading({ lut: 'tealOrange', lutIntensity: 0.7 }); // 内置：identity / tealOrange / sepia / bleachBypass
pipeline.setGrading({ lut: createLUT((r, g, b) => [r, g * 0.9, b * 1.1]) }); // 程序化
const cube = await new LUTCubeLoader().loadAsync('film.cube');
pipeline.setGrading({ lut: cube.texture3D }); // 外部 .cube 文件
```

## 物理同步

```ts
import { PhysicsView, ThreeDebugRenderer } from '@thunder/render';

const view = new PhysicsView(world, { sleepingBrightness: 0.6 });
pipeline.scene.add(view.root);
view.setAppearance(body, {
  color: 0xffffff,
  emissive: 0xff0000,
  emissiveIntensity: 2,
  roughness: 0.3,
  metalness: 0.8,
});
view.sync(alpha); // 每帧：按插值系数同步位姿，自动增删网格

// 关卡模型由游戏自己渲染时，可以只用它生成碰撞体、不再重复显示
view.setAppearance(levelBody, { visible: false });

const debug = new ThreeDebugRenderer(); // DebugDrawer 的 three.js 实现
pipeline.scene.add(debug.object);
debug.begin();
world.debugDraw(debug, { contacts: true });
debug.end();
```

`PhysicsView` 会为三角网格（平直着色）与高度场（平滑法线）生成网格；双面三角网格使用双面材质。

## 从 three.js / glTF 生成碰撞体

```ts
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import {
  collectTriangles,
  createConvexHullFromObject,
  createTriMeshFromGeometry,
  createTriMeshFromObject,
} from '@thunder/render';

const gltf = await new GLTFLoader().loadAsync('level.glb');
scene.add(gltf.scene);

// 静态关卡：收集所有网格（含 InstancedMesh 的每个实例），缩放 / 镜像已烘焙
const level = world.createBody({ type: 'static' });
level.addCollider({
  shape: createTriMeshFromObject(gltf.scene, {
    relativeTo: null, // 世界坐标（默认是对象自身的局部坐标）
    filter: (mesh) => !mesh.name.endsWith('_nocollide'),
  }),
});

// 动态道具：用模型顶点生成近似凸包（maxPoints 控制复杂度）
const rock = world.createBody({ position });
rock.addCollider({ shape: createConvexHullFromObject(rockModel, { maxPoints: 32 }) });
```
