import * as THREE from 'three';
import { LightRig, type ShadowQuality, type SunShadowOptions } from './lighting/LightRig';
import { type LightingPreset, type LightingPresetName } from './lighting/presets';
import type { ColorGradingSettings, GradingPresetName } from './postprocessing/grading';
import { PostProcessor } from './postprocessing/PostProcessor';

export interface RenderPipelineOptions {
  /** 使用已有 canvas；不传则创建新 canvas */
  canvas?: HTMLCanvasElement;
  /** 创建的 canvas 会插入到该元素开头 */
  container?: HTMLElement;
  /** 最大像素比，默认 2 */
  maxPixelRatio?: number;
  /** 光照预设，默认 studio */
  lighting?: LightingPresetName | LightingPreset;
  /** 调色预设或参数，默认 neutral */
  grading?: GradingPresetName | Partial<ColorGradingSettings>;
  /** 阴影质量，默认 medium */
  shadowQuality?: ShadowQuality;
  sunShadow?: SunShadowOptions;
  /** 是否启用环境贴图（IBL），默认 true */
  environment?: boolean;
  /** 是否启用后期（关闭后直接渲染，色调映射仍由渲染器完成），默认 true */
  postprocessing?: boolean;
  /** 后期渲染目标的 MSAA 采样数，默认 4 */
  samples?: number;
}

/**
 * 渲染管线：WebGL 渲染器 + 场景 + 光源系统 + 后期调色，一站式配置。
 *
 * ```ts
 * const pipeline = new RenderPipeline(camera, { container, lighting: 'sunset', grading: 'cinematic' });
 * pipeline.scene.add(physicsView.root);
 * // 每帧：
 * pipeline.render(controls.target, alpha);
 * ```
 */
export class RenderPipeline {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly lights: LightRig;
  readonly post: PostProcessor;
  /** 是否使用后期；关闭时直接 renderer.render */
  postprocessing: boolean;
  private cameraRef: THREE.Camera;
  private readonly maxPixelRatio: number;

  constructor(camera: THREE.Camera, options: RenderPipelineOptions = {}) {
    this.cameraRef = camera;
    this.maxPixelRatio = options.maxPixelRatio ?? 2;
    this.postprocessing = options.postprocessing ?? true;
    this.renderer = new THREE.WebGLRenderer({
      canvas: options.canvas,
      // 后期使用带 MSAA 的离屏目标，画布本身无需抗锯齿
      antialias: !this.postprocessing,
      powerPreference: 'high-performance',
    });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    // three r18x 起 PCFSoftShadowMap 已移除，PCFShadowMap 通过 shadow.radius 控制柔和度
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio ?? 1, this.maxPixelRatio));
    if (!options.canvas && options.container) options.container.prepend(this.renderer.domElement);

    this.lights = new LightRig(options.sunShadow);
    this.scene.add(this.lights.group);
    this.post = new PostProcessor(this.renderer, this.scene, camera, { samples: options.samples });

    this.setShadowQuality(options.shadowQuality ?? 'medium');
    this.setLighting(options.lighting ?? 'studio');
    this.setGrading(options.grading ?? 'neutral');
    if (options.environment ?? true) this.lights.enableEnvironment(this.renderer, this.scene);
  }

  get camera(): THREE.Camera {
    return this.cameraRef;
  }

  set camera(camera: THREE.Camera) {
    this.cameraRef = camera;
    this.post.camera = camera;
  }

  get canvas(): HTMLCanvasElement {
    return this.renderer.domElement;
  }

  /** 应用光照预设（同时设置背景、雾与环境贴图强度） */
  setLighting(preset: LightingPresetName | LightingPreset): void {
    this.lights.applyPreset(preset, this.scene);
  }

  /** 设置调色：预设名称整体替换，部分参数与当前合并 */
  setGrading(grading: GradingPresetName | Partial<ColorGradingSettings>): void {
    this.post.setGrading(grading);
  }

  setShadowQuality(quality: ShadowQuality): void {
    this.lights.setShadowQuality(quality);
    this.renderer.shadowMap.enabled = quality !== 'off';
  }

  /** 画布尺寸（CSS 像素）；透视相机的宽高比会一并更新 */
  setSize(width: number, height: number): void {
    this.renderer.setSize(width, height, false);
    this.post.setPixelRatio(this.renderer.getPixelRatio());
    this.post.setSize(width, height);
    const cam = this.cameraRef;
    if (cam instanceof THREE.PerspectiveCamera) {
      cam.aspect = height > 0 ? width / height : 1;
      cam.updateProjectionMatrix();
    }
  }

  /**
   * 渲染一帧。
   * @param focus 主光阴影跟随的中心（一般为相机注视点）
   * @param alpha 物理插值系数（用于绑定在刚体上的光源）
   */
  render(focus?: THREE.Vector3Like, alpha = 1): void {
    this.lights.update(focus, alpha);
    if (this.postprocessing) this.post.render();
    else this.renderer.render(this.scene, this.cameraRef);
  }

  dispose(): void {
    this.post.dispose();
    this.lights.dispose();
    this.renderer.dispose();
  }
}
