import * as THREE from 'three';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { LUTPass } from 'three/addons/postprocessing/LUTPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { ColorGradingShader } from './ColorGradingShader';
import {
  type ColorGradingSettings,
  type GradingPresetName,
  defaultGrading,
  resolveGrading,
  toneMappings,
  whiteBalance,
} from './grading';
import { getBuiltinLUT } from './lut';

type Uniforms = typeof ColorGradingShader.uniforms;

/** 把调色参数写入着色器 uniform（不依赖 WebGL，可单独测试） */
export function applyGradingUniforms(settings: Readonly<ColorGradingSettings>, u: Uniforms): void {
  whiteBalance(settings.temperature, settings.tint, u.whiteBalance.value);
  u.lift.value.fromArray(settings.lift);
  u.gamma.value.fromArray(settings.gamma);
  u.gain.value.fromArray(settings.gain);
  u.contrast.value = settings.contrast;
  u.saturation.value = settings.saturation;
  u.vignette.value = settings.vignette;
  u.vignetteSmoothness.value = THREE.MathUtils.clamp(settings.vignetteSmoothness, 0.01, 1);
}

/** 调色参数是否等价于“不做显示空间调整”，此时跳过调色通道以节省一次全屏绘制 */
export function isGradingIdentity(s: Readonly<ColorGradingSettings>): boolean {
  return (
    s.temperature === 0 &&
    s.tint === 0 &&
    s.contrast === 1 &&
    s.saturation === 1 &&
    s.vignette === 0 &&
    s.lift.every((v) => v === 0) &&
    s.gamma.every((v) => v === 1) &&
    s.gain.every((v) => v === 1)
  );
}

/**
 * 后期处理管线：RenderPass（线性 HDR）→ OutputPass（曝光 + 色调映射 + sRGB）
 * → 调色（白平衡 / LGG / 对比度 / 饱和度 / 暗角）→ LUT。
 * 渲染目标使用半精度浮点并开启 MSAA。
 */
export class PostProcessor {
  readonly composer: EffectComposer;
  readonly renderPass: RenderPass;
  readonly outputPass: OutputPass;
  readonly gradingPass: ShaderPass;
  readonly lutPass: LUTPass;
  private settings: ColorGradingSettings = resolveGrading({});

  constructor(
    readonly renderer: THREE.WebGLRenderer,
    scene: THREE.Scene,
    camera: THREE.Camera,
    options: { samples?: number } = {},
  ) {
    const size = renderer.getDrawingBufferSize(new THREE.Vector2());
    const target = new THREE.WebGLRenderTarget(size.x, size.y, {
      type: THREE.HalfFloatType,
      samples: options.samples ?? 4,
    });
    this.composer = new EffectComposer(renderer, target);
    this.renderPass = new RenderPass(scene, camera);
    this.outputPass = new OutputPass();
    this.gradingPass = new ShaderPass(ColorGradingShader);
    this.lutPass = new LUTPass({ intensity: 1 });
    this.composer.addPass(this.renderPass);
    this.composer.addPass(this.outputPass);
    this.composer.addPass(this.gradingPass);
    this.composer.addPass(this.lutPass);
    this.setGrading(defaultGrading);
  }

  get camera(): THREE.Camera {
    return this.renderPass.camera;
  }

  set camera(camera: THREE.Camera) {
    this.renderPass.camera = camera;
  }

  /** 当前调色参数（只读副本） */
  get grading(): Readonly<ColorGradingSettings> {
    return this.settings;
  }

  /**
   * 设置调色：传预设名称会整体替换为该预设；传部分参数则与当前参数合并。
   */
  setGrading(input: GradingPresetName | Partial<ColorGradingSettings>): void {
    this.settings = resolveGrading(input, this.settings);
    const s = this.settings;
    this.renderer.toneMapping = toneMappings[s.toneMapping];
    this.renderer.toneMappingExposure = s.exposure;
    applyGradingUniforms(s, this.gradingPass.uniforms as unknown as Uniforms);
    this.gradingPass.enabled = !isGradingIdentity(s);
    const lut = typeof s.lut === 'string' ? getBuiltinLUT(s.lut) : s.lut;
    this.lutPass.lut = lut ?? undefined;
    this.lutPass.intensity = s.lutIntensity;
    this.lutPass.enabled = !!lut && s.lutIntensity > 0;
  }

  setSize(width: number, height: number): void {
    this.composer.setSize(width, height);
    const u = this.gradingPass.uniforms as unknown as Uniforms;
    u.aspect.value = height > 0 ? width / height : 1;
  }

  setPixelRatio(ratio: number): void {
    this.composer.setPixelRatio(ratio);
  }

  render(deltaTime?: number): void {
    this.composer.render(deltaTime);
  }

  dispose(): void {
    this.composer.dispose();
    this.gradingPass.dispose();
    this.lutPass.dispose();
    this.outputPass.dispose();
  }
}
