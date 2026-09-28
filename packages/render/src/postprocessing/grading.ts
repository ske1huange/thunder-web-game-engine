import * as THREE from 'three';
import type { BuiltinLUTName } from './lut';

export type ToneMappingName =
  'none' | 'linear' | 'reinhard' | 'cineon' | 'aces' | 'agx' | 'neutral';

export const toneMappings: Record<ToneMappingName, THREE.ToneMapping> = {
  none: THREE.NoToneMapping,
  linear: THREE.LinearToneMapping,
  reinhard: THREE.ReinhardToneMapping,
  cineon: THREE.CineonToneMapping,
  aces: THREE.ACESFilmicToneMapping,
  agx: THREE.AgXToneMapping,
  neutral: THREE.NeutralToneMapping,
};

export type RGB = [number, number, number];

/**
 * 调色参数。曝光与色调映射作用于线性 HDR 画面，其余调整在色调映射之后的显示空间进行。
 */
export interface ColorGradingSettings {
  toneMapping: ToneMappingName;
  /** 曝光（线性倍数），默认 1 */
  exposure: number;
  /** 色温 -1（冷）.. 1（暖） */
  temperature: number;
  /** 色调 -1（偏绿）.. 1（偏品红） */
  tint: number;
  /** 对比度，1 为原样 */
  contrast: number;
  /** 饱和度，0 为黑白，1 为原样 */
  saturation: number;
  /** 暗部偏移（lift），各通道 -1..1，默认 0 */
  lift: RGB;
  /** 中间调伽马，各通道 > 0，默认 1 */
  gamma: RGB;
  /** 亮部增益（gain），各通道默认 1 */
  gain: RGB;
  /** 暗角强度 0..1 */
  vignette: number;
  /** 暗角柔和度 0..1 */
  vignetteSmoothness: number;
  /** 3D LUT：内置名称、自定义 Data3DTexture，或 null */
  lut: BuiltinLUTName | THREE.Data3DTexture | null;
  /** LUT 混合强度 0..1 */
  lutIntensity: number;
}

export const defaultGrading: Readonly<ColorGradingSettings> = Object.freeze<ColorGradingSettings>({
  toneMapping: 'aces',
  exposure: 1,
  temperature: 0,
  tint: 0,
  contrast: 1,
  saturation: 1,
  lift: [0, 0, 0],
  gamma: [1, 1, 1],
  gain: [1, 1, 1],
  vignette: 0,
  vignetteSmoothness: 0.5,
  lut: null,
  lutIntensity: 1,
});

export interface GradingPreset extends Partial<ColorGradingSettings> {
  /** 中文名称，用于界面显示 */
  label: string;
}

export const gradingPresets = {
  neutral: { label: '中性' },
  cinematic: {
    label: '电影感',
    exposure: 0.95,
    contrast: 1.18,
    saturation: 1.05,
    lift: [0, 0.012, 0.03],
    gain: [1.05, 1, 0.94],
    vignette: 0.35,
    lut: 'tealOrange',
    lutIntensity: 0.6,
  },
  warm: { label: '暖色', temperature: 0.35, tint: 0.05, saturation: 1.05, vignette: 0.15 },
  cool: { label: '冷色', temperature: -0.35, tint: -0.03, contrast: 1.05, vignette: 0.15 },
  vintage: {
    label: '复古',
    toneMapping: 'agx',
    contrast: 0.9,
    saturation: 0.7,
    temperature: 0.25,
    lift: [0.06, 0.05, 0.03],
    gain: [1, 0.97, 0.9],
    vignette: 0.45,
    vignetteSmoothness: 0.6,
    lut: 'sepia',
    lutIntensity: 0.35,
  },
  noir: {
    label: '黑白',
    contrast: 1.35,
    saturation: 0,
    exposure: 1.1,
    vignette: 0.55,
    lut: 'bleachBypass',
    lutIntensity: 0.5,
  },
  vivid: {
    label: '鲜艳',
    toneMapping: 'neutral',
    exposure: 1.1,
    contrast: 1.08,
    saturation: 1.35,
    vignette: 0.1,
  },
} satisfies Record<string, GradingPreset>;

export type GradingPresetName = keyof typeof gradingPresets;

export const gradingPresetNames = Object.keys(gradingPresets) as GradingPresetName[];

/** 把预设名称或部分参数解析为完整的调色参数（基于默认值） */
export function resolveGrading(
  input: GradingPresetName | Partial<ColorGradingSettings>,
  base: Readonly<ColorGradingSettings> = defaultGrading,
): ColorGradingSettings {
  let partial: Partial<ColorGradingSettings>;
  if (typeof input === 'string') {
    const { label: _label, ...rest } = gradingPresets[input] as GradingPreset;
    partial = rest;
    base = defaultGrading;
  } else {
    partial = input;
  }
  return {
    ...base,
    ...partial,
    lift: [...(partial.lift ?? base.lift)] as RGB,
    gamma: [...(partial.gamma ?? base.gamma)] as RGB,
    gain: [...(partial.gain ?? base.gain)] as RGB,
  };
}

/**
 * 白平衡：由色温与色调得到 RGB 乘数，并归一化保持亮度不变。
 * 暖色提高红、降低蓝；色调为正时减少绿（偏品红）。
 */
export function whiteBalance(temperature: number, tint: number, out: THREE.Vector3): THREE.Vector3 {
  const t = THREE.MathUtils.clamp(temperature, -1, 1);
  const g = THREE.MathUtils.clamp(tint, -1, 1);
  out.set(1 + 0.3 * t, 1 - 0.25 * g, 1 - 0.3 * t);
  const luminance = 0.2126 * out.x + 0.7152 * out.y + 0.0722 * out.z;
  return out.multiplyScalar(1 / luminance);
}
