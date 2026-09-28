import type { ColorRepresentation } from 'three';

/** 光照预设：主光（太阳/月亮）、半球天光、环境光、背景与雾、环境贴图强度 */
export interface LightingPreset {
  /** 中文名称，用于界面显示 */
  label: string;
  sun: {
    color: ColorRepresentation;
    intensity: number;
    /** 方位角（度）：0 表示光从 +Z 方向照来，逆时针为正（从上往下看） */
    azimuth: number;
    /** 仰角（度）：0 为地平线，90 为正上方 */
    elevation: number;
    castShadow: boolean;
  };
  hemisphere: { sky: ColorRepresentation; ground: ColorRepresentation; intensity: number };
  ambient: { color: ColorRepresentation; intensity: number };
  background: ColorRepresentation;
  /** 线性雾（null 表示关闭） */
  fog: { color: ColorRepresentation; near: number; far: number } | null;
  /** 环境贴图（IBL）强度，影响反射与间接光 */
  environmentIntensity: number;
}

export const lightingPresets = {
  studio: {
    label: '室内',
    sun: { color: 0xffffff, intensity: 1.6, azimuth: 40, elevation: 58, castShadow: true },
    hemisphere: { sky: 0xdde6ff, ground: 0x30303a, intensity: 0.7 },
    ambient: { color: 0xffffff, intensity: 0 },
    background: 0x0f1115,
    fog: { color: 0x0f1115, near: 40, far: 120 },
    environmentIntensity: 0.35,
  },
  day: {
    label: '白天',
    sun: { color: 0xfff4e0, intensity: 2.4, azimuth: 35, elevation: 55, castShadow: true },
    hemisphere: { sky: 0xbfdcff, ground: 0x5a4f40, intensity: 0.9 },
    ambient: { color: 0xffffff, intensity: 0 },
    background: 0x9cc9ee,
    fog: { color: 0x9cc9ee, near: 50, far: 160 },
    environmentIntensity: 0.5,
  },
  sunset: {
    label: '黄昏',
    sun: { color: 0xff9b54, intensity: 2.6, azimuth: -65, elevation: 12, castShadow: true },
    hemisphere: { sky: 0xffb38a, ground: 0x2e2230, intensity: 0.45 },
    ambient: { color: 0x6a4a6e, intensity: 0.15 },
    background: 0x7a4a5c,
    fog: { color: 0x7a4a5c, near: 30, far: 110 },
    environmentIntensity: 0.25,
  },
  night: {
    label: '夜晚',
    sun: { color: 0x9db4ff, intensity: 0.35, azimuth: 130, elevation: 40, castShadow: true },
    hemisphere: { sky: 0x1c2640, ground: 0x07070c, intensity: 0.25 },
    ambient: { color: 0x2a3350, intensity: 0.1 },
    background: 0x05070d,
    fog: { color: 0x05070d, near: 25, far: 90 },
    environmentIntensity: 0.05,
  },
  overcast: {
    label: '阴天',
    sun: { color: 0xffffff, intensity: 0.7, azimuth: 20, elevation: 70, castShadow: true },
    hemisphere: { sky: 0xdfe5ec, ground: 0x6b6b6b, intensity: 1.5 },
    ambient: { color: 0xffffff, intensity: 0.1 },
    background: 0xb6bec7,
    fog: { color: 0xb6bec7, near: 30, far: 120 },
    environmentIntensity: 0.6,
  },
} satisfies Record<string, LightingPreset>;

export type LightingPresetName = keyof typeof lightingPresets;

export const lightingPresetNames = Object.keys(lightingPresets) as LightingPresetName[];
