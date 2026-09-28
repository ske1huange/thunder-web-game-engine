import * as THREE from 'three';

/** 颜色变换函数：输入输出均为显示空间（sRGB）[0, 1] 的 RGB */
export type ColorTransform = (r: number, g: number, b: number) => [number, number, number];

/**
 * 由颜色变换函数生成 3D LUT（Data3DTexture），可直接用于 LUTPass。
 * 这样无需外部 .cube 文件也能做风格化调色；外部 LUT 可用 three 的 LUTCubeLoader 加载。
 */
export function createLUT(transform: ColorTransform, size = 33): THREE.Data3DTexture {
  const data = new Uint8Array(size * size * size * 4);
  const max = size - 1;
  let i = 0;
  // 布局与 three 的 LUTCubeLoader 相同：r 变化最快，其次 g，最后 b
  for (let b = 0; b < size; b++) {
    for (let g = 0; g < size; g++) {
      for (let r = 0; r < size; r++) {
        const [or, og, ob] = transform(r / max, g / max, b / max);
        data[i++] = Math.round(THREE.MathUtils.clamp(or, 0, 1) * 255);
        data[i++] = Math.round(THREE.MathUtils.clamp(og, 0, 1) * 255);
        data[i++] = Math.round(THREE.MathUtils.clamp(ob, 0, 1) * 255);
        data[i++] = 255;
      }
    }
  }
  const texture = new THREE.Data3DTexture(data, size, size, size);
  texture.format = THREE.RGBAFormat;
  texture.type = THREE.UnsignedByteType;
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;
  texture.wrapS = THREE.ClampToEdgeWrapping;
  texture.wrapT = THREE.ClampToEdgeWrapping;
  texture.wrapR = THREE.ClampToEdgeWrapping;
  texture.generateMipmaps = false;
  texture.unpackAlignment = 1;
  texture.needsUpdate = true;
  return texture;
}

function luma(r: number, g: number, b: number): number {
  return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}

function smooth(x: number): number {
  const t = THREE.MathUtils.clamp(x, 0, 1);
  return t * t * (3 - 2 * t);
}

/** 内置的程序化 LUT */
export const builtinLUTs = {
  /** 恒等变换（用于测试 / 对比） */
  identity: (r: number, g: number, b: number): [number, number, number] => [r, g, b],
  /** 青橙色调：暗部偏青、亮部偏橙，电影感 */
  tealOrange: (r: number, g: number, b: number): [number, number, number] => {
    const l = luma(r, g, b);
    const shadow = 1 - smooth(l * 1.6);
    const highlight = smooth((l - 0.35) * 1.6);
    return [
      r + highlight * 0.08 - shadow * 0.06,
      g + highlight * 0.02 + shadow * 0.03,
      b - highlight * 0.07 + shadow * 0.07,
    ];
  },
  /** 褐色老照片 */
  sepia: (r: number, g: number, b: number): [number, number, number] => [
    r * 0.393 + g * 0.769 + b * 0.189,
    r * 0.349 + g * 0.686 + b * 0.168,
    r * 0.272 + g * 0.534 + b * 0.131,
  ],
  /** 漂白效果：降低饱和度并提高对比 */
  bleachBypass: (r: number, g: number, b: number): [number, number, number] => {
    const l = luma(r, g, b);
    const mix = (c: number) => {
      const overlay = l < 0.5 ? 2 * l * c : 1 - 2 * (1 - l) * (1 - c);
      return c + (overlay - c) * 0.6;
    };
    return [mix(r), mix(g), mix(b)];
  },
} satisfies Record<string, ColorTransform>;

export type BuiltinLUTName = keyof typeof builtinLUTs;

const cache = new Map<BuiltinLUTName, THREE.Data3DTexture>();

/** 获取（并缓存）内置 LUT 纹理 */
export function getBuiltinLUT(name: BuiltinLUTName): THREE.Data3DTexture {
  let t = cache.get(name);
  if (!t) {
    t = createLUT(builtinLUTs[name]);
    cache.set(name, t);
  }
  return t;
}
