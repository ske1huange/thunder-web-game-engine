import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import {
  ColorGradingShader,
  applyGradingUniforms,
  builtinLUTs,
  createLUT,
  defaultGrading,
  gradingPresetNames,
  gradingPresets,
  isGradingIdentity,
  resolveGrading,
  toneMappings,
  whiteBalance,
} from '../src';

const luminance = (v: THREE.Vector3) => 0.2126 * v.x + 0.7152 * v.y + 0.0722 * v.z;

describe('白平衡', () => {
  it('中性时不改变颜色，调整后亮度保持不变', () => {
    const v = whiteBalance(0, 0, new THREE.Vector3());
    expect(v.toArray().map((x) => +x.toFixed(12))).toEqual([1, 1, 1]);
    for (const [t, g] of [
      [0.5, 0],
      [-0.7, 0.3],
      [1, -1],
    ]) {
      expect(luminance(whiteBalance(t!, g!, new THREE.Vector3()))).toBeCloseTo(1, 12);
    }
  });

  it('暖色偏红、冷色偏蓝、正色调减少绿色', () => {
    const warm = whiteBalance(0.5, 0, new THREE.Vector3());
    expect(warm.x).toBeGreaterThan(warm.z);
    const cool = whiteBalance(-0.5, 0, new THREE.Vector3());
    expect(cool.z).toBeGreaterThan(cool.x);
    const magenta = whiteBalance(0, 0.5, new THREE.Vector3());
    expect(magenta.y).toBeLessThan(magenta.x);
  });
});

describe('调色参数', () => {
  it('预设基于默认值解析，并且数组不与预设共享', () => {
    for (const name of gradingPresetNames) {
      const s = resolveGrading(name);
      expect(s.lift.length).toBe(3);
      expect(toneMappings[s.toneMapping]).toBeDefined();
      expect('label' in s).toBe(false);
    }
    const cine = resolveGrading('cinematic');
    expect(cine.contrast).toBe(gradingPresets.cinematic.contrast);
    expect(cine.saturation).toBe(gradingPresets.cinematic.saturation);
    expect(cine.lut).toBe('tealOrange');
    cine.lift[0] = 99;
    expect(gradingPresets.cinematic.lift[0]).toBe(0);
    // 预设未指定的项回到默认值
    const noir = resolveGrading('noir');
    expect(noir.temperature).toBe(0);
    expect(noir.saturation).toBe(0);
  });

  it('部分参数与当前参数合并', () => {
    const base = resolveGrading('warm');
    const merged = resolveGrading({ contrast: 1.4 }, base);
    expect(merged.contrast).toBe(1.4);
    expect(merged.temperature).toBe(gradingPresets.warm.temperature);
  });

  it('中性判定与 uniform 写入', () => {
    expect(isGradingIdentity(resolveGrading('neutral'))).toBe(true);
    expect(isGradingIdentity(resolveGrading('warm'))).toBe(false);
    const u = THREE.UniformsUtils.clone(
      ColorGradingShader.uniforms,
    ) as typeof ColorGradingShader.uniforms;
    const s = resolveGrading({
      ...defaultGrading,
      contrast: 1.2,
      saturation: 0.5,
      lift: [0.1, 0, 0],
      vignette: 0.4,
      temperature: 0.3,
    });
    applyGradingUniforms(s, u);
    expect(u.contrast.value).toBe(1.2);
    expect(u.saturation.value).toBe(0.5);
    expect(u.lift.value.x).toBe(0.1);
    expect(u.vignette.value).toBe(0.4);
    expect(u.whiteBalance.value.x).toBeGreaterThan(u.whiteBalance.value.z);
  });

  it('着色器源码包含全部 uniform', () => {
    for (const name of Object.keys(ColorGradingShader.uniforms)) {
      expect(ColorGradingShader.fragmentShader).toContain(`uniform`);
      expect(ColorGradingShader.fragmentShader).toContain(name);
    }
  });
});

describe('3D LUT', () => {
  it('恒等 LUT 在网格点上还原输入', () => {
    const size = 9;
    const lut = createLUT(builtinLUTs.identity, size);
    expect(lut.image.width).toBe(size);
    const data = lut.image.data as Uint8Array;
    const at = (r: number, g: number, b: number) => {
      const i = (r + g * size + b * size * size) * 4;
      return [data[i]!, data[i + 1]!, data[i + 2]!];
    };
    expect(at(0, 0, 0)).toEqual([0, 0, 0]);
    expect(at(8, 8, 8)).toEqual([255, 255, 255]);
    expect(at(8, 0, 4)).toEqual([255, 0, 128]);
  });

  it('内置 LUT 输出在 [0,1]，青橙色调让暗部偏青、亮部偏橙', () => {
    for (const fn of Object.values(builtinLUTs)) {
      for (const v of [0, 0.25, 0.5, 0.75, 1]) {
        for (const c of fn(v, v * 0.8, 1 - v)) {
          expect(Number.isFinite(c)).toBe(true);
        }
      }
    }
    const dark = builtinLUTs.tealOrange(0.15, 0.15, 0.15);
    expect(dark[2]).toBeGreaterThan(dark[0]);
    const bright = builtinLUTs.tealOrange(0.85, 0.85, 0.85);
    expect(bright[0]).toBeGreaterThan(bright[2]);
    const sepia = builtinLUTs.sepia(0.5, 0.5, 0.5);
    expect(sepia[0]).toBeGreaterThan(sepia[2]);
  });
});
