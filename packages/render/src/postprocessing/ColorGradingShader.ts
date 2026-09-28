import * as THREE from 'three';

/**
 * 显示空间调色着色器（放在 OutputPass 之后，输入已经过色调映射与 sRGB 编码）：
 * 白平衡 → Lift/Gamma/Gain → 对比度 → 饱和度 → 暗角。
 */
export const ColorGradingShader = {
  name: 'ColorGradingShader',
  uniforms: {
    tDiffuse: { value: null as THREE.Texture | null },
    whiteBalance: { value: new THREE.Vector3(1, 1, 1) },
    lift: { value: new THREE.Vector3(0, 0, 0) },
    gamma: { value: new THREE.Vector3(1, 1, 1) },
    gain: { value: new THREE.Vector3(1, 1, 1) },
    contrast: { value: 1 },
    saturation: { value: 1 },
    vignette: { value: 0 },
    vignetteSmoothness: { value: 0.5 },
    aspect: { value: 1 },
  },
  vertexShader: /* glsl */ `
    varying vec2 vUv;
    void main() {
      vUv = uv;
      gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    }
  `,
  fragmentShader: /* glsl */ `
    uniform sampler2D tDiffuse;
    uniform vec3 whiteBalance;
    uniform vec3 lift;
    uniform vec3 gamma;
    uniform vec3 gain;
    uniform float contrast;
    uniform float saturation;
    uniform float vignette;
    uniform float vignetteSmoothness;
    uniform float aspect;
    varying vec2 vUv;

    void main() {
      vec4 texel = texture2D(tDiffuse, vUv);
      vec3 c = texel.rgb * whiteBalance;
      // Lift / Gamma / Gain（ASC CDL 风格）
      c = gain * (c + lift * (1.0 - c));
      c = pow(max(c, vec3(0.0)), 1.0 / max(gamma, vec3(1e-3)));
      // 以中灰为轴的对比度
      c = (c - 0.5) * contrast + 0.5;
      // 饱和度（Rec.709 亮度）
      float l = dot(c, vec3(0.2126, 0.7152, 0.0722));
      c = mix(vec3(l), c, saturation);
      // 暗角：按画面宽高比修正为圆形
      vec2 d = (vUv - 0.5) * vec2(aspect, 1.0);
      float dist = length(d) / length(vec2(aspect, 1.0) * 0.5);
      float edge = 1.0 - smoothstep(1.0 - vignetteSmoothness, 1.0 + 0.001, dist);
      c *= mix(1.0, edge, vignette);
      gl_FragColor = vec4(clamp(c, 0.0, 1.0), texel.a);
    }
  `,
};
