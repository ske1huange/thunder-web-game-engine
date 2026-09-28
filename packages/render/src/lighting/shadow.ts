import * as THREE from 'three';

/**
 * 由方位角、仰角（度）得到“指向光源”的单位方向。
 * 方位角 0 为 +Z，逆时针为正（从上往下看）；仰角 0 为地平线。
 */
export function directionFromAngles(
  azimuthDeg: number,
  elevationDeg: number,
  out: THREE.Vector3,
): THREE.Vector3 {
  // 仰角限制在 (−89°, 89°)，避免与阴影相机的 up 方向平行
  const el = THREE.MathUtils.degToRad(THREE.MathUtils.clamp(elevationDeg, -89, 89));
  const az = THREE.MathUtils.degToRad(azimuthDeg);
  return out.set(Math.cos(el) * Math.sin(az), Math.sin(el), Math.cos(el) * Math.cos(az));
}

const right = new THREE.Vector3();
const up = new THREE.Vector3();
const WORLD_UP = new THREE.Vector3(0, 1, 0);

/**
 * 平行光阴影的“纹素对齐”：把阴影中心在光源视平面内对齐到阴影贴图的纹素网格，
 * 使相机移动时阴影边缘不闪烁。坐标轴与 three.js 阴影相机 lookAt 的结果一致。
 *
 * @param focus 希望阴影覆盖的中心（世界坐标）
 * @param toLight 指向光源的单位方向
 * @param halfExtent 阴影正交相机的半宽（米）
 * @param mapSize 阴影贴图分辨率
 */
export function snapShadowCenter(
  focus: THREE.Vector3,
  toLight: THREE.Vector3,
  halfExtent: number,
  mapSize: number,
  out: THREE.Vector3,
): THREE.Vector3 {
  // 与 Matrix4.lookAt 相同：z = toLight，x = up × z，y = z × x
  right.crossVectors(WORLD_UP, toLight);
  if (right.lengthSq() < 1e-12) right.set(1, 0, 0);
  right.normalize();
  up.crossVectors(toLight, right);
  const texel = (2 * halfExtent) / mapSize;
  const x = focus.dot(right);
  const y = focus.dot(up);
  const sx = Math.round(x / texel) * texel;
  const sy = Math.round(y / texel) * texel;
  return out
    .copy(focus)
    .addScaledVector(right, sx - x)
    .addScaledVector(up, sy - y);
}
