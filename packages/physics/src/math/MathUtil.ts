/** 通用数学工具与常量 */

export const EPSILON = 1e-9;
export const PI = Math.PI;
export const TWO_PI = Math.PI * 2;

export function clamp(value: number, min: number, max: number): number {
  return value < min ? min : value > max ? max : value;
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

/** 把角度规范到 [-PI, PI] */
export function wrapAngle(angle: number): number {
  let a = angle % TWO_PI;
  if (a > PI) a -= TWO_PI;
  else if (a < -PI) a += TWO_PI;
  return a;
}

export function degToRad(deg: number): number {
  return (deg * PI) / 180;
}

export function radToDeg(rad: number): number {
  return (rad * 180) / PI;
}
