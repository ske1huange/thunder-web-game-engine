import type { AABB } from '../math/AABB';
import { Mat3 } from '../math/Mat3';
import type { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import type { ConvexPolyhedron } from './ConvexPolyhedron';

export const ShapeType = {
  Sphere: 0,
  Capsule: 1,
  Box: 2,
  Cylinder: 3,
  ConvexHull: 4,
  Plane: 5,
} as const;
export type ShapeType = (typeof ShapeType)[keyof typeof ShapeType];

/** 质量属性：质心与惯性张量都在形状局部坐标系下，惯性张量关于质心 */
export class MassProperties {
  mass = 0;
  readonly center = new Vec3();
  readonly inertia = new Mat3().setZero();
}

/** 局部坐标系下的射线检测结果 */
export interface ShapeRayHit {
  /** 命中参数：命中点 = origin + t * dir */
  t: number;
  /** 命中面的外法线（局部坐标） */
  normal: Vec3;
}

/**
 * 碰撞形状基类。
 *
 * 所有有限形状都表示为“核心 + 半径”：球 = 点 + r，胶囊 = 线段 + r，多面体 = 顶点集 + 0。
 * GJK 距离计算只作用于核心点集，然后再扣除半径（与 Box2D 的做法一致）。
 */
export abstract class Shape {
  abstract readonly type: ShapeType;
  /** 核心半径（球/胶囊为半径，多面体为 0） */
  abstract readonly radius: number;
  /** 多面体形状的几何数据，非多面体为 null */
  readonly hull: ConvexPolyhedron | null = null;

  /** 核心点集（局部坐标），供 GJK 使用。平面返回空数组 */
  abstract getCorePoints(): readonly Vec3[];

  /** 计算给定变换下的世界包围盒 */
  abstract computeAABB(transform: Readonly<Transform>, out: AABB): AABB;

  /** 按密度计算质量属性 */
  abstract computeMass(density: number, out: MassProperties): MassProperties;

  /**
   * 局部坐标系下的射线检测（起点在形状内部时视为未命中）。
   * @param maxT 最大参数（命中点 = origin + t * dir）
   */
  abstract raycast(
    origin: Readonly<Vec3>,
    dir: Readonly<Vec3>,
    maxT: number,
    out: ShapeRayHit,
  ): boolean;

  /** 体积 */
  abstract getVolume(): number;
}

/** 多面体局部射线检测（逐面裁剪） */
export function raycastPolyhedron(
  hull: ConvexPolyhedron,
  origin: Readonly<Vec3>,
  dir: Readonly<Vec3>,
  maxT: number,
  out: ShapeRayHit,
): boolean {
  let tEnter = 0;
  let tExit = maxT;
  let enterFace = -1;
  const faces = hull.faces;
  for (let i = 0; i < faces.length; i++) {
    const f = faces[i]!;
    const denom = f.normal.dot(dir);
    const dist = f.d - f.normal.dot(origin); // > 0 表示起点在该面内侧
    if (denom === 0) {
      if (dist < 0) return false;
      continue;
    }
    const t = dist / denom;
    if (denom < 0) {
      if (t > tEnter) {
        tEnter = t;
        enterFace = i;
      }
    } else if (t < tExit) {
      tExit = t;
    }
    if (tEnter > tExit) return false;
  }
  if (enterFace < 0) return false;
  out.t = tEnter;
  out.normal.copy(faces[enterFace]!.normal);
  return true;
}

/** 按顶点计算多面体在给定变换下的包围盒 */
export function aabbOfPoints(
  points: readonly Vec3[],
  transform: Readonly<Transform>,
  radius: number,
  out: AABB,
): AABB {
  out.makeEmpty();
  for (const p of points) {
    transform.transformPoint(p, tmpP);
    out.expandByPoint(tmpP);
  }
  if (radius > 0) out.expandByScalar(radius);
  return out;
}

const tmpP = new Vec3();
