import { AABB } from '../math/AABB';
import type { Transform } from '../math/Transform';
import { Vec3 } from '../math/Vec3';
import { type MassProperties, Shape, type ShapeRayHit } from './Shape';

/**
 * 由三角形组成的静态几何（三角网格、高度场）的公共基类。
 *
 * 只能挂在静态或运动学刚体上（没有体积，不参与质量计算）。
 * 碰撞时按其他形状的包围盒查询相交的三角形，逐个三角形生成接触。
 */
export abstract class MeshShape extends Shape {
  readonly radius = 0;
  /** 为 true 时两面都会碰撞；默认只有正面（逆时针为正面）碰撞 */
  abstract readonly doubleSided: boolean;
  /** 局部包围盒 */
  readonly localBounds = new AABB();

  abstract readonly triangleCount: number;

  /** 读取第 index 个三角形的三个顶点（局部坐标） */
  abstract getTriangle(index: number, a: Vec3, b: Vec3, c: Vec3): void;

  /** 第 index 个三角形的活跃棱标志（见 meshUtils 中的 EDGE_*） */
  abstract getEdgeFlags(index: number): number;

  /**
   * 遍历与局部包围盒相交的三角形，回调返回 false 时提前结束。
   */
  abstract queryTriangles(
    localAabb: Readonly<AABB>,
    callback: (index: number) => boolean | void,
  ): void;

  getCorePoints(): readonly Vec3[] {
    return NO_POINTS;
  }

  computeAABB(transform: Readonly<Transform>, out: AABB): AABB {
    // 变换局部包围盒的 8 个角点
    const mn = this.localBounds.min;
    const mx = this.localBounds.max;
    out.makeEmpty();
    for (let i = 0; i < 8; i++) {
      corner.set(i & 1 ? mx.x : mn.x, i & 2 ? mx.y : mn.y, i & 4 ? mx.z : mn.z);
      transform.transformPoint(corner, corner);
      out.expandByPoint(corner);
    }
    return out;
  }

  computeMass(_density: number, out: MassProperties): MassProperties {
    out.mass = 0;
    out.center.setZero();
    out.inertia.setZero();
    return out;
  }

  getVolume(): number {
    return 0;
  }

  abstract override raycast(
    origin: Readonly<Vec3>,
    dir: Readonly<Vec3>,
    maxT: number,
    out: ShapeRayHit,
  ): boolean;
}

const NO_POINTS: readonly Vec3[] = [];
const corner = new Vec3();
